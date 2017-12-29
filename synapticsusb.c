/****
 * USB Synaptics device driver
 *
 *  Copyright (c) 2002 Rob Miller (rob@inpharmatica . co . uk)
 *  Copyright (c) 2003 Ron Lee (ron@debian.org)
 *	cPad driver for kernel 2.4
 *
 *  Copyright (c) 2004 Jan Steinhoff <jan.steinhoff@uni-jena.de>
 *  Copyright (c) 2004 Ron Lee (ron@debian.org)
 *	rewritten for kernel 2.6
 *  cPad character device part now in cpad.c
 *
 * Bases on: 	usb_skeleton.c v2.2 by Greg Kroah-Hartman (greg@kroah.com)
 *		drivers/hid/usbhid/usbmouse.c by Dmitry Torokhov <dtor@mail.ru>
 *		drivers/input/mouse/synaptics.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * Trademarks are the property of their respective owners.
 */

#include "kconfig.h"
#include "synusb-kcompat.h"

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,5)
#include <linux/kref.h>
#endif
#include <linux/usb.h>
#include <linux/mutex.h>
#include <linux/input.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
#include <linux/usb/input.h>
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,13)
#include <linux/usb_input.h>
#endif
#include <linux/workqueue.h>

#include "synapticsusb.h"
#include "cpad.h"

/*
 * input device code
 */

MODULE_PARM_DESC(xmin, "minimal horizontal finger position");
MODULE_PARM_DESC(xmax, "maximal horizontal finger position");
MODULE_PARM_DESC(ymin, "minimal vertical finger position");
MODULE_PARM_DESC(ymax, "maximal vertical finger position");
static int xmin = 1472;
static int xmax = 5472;
static int ymin = 1408;
static int ymax = 4448;
module_param(xmin, int, 0444);
module_param(xmax, int, 0444);
module_param(ymin, int, 0444);
module_param(ymax, int, 0444);

MODULE_PARM_DESC(btn_middle, "if set, cPad menu button is reported as middle button");
static int btn_middle = 1;
module_param(btn_middle, int, 0644);

static const char synusb_pad_name[] = "Synaptics USB TouchPad";
static const char synusb_stick_name[] = "Synaptics USB Styk";
static const char synusb_screen_name[] = "Synaptics USB TouchScreen";

static const char* synusb_get_name(struct synusb *synusb)
{
	switch (synusb->input_type) {
	case SYNUSB_PAD:
		return synusb_pad_name;
	case SYNUSB_STICK:
		return synusb_stick_name;
	case SYNUSB_SCREEN:
		return synusb_screen_name;
	}
	return NULL;
}

/* report tool_width for touchpads */
static inline void synusb_report_width(struct input_dev *idev, int pressure, int w)
{
	int num_fingers, tool_width;

	if (pressure > 0) {
		num_fingers = 1;
		tool_width = 5;
		switch (w) {
		case 0 ... 1:
			num_fingers = 2 + w;
			break;
		case 2:	                /* pen, pretend its a finger */
			break;
		case 4 ... 15:
			tool_width = w;
			break;
		}
	}
	else {
		num_fingers = 0;
		tool_width = 0;
	}

	input_report_abs(idev, ABS_TOOL_WIDTH, tool_width);

	input_report_key(idev, BTN_TOOL_FINGER, num_fingers == 1);
	input_report_key(idev, BTN_TOOL_DOUBLETAP, num_fingers == 2);
	input_report_key(idev, BTN_TOOL_TRIPLETAP, num_fingers == 3);
}

/* convert signed or unsigned 13 bit number to int */
static inline int us13_to_int(u8 high, u8 low, int has_sign) {
        int res;

        res = ((int)(high & 0x1f) << 8) | low;
        if (has_sign && (high & 0x10))
                res -= 0x2000;

        return res;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
static void synusb_input_callback(struct urb *urb)
#else
static void synusb_input_callback(struct urb *urb, struct pt_regs *regs)
#endif
{
	struct synusb *synusb = (struct synusb *)urb->context;
	u8 *data = urb->transfer_buffer;
	struct input_dev *idev = synusb->idev;
	int res, x, y, pressure;
	int is_stick = (synusb->input_type == SYNUSB_STICK) ? 1 : 0;

	if (urb->status) {
		if (synusb_urb_status_error(urb)) {
			err("nonzero read int status received: %d", urb->status);
			goto resubmit;
		}

		/* unlink urb, do not resubmit */
		return;
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
	input_regs(idev, regs);
#endif

	pressure = data[6];
	x = us13_to_int(data[2], data[3], is_stick);
	y = us13_to_int(data[4], data[5], is_stick);

	if (is_stick) {
		if (pressure > 6)
			input_report_key(idev, BTN_TOUCH, 1);
		if (pressure < 5)
			input_report_key(idev, BTN_TOUCH, 0);
		input_report_key(idev, BTN_TOOL_FINGER, pressure ? 1 : 0);
	} else {
		if (pressure > 30)
			input_report_key(idev, BTN_TOUCH, 1);
		if (pressure < 25)
			input_report_key(idev, BTN_TOUCH, 0);
		synusb_report_width(idev, pressure, data[0] & 0x0f);
		y = ymin + ymax - y;
	}

	if (pressure > 0) {
		input_report_abs(idev, ABS_X, x);
		input_report_abs(idev, ABS_Y, y);
	}
	input_report_abs(idev, ABS_PRESSURE, pressure);

	input_report_key(idev, BTN_LEFT, data[1] & 0x04);
	input_report_key(idev, BTN_RIGHT, data[1] & 0x01);
	input_report_key(idev, BTN_MIDDLE, data[1] & 0x02);
	if (synusb->has_display)
		input_report_key(idev, btn_middle ? BTN_MIDDLE : BTN_MISC, data[1] & 0x08);

	input_sync(idev);
resubmit:
	res = usb_submit_urb(urb, GFP_ATOMIC);
	if (res)
		err("usb_submit_urb int in failed with result %d", res);
}

/* Data must always be fetched from the int endpoint, otherwise the device
 * would reconnect to force driver reload. So this is always scheduled by probe.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void synusb_submit_int(void *work)
#else
static void synusb_submit_int(struct work_struct *work)
#endif
{
	struct synusb *synusb = container_of(work, struct synusb, isubmit.work);
	int res;

	res = usb_submit_urb(synusb->iurb, GFP_KERNEL);
	if (res)
		err ("usb_submit_urb int in failed with result %d", res);
}

static int synusb_init_input(struct synusb *synusb)
{
	struct input_dev *idev;
	struct usb_device *udev = synusb->udev;
	int is_stick = (synusb->input_type == SYNUSB_STICK) ? 1 : 0;
	int retval = -ENOMEM;

	idev = input_allocate_device();
	if (!idev) {
		err("Can not allocate input device");
		goto error;
	}

	set_bit(EV_ABS, idev->evbit);
	set_bit(EV_KEY, idev->evbit);

	if (is_stick) {
		input_set_abs_params(idev, ABS_X, -127, 127, 0, 0);
		input_set_abs_params(idev, ABS_Y, -127, 127, 0, 0);
		input_set_abs_params(idev, ABS_PRESSURE, 0, 16, 0, 0);
	} else {
		input_set_abs_params(idev, ABS_X, xmin, xmax, 0, 0);
		input_set_abs_params(idev, ABS_Y, ymin, ymax, 0, 0);
		input_set_abs_params(idev, ABS_PRESSURE, 0, 255, 0, 0);
		input_set_abs_params(idev, ABS_TOOL_WIDTH, 0, 15, 0, 0);
		set_bit(BTN_TOOL_DOUBLETAP, idev->keybit);
		set_bit(BTN_TOOL_TRIPLETAP, idev->keybit);
	}

	set_bit(BTN_TOUCH, idev->keybit);
	set_bit(BTN_TOOL_FINGER, idev->keybit);
	set_bit(BTN_LEFT, idev->keybit);
	set_bit(BTN_RIGHT, idev->keybit);
	set_bit(BTN_MIDDLE, idev->keybit);
	if (synusb->has_display)
		set_bit(BTN_MISC, idev->keybit);

	usb_make_path(udev, synusb->iphys, sizeof(synusb->iphys));
	strlcat(synusb->iphys, "/input0", sizeof(synusb->iphys));
	idev->phys = synusb->iphys;
	idev->name = synusb_get_name(synusb);
	usb_to_input_id(udev, &idev->id);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,22)
	idev->dev.parent = &synusb->interface->dev;
	input_set_drvdata(idev, synusb);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15)
	idev->cdev.dev = &synusb->interface->dev;
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,2)
	idev->dev = &synusb->interface->dev;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,22)
	retval = input_register_device(idev);
	if (retval) {
		err("Can not register input device");
		goto error;
	}
#else
	input_register_device(idev);
#endif
	synusb->idev = idev;

	synusb->interface->needs_remote_wakeup = 1;

	return 0;
error:
	if (idev)
		input_free_device(idev);

	return retval;
}


/*
 * initialization of usb data structures
 */

static int synusb_setup_iurb(struct synusb *synusb, struct usb_endpoint_descriptor *endpoint)
{
	char *buf;

	if (endpoint->wMaxPacketSize < 8)
		return 0;
	if (synusb->iurb) {
		pr_warning("More than one possible int in endpoint found.\n");
		return 0;
	}
	synusb->iurb = usb_alloc_urb(0, GFP_KERNEL);
	if (!synusb->iurb)
		return -ENOMEM;
	buf = usb_buffer_alloc(synusb->udev, 8, GFP_ATOMIC,
			       &synusb->iurb->transfer_dma);
	if (!buf)
		return -ENOMEM;
	usb_fill_int_urb(synusb->iurb, synusb->udev,
			 usb_rcvintpipe(synusb->udev,
					endpoint->bEndpointAddress),
			 buf, 8, synusb_input_callback,
			 synusb, endpoint->bInterval);
	synusb->iurb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	return 0;
}

static int synusb_check_int_setup(struct synusb *synusb)
{
	if (synusb->iurb)
		return 0;
	return -ENODEV;
}

static struct synusb_endpoint_table {
	__u8 dir;
	__u8 xfer_type;
	int  (*setup)(struct synusb *,
		      struct usb_endpoint_descriptor *);
} synusb_endpoints [] = {
	{ USB_DIR_IN,	USB_ENDPOINT_XFER_BULK,	cpad_setup_in },
	{ USB_DIR_OUT,	USB_ENDPOINT_XFER_BULK,	cpad_setup_out },
	{ USB_DIR_IN,	USB_ENDPOINT_XFER_INT,	synusb_setup_iurb },
	{ }
};

/* return entry index in synusb_endpoint_table that matches ep */
static inline int synusb_match_endpoint(struct usb_endpoint_descriptor *ep)
{
	int i;

	for (i=0; synusb_endpoints[i].setup; i++)
		if (( (ep->bEndpointAddress & USB_DIR_IN)
				== synusb_endpoints[i].dir		) &&
		    ( (ep->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
				== synusb_endpoints[i].xfer_type	))
			return i;
	return -ENODEV;
}

static int synusb_setup_endpoints(struct synusb *synusb)
{
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	int int_num = synusb->interface->cur_altsetting->desc.bInterfaceNumber;
	unsigned altsetting;
	int i, j, res;

	altsetting = min((unsigned) (synusb->has_display ? 2 : 1),
			 synusb->interface->num_altsetting);
	res = usb_set_interface(synusb->udev, int_num, altsetting);
	if (res) {
		err("Can not set alternate setting to %i, error: %i",
		    altsetting, res);
		return res;
	}

	res = cpad_alloc(synusb);
	if (res)
		return res;

	iface_desc = synusb->interface->cur_altsetting;
	for (i = 0; i < iface_desc->desc.bNumEndpoints; i++) {
		endpoint = &iface_desc->endpoint[i].desc;
		j = synusb_match_endpoint(endpoint);
		if (j >= 0) {
			res = synusb_endpoints[j].setup(synusb, endpoint);
			if (res)
				return res;
		}
	}

	res = synusb_check_int_setup(synusb);
	if (res)
		return res;

	res = cpad_check_setup(synusb);
	if (res)
		return res;

	return 0;
}

static void synusb_detect_type(struct synusb *synusb, const struct usb_device_id *id)
{
	int int_num = synusb->interface->cur_altsetting->desc.bInterfaceNumber;

	synusb->has_display = 0;
	if (id->idVendor == USB_VID_SYNAPTICS) {
		switch (id->idProduct) {
		case USB_DID_SYN_TS:
			synusb->input_type = SYNUSB_SCREEN;
			break;
		case USB_DID_SYN_STICK:
			synusb->input_type = SYNUSB_STICK;
			break;
		case USB_DID_SYN_COMP_TP:
			if (int_num == 1)
				synusb->input_type = SYNUSB_STICK;
			else
				synusb->input_type = SYNUSB_PAD;
			break;
		case USB_DID_SYN_CPAD:
			synusb->has_display = 1;
		default:
			synusb->input_type = SYNUSB_PAD;
		}
	} else {
		synusb->input_type = SYNUSB_PAD;
	}
}

void synusb_free_urb(struct urb *urb)
{
	if (!urb)
		return;
	usb_buffer_free(urb->dev, urb->transfer_buffer_length,
			urb->transfer_buffer, urb->transfer_dma);
	usb_free_urb(urb);
}

void synusb_delete(struct kref *kref)
{
	struct synusb *synusb = container_of(kref, struct synusb, kref);

	synusb_free_urb(synusb->iurb);
	if (synusb->idev)
		input_unregister_device(synusb->idev);

	cpad_free(synusb);

	usb_put_dev(synusb->udev);
	kfree(synusb);
}

static int synusb_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	struct synusb *synusb = NULL;
	struct usb_device *udev = interface_to_usbdev(interface);
	int retval = -ENOMEM;

	synusb = kzalloc(sizeof(*synusb), GFP_KERNEL);
	if (synusb == NULL) {
		err("Out of memory");
		goto error;
	}

	synusb->udev = usb_get_dev(udev);
	synusb->interface = interface;
	kref_init(&synusb->kref);
	usb_set_intfdata(interface, synusb);

	synusb_detect_type(synusb, id);

	retval = synusb_setup_endpoints(synusb);
	if (retval) {
		err("Can not set up endpoints, error: %i", retval);
		goto error;
	}

	retval = synusb_init_input(synusb);
	if (retval)
		goto error;

	retval = cpad_init(synusb);
	if (retval)
		goto error;

	INIT_DELAYED_WORK(&synusb->isubmit, synusb_submit_int);
	schedule_delayed_work(&synusb->isubmit, HZ/100);

	return 0;

error:
	if (synusb) {
		usb_set_intfdata(interface, NULL);
		kref_put(&synusb->kref, synusb_delete);
	}
	return retval;
}

static void synusb_disconnect(struct usb_interface *interface)
{
	struct synusb *synusb;

	synusb = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);

	cpad_disconnect(synusb);

	cancel_delayed_work_sync(&synusb->isubmit);

	usb_kill_urb(synusb->iurb);
	input_unregister_device(synusb->idev);
	synusb->idev = NULL;

	kref_put(&synusb->kref, synusb_delete);

	pr_info("Synaptics device disconnected.\n");
}


/*
 * suspend code
 */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,23)

static void synusb_draw_down(struct synusb *synusb)
{
	cancel_delayed_work_sync(&synusb->isubmit);
	usb_kill_urb(synusb->iurb);
}

static int synusb_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct synusb *synusb = usb_get_intfdata(intf);
	int res;

	if (!synusb)
		return 0;

	res = cpad_suspend(synusb);
	if (res)
		goto error;

	synusb_draw_down(synusb);
error:
	return res;
}

static int synusb_resume(struct usb_interface *intf)
{
	struct synusb *synusb = usb_get_intfdata(intf);
	int res;

	res = usb_submit_urb(synusb->iurb, GFP_ATOMIC);
	if (res) {
		err("usb_submit_urb int in failed during resume with result %d", res);
		goto error;
	}

	res = cpad_resume(synusb);
	if (res)
		synusb_draw_down(synusb);
error:
	return res;
}

static int synusb_reset_resume(struct usb_interface *intf)
{
	struct synusb *synusb = usb_get_intfdata(intf);
	int res;

	res = usb_submit_urb(synusb->iurb, GFP_ATOMIC);
	if (res) {
		err("usb_submit_urb int in failed during resume with result %d", res);
		goto error;
	}

	res = cpad_reset_resume(synusb);
	if (res)
		synusb_draw_down(synusb);
error:
	return res;
}

static int synusb_pre_reset(struct usb_interface *intf)
{
	struct synusb *synusb = usb_get_intfdata(intf);
	int res;

	res = cpad_pre_reset(synusb);
	if (res)
		goto error;

	synusb_draw_down(synusb);
error:
	return res;
}

static int synusb_post_reset(struct usb_interface *intf)
{
	struct synusb *synusb = usb_get_intfdata(intf);
	int res;

	res = usb_submit_urb(synusb->iurb, GFP_ATOMIC);
	if (res) {
		err("usb_submit_urb int in failed during post_reset with result %d", res);
		goto error;
	}

	res = cpad_post_reset(synusb);
	if (res)
		synusb_draw_down(synusb);
error:
	return res;
}

#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,23) */

/* the id table is filled via sysfs, so usbhid is always the default driver */
static struct usb_device_id synusb_idtable [] = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
	{ USB_DEVICE(USB_VID_SYNAPTICS, USB_DID_SYN_TP) },
	{ USB_DEVICE(USB_VID_SYNAPTICS, USB_DID_SYN_INT_TP) },
	{ USB_DEVICE(USB_VID_SYNAPTICS, USB_DID_SYN_CPAD) },
	{ USB_DEVICE(USB_VID_SYNAPTICS, USB_DID_SYN_TS) },
	{ USB_DEVICE(USB_VID_SYNAPTICS, USB_DID_SYN_STICK) },
	{ USB_DEVICE(USB_VID_SYNAPTICS, USB_DID_SYN_WP) },
	{ USB_DEVICE(USB_VID_SYNAPTICS, USB_DID_SYN_COMP_TP) },
	{ USB_DEVICE(USB_VID_SYNAPTICS, USB_DID_SYN_WTP) },
	{ USB_DEVICE(USB_VID_SYNAPTICS, USB_DID_SYN_DP) },
#endif
	{ }
};
MODULE_DEVICE_TABLE (usb, synusb_idtable);

struct usb_driver synusb_driver = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16)
	.owner =	THIS_MODULE,
#endif
	.name =		"synaptics-usb",
	.probe =	synusb_probe,
	.disconnect =	synusb_disconnect,
	.id_table =	synusb_idtable,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,23)
	.suspend =	synusb_suspend,
	.resume =	synusb_resume,
	.reset_resume = synusb_reset_resume,
	.pre_reset =	synusb_pre_reset,
	.post_reset =	synusb_post_reset,
	.supports_autosuspend = 1,
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,23) */
};

static int __init synusb_init(void)
{
	int result;

	result = usb_register(&synusb_driver);
	if (result)
		err("usb_register failed. Error number %d", result);
	else
		pr_info(DRIVER_DESC " " DRIVER_VERSION "\n");

	return result;
}

static void __exit synusb_exit(void)
{
	usb_deregister(&synusb_driver);
}

module_init(synusb_init);
module_exit(synusb_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
