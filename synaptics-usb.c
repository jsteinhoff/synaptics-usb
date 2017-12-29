/****
 * USB Synaptics Touchpad driver
 *
 *  Copyright (c) 2002 Rob Miller (rob@inpharmatica . co . uk)
 *  Copyright (c) 2003 Ron Lee (ron@debian.org)
 *	cPad driver for kernel 2.4
 *
 *  Copyright (c) 2004 Jan Steinhoff <jan.steinhoff@uni-jena.de>
 *  Copyright (c) 2004 Ron Lee (ron@debian.org)
 *	rewritten for kernel 2.6
 *
 * Bases on: 	usb_skeleton.c v2.2 by Greg Kroah-Hartman (greg@kroah.com)
 *		drivers/input/mouse/synaptics.c
 *		drivers/usb/input/usbmouse.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * Trademarks are the property of their respective owners.
 */

/* TODO:
 * only one open
 * cpad_light_off: actual_length & inbuffer backup
 * has_indata in write and ioctl
 * check coding style and comments
 */

#include "synusb-kcompat.h"

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)
#include <linux/config.h>
#endif
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <asm/uaccess.h>
#include <linux/usb.h>
#include <linux/mutex.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
#include <linux/usb/input.h>
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,13)
#include <linux/usb_input.h>
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,5)
#include <linux/kref.h>
#endif
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/moduleparam.h>
#include <linux/cpad.h>


#define DRIVER_VERSION	"v1.4"
#define DRIVER_AUTHOR	"Rob Miller (rob@inpharmatica . co . uk), "\
			"Ron Lee (ron@debian.org), "\
			"Jan Steinhoff <jan.steinhoff@uni-jena.de>"
#define DRIVER_DESC	"USB Synaptics touchpad Driver"

/* vendor and device IDs */
#define USB_VID_SYNAPTICS	0x06cb	/* Synaptics vendor ID */
#define USB_DID_SYN_TP		0x0001	/* Synaptics USB TouchPad */
#define USB_DID_SYN_INT_TP	0x0002	/* Synaptics Integrated USB TouchPad */
#define USB_DID_SYN_CPAD	0x0003	/* Synaptics cPad */
#define USB_DID_SYN_WP		0x0008	/* Synaptics USB WheelPad */
#define USB_DID_SYN_COMP_TP	0x0009	/* Synaptics Composite USB TouchPad */

static struct usb_device_id synusb_idtable [] = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24)
	{ USB_DEVICE(USB_VID_SYNAPTICS, USB_DID_SYN_TP) },
	{ USB_DEVICE(USB_VID_SYNAPTICS, USB_DID_SYN_INT_TP) },
	{ USB_DEVICE(USB_VID_SYNAPTICS, USB_DID_SYN_CPAD) },
	{ USB_DEVICE(USB_VID_SYNAPTICS, USB_DID_SYN_WP) },
	{ USB_DEVICE(USB_VID_SYNAPTICS, USB_DID_SYN_COMP_TP) },
#endif
	{ }
};
MODULE_DEVICE_TABLE (usb, synusb_idtable);


struct synusb {
	struct usb_device *	udev;
	struct usb_interface *	interface;
	struct kref		kref;

	int			is_cpad;
#ifdef CONFIG_USB_CPADDEV
	/* cPad character device */
	struct mutex		io_mutex;
	struct mutex		autoput_mutex;
	struct usb_anchor	submitted;
	struct urb		*in, *out;
	struct completion	cpad_done;
	int			has_indata;
	int			error;
	struct delayed_work	flash;
#endif /* CONFIG_USB_CPADDEV */

	/* input device */
	struct input_dev	*idev;
	char			iphys[64];
	struct delayed_work	isubmit;
	struct urb *		iurb;
};

static struct usb_driver synusb_driver;

static inline void synusb_free_urb(struct urb *urb)
{
	if (!urb)
		return;
	usb_buffer_free(urb->dev, urb->transfer_buffer_length,
			urb->transfer_buffer, urb->transfer_dma);
	usb_free_urb(urb);
}

static void synusb_delete(struct kref *kref)
{
	struct synusb *synusb = container_of(kref, struct synusb, kref);

	synusb_free_urb(synusb->iurb);
	if (synusb->idev)
		input_unregister_device(synusb->idev);

#ifdef CONFIG_USB_CPADDEV
	if (synusb->is_cpad) {
		synusb->out->transfer_buffer_length = 274*32;
		synusb_free_urb(synusb->in);
		synusb_free_urb(synusb->out);
	}
#endif /* CONFIG_USB_CPADDEV */

	usb_put_dev(synusb->udev);
	kfree(synusb);
}


/*
 * input device
 */

#define XMIN_NOMINAL 1472
#define XMAX_NOMINAL 5472
#define YMIN_NOMINAL 1408
#define YMAX_NOMINAL 4448

static int xmin = XMIN_NOMINAL;
static int xmax = XMAX_NOMINAL;
static int ymin = YMIN_NOMINAL;
static int ymax = YMAX_NOMINAL;
module_param(xmin, int, 0444);
module_param(xmax, int, 0444);
module_param(ymin, int, 0444);
module_param(ymax, int, 0444);

static int btn_middle = 1;
module_param(btn_middle, int, 0644);

static char synusb_input_name[] = "Synaptics USB touchpad";

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
static void synusb_input_callback(struct urb *urb)
#else
static void synusb_input_callback(struct urb *urb, struct pt_regs *regs)
#endif
{
	struct synusb *synusb = (struct synusb *)urb->context;
	u8 *data = urb->transfer_buffer;
	struct input_dev *idev = synusb->idev;
	int res, w, pressure, num_fingers, tool_width;

	switch (urb->status) {
	case 0:
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		return;
	default:
		goto resubmit;
	}

	w = data[0] & 0x0f;
	pressure = data[6];
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

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
	input_regs (idev, regs);
#endif

	if (pressure > 30) input_report_key (idev, BTN_TOUCH, 1);
	if (pressure < 25) input_report_key (idev, BTN_TOUCH, 0);

	if (pressure > 0) {
		input_report_abs (idev, ABS_X, (data[2] << 8) | data[3]);
		input_report_abs (idev, ABS_Y,
				  ymin + ymax - ((data[4] << 8) | data[5]));
	}
	input_report_abs (idev, ABS_PRESSURE, pressure);

	input_report_abs (idev, ABS_TOOL_WIDTH, tool_width);
	input_report_key (idev, BTN_TOOL_FINGER, num_fingers == 1);
	input_report_key (idev, BTN_TOOL_DOUBLETAP, num_fingers == 2);
	input_report_key (idev, BTN_TOOL_TRIPLETAP, num_fingers == 3);
	input_report_key (idev, BTN_LEFT, data[1] & 0x04);
	input_report_key (idev, BTN_RIGHT, data[1] & 0x01);
	if (synusb->is_cpad)
		input_report_key (idev, btn_middle ? BTN_MIDDLE : BTN_MISC,
				  data[1] & 0x08);
	input_sync (idev);
resubmit:
	res = usb_submit_urb (urb, GFP_ATOMIC);
	if (res)
		err("usb_submit_urb int in failed with result %d", res);
}

/* Data must always be fetched from the int endpoint, otherwise the touchpad
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
	int retval = -ENOMEM;

	idev = input_allocate_device();
	if (!idev) {
		err("Can not allocate input device");
		goto error;
	}

	set_bit(EV_ABS, idev->evbit);
	input_set_abs_params(idev, ABS_X, XMIN_NOMINAL, XMAX_NOMINAL, 0, 0);
	input_set_abs_params(idev, ABS_Y, YMIN_NOMINAL, YMAX_NOMINAL, 0, 0);
	input_set_abs_params(idev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(idev, ABS_TOOL_WIDTH, 0, 15, 0, 0);

	set_bit(EV_KEY, idev->evbit);
	set_bit(BTN_TOUCH, idev->keybit);
	set_bit(BTN_TOOL_FINGER, idev->keybit);
	set_bit(BTN_TOOL_DOUBLETAP, idev->keybit);
	set_bit(BTN_TOOL_TRIPLETAP, idev->keybit);
	set_bit(BTN_LEFT, idev->keybit);
	set_bit(BTN_RIGHT, idev->keybit);
	if (synusb->is_cpad) {
		set_bit(BTN_MIDDLE, idev->keybit);
		set_bit(BTN_MISC, idev->keybit);
	}

	usb_make_path(udev, synusb->iphys, sizeof(synusb->iphys));
	strlcat(synusb->iphys, "/input0", sizeof(synusb->iphys));
	idev->phys = synusb->iphys;
	idev->name = synusb_input_name;
	usb_to_input_id(udev, &idev->id);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,22)
	idev->dev.parent = &synusb->interface->dev;
	input_set_drvdata(idev, synusb);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15)
	idev->cdev.dev = &synusb->interface->dev;
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,2)
	idev->dev = &synusb->interface->dev;
#endif

	retval = input_register_device(idev);
	if (retval) {
		err("Can not register input device");
		goto error;
	}

	INIT_DELAYED_WORK(&synusb->isubmit, synusb_submit_int);
	schedule_delayed_work(&synusb->isubmit, HZ/100);

	synusb->idev = idev;
	return 0;

error:
	if (idev)
		input_free_device(idev);
	return retval;
}


#ifdef CONFIG_USB_CPADDEV

/*
 * cPad character device
 */

/****
 * the cPad is an USB touchpad with background display (240x160 mono)
 * it has one interface with three possible alternate settings
 *	setting 0: one int endpoint for rel movement (used by usbhid.ko)
 *	setting 1: one int endpoint for abs finger position
 *	setting 2: one int endpoint for abs finger position and
 *		   two bulk endpoints for the display (in/out)
 * The Synaptics Touchpads without display only have settings 0 and 1, so
 * this driver uses setting 2 for the cPad and setting 1 for other Touchpads.
 *
 * How the bulk endpoints work:
 *
 * the cPad display is controlled by a Seiko/Epson 1335 LCD Controller IC
 * in order to send commands to the sed1335, each packet in the urb must look
 * like this:
 *	02 <1335 command> [<data in reversed order> ...]
 * the possible commands for the sed1335 are listed in cpad.h. the data must
 * be in reversed order as stated in the sed1335 data sheet.
 * the urb must be send to the bulk out endpoint. because the packet size of
 * this endoint is 32 bytes, "02 <1335 command>" must be repeated after 30
 * bytes of data. the data must be in reversed order in each of this 30 bytes
 * block. all this is done automatically when writing to the character device.
 *
 * functions that are not controlled by the sed1335, like the backlight, can
 * be accessed by
 *	01 <function> <state>
 * the packet must be send to the bulk out endpoint. these functions can be
 * accessed via ioctls.
 *
 * observed functions are: */
#define CPAD_W_ROM	0x01	/* write EEPROM (not supported) */
#define CPAD_R_ROM	0x02	/* read EEPROM */
#define CPAD_W_LIGHT	0x03	/* write backlight state (on/off) */
#define CPAD_R_LIGHT	0x04	/* read backlight state (on/off) */
#define CPAD_W_LCD	0x05	/* write lcd state (on/off) */
#define CPAD_R_LCD	0x06	/* read lcd state (on/off) */
#define CPAD_RSRVD	0x07

/* possible values for the first byte of a packet */
#define SEL_CPAD	0x01	/* cPad not-lcd-controller select */
#define SEL_1335	0x02	/* lcd controller select */

/* an urb to the bulk out endpoint should be followed by an urb to the bulk
 * in endpoint. this gives the answer of the cpad/sed1335. */

#define USB_CPAD_MINOR_BASE	192
#define CPAD_DRIVER_NUM 	8

static int cpad_open(struct inode *inode, struct file *file)
{
	struct synusb *synusb;
	struct usb_interface *interface;
	int subminor, retval;

	subminor = iminor(inode);

	interface = usb_find_interface(&synusb_driver, subminor);
	if (!interface) {
		err ("Can not find device for minor %d", subminor);
		return -ENODEV;
	}

	synusb = usb_get_intfdata(interface);
	if (!synusb)
		return -ENODEV;

	retval = usb_autopm_get_interface(interface);
	if (retval)
		return retval;

	kref_get(&synusb->kref);
	file->private_data = synusb;

	return 0;
}

static int cpad_release(struct inode *inode, struct file *file)
{
	struct synusb *synusb;

	synusb = (struct synusb *)file->private_data;
	if (synusb == NULL)
		return -ENODEV;

	mutex_lock(&synusb->autoput_mutex);
	if (synusb->interface)
		usb_autopm_put_interface(synusb->interface);
	mutex_unlock(&synusb->autoput_mutex);

	kref_put(&synusb->kref, synusb_delete);
	return 0;
}

static inline int cpad_lock(struct synusb* synusb, struct file *file)
{
	int retval = 0;
	if (!mutex_trylock(&synusb->io_mutex)) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		if (mutex_lock_interruptible(&synusb->io_mutex))
			return -ERESTARTSYS;
	}
	if (!synusb->interface) {
		retval = -ENODEV;
		goto error;
	}
	if (synusb->error) {
		retval = synusb->error;
		synusb->error = 0;
		goto error;
	}
	return 0;
error:
	mutex_unlock(&synusb->io_mutex);
	return retval;
}

static ssize_t cpad_read(struct file *file, char *buffer,
			 size_t count, loff_t *ppos)
{
	struct synusb *synusb;
	int retval;

	synusb = (struct synusb *)file->private_data;

	retval = cpad_lock(synusb, file);
	if (retval)
		return retval;

	if (!synusb->has_indata) {
		retval = -ENODATA;
		goto error;
	}

	count = min(count, (size_t)synusb->in->actual_length);
	if (copy_to_user(buffer, synusb->in->transfer_buffer, count))
		retval = -EFAULT;
	else
		retval = count;
error:
	mutex_unlock(&synusb->io_mutex);
	return retval;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
static void cpad_in_callback(struct urb *urb)
#else
static void cpad_in_callback(struct urb *urb, struct pt_regs *regs)
#endif
{
	struct synusb *synusb;

	synusb = (struct synusb *)urb->context;

	complete(&synusb->cpad_done);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
static void cpad_out_callback(struct urb *urb)
#else
static void cpad_out_callback(struct urb *urb, struct pt_regs *regs)
#endif
{
	struct synusb *synusb;

	synusb = (struct synusb *)urb->context;

	if (urb->status)
		goto error;

	synusb->error = usb_submit_urb(synusb->in, GFP_ATOMIC);
	if (!synusb->error)
		return;
	err("usb_submit_urb bulk in failed, error %d", synusb->error);
	if (synusb->error == -EPIPE)
		synusb->error = -EIO;
error:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
	cpad_in_callback(urb);
#else
	cpad_in_callback(urb, regs);
#endif
}

/* send out and in urbs synchronously */
static int cpad_submit_bulk(struct synusb* synusb)
{
	int retval;

	usb_anchor_urb(synusb->out, &synusb->submitted);
	usb_anchor_urb(synusb->in, &synusb->submitted);
	retval = usb_submit_urb(synusb->out, GFP_KERNEL);
	if (retval) {
		usb_unanchor_urb(synusb->out);
		usb_unanchor_urb(synusb->in);
		err("usb_submit_urb bulk out failed, error %d", retval);
		goto error;
	}

	retval = wait_for_completion_interruptible_timeout(&synusb->cpad_done,
						  2*HZ);
	if (retval <= 0) {
		usb_kill_anchored_urbs(&synusb->submitted);
		retval = retval ? retval : -ETIMEDOUT;
		goto error;
	}

	if (synusb->out->status) {
		retval = synusb->out->status;
	} else if (synusb->error) {
		retval = synusb->error;
		usb_unanchor_urb(synusb->in);
	} else
		retval = synusb->in->status;
error:
	synusb->error = 0;
	synusb->has_indata = retval < 0 ? 0 : 1;
	return retval;
}

static inline u8 *cpad_1335_fillpacket
		(u8 cmd, u8 *param,
		 size_t param_size, u8 *out_buf)
{
	u8 *point;

	/* select 1335, set 1335 command, reverse params */
	*(out_buf++) = SEL_1335;
	*(out_buf++) = cmd;
	for (point=param+param_size-1; point>=param; point--)
		*(out_buf++) = *point;
	return out_buf;
}

static int cpad_write_fillbuffer(struct synusb *synusb,
				 const u8 *ubuffer, size_t count)
{
	u8 *out_buf = synusb->out->transfer_buffer;
	const u8 *ubuffer_orig = ubuffer;
	u8 param[30];
	size_t param_size, actual_length;
	u8 cmd;

	/* get 1335 command first */
	if (get_user(cmd, ubuffer))
		return -EFAULT;
	ubuffer++;
	if (cmd == SLEEP_1335)
		warn("sleeping sed1335 might damage the display");

	if (count == 1) {
		/* 1335 command without params */
		*(out_buf++) = SEL_1335;
		*(out_buf++) = cmd;
		synusb->out->transfer_buffer_length = 2;
		return 1;
	}

	actual_length = 0;
	count--;
	while (count > 0) {
		param_size = min(count, (size_t)30);
		if (actual_length+param_size+2 > 274*32)
			break;

		if (copy_from_user(param, ubuffer, param_size))
			return -EFAULT;

		ubuffer += param_size;
		count -= param_size;

		out_buf = cpad_1335_fillpacket(cmd, param,
					       param_size, out_buf);
		actual_length += param_size + 2;
	}

	synusb->out->transfer_buffer_length = actual_length;
	return ubuffer-ubuffer_orig;
}

static ssize_t cpad_write(struct file *file, const char *user_buffer,
			  size_t count, loff_t *ppos)
{
	struct synusb *synusb;
	int length, retval;

	synusb = (struct synusb *)file->private_data;

	if (count == 0)
		return 0;

	retval = cpad_lock(synusb, file);
	if (retval)
		return retval;

	length = cpad_write_fillbuffer(synusb, (u8*) user_buffer,
				       count);
	if (length < 0) {
		retval = length;
		goto error;
	}

	retval = cpad_submit_bulk(synusb);
	if (!retval)
		retval = length;
error:
	mutex_unlock(&synusb->io_mutex);
	return retval;
}

static int cpad_nlcd_function(struct synusb *synusb, u8 func, u8 val)
{
	u8 *out_buf = synusb->out->transfer_buffer;
	int retval;

	if ((func < (u8) 2) || (func > (u8) 6)) {
		err("1335 nlcd invalid cmd");
		return -EINVAL;
	}

	*(out_buf++) = SEL_CPAD;
	*(out_buf++) = func;
	*(out_buf++) = val;
	synusb->out->transfer_buffer_length = 3;
	((u8 *)synusb->in->transfer_buffer)[2] = 0;

	retval = cpad_submit_bulk(synusb);
	if (!retval)
		retval = ((u8 *)synusb->in->transfer_buffer)[2];

	return retval;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void cpad_light_off(void *work)
#else
static void cpad_light_off(struct work_struct *work)
#endif
{
	struct synusb *synusb = container_of(work, struct synusb, flash.work);

	mutex_lock(&synusb->io_mutex);
	if (!synusb->interface)
		return;
	if (cpad_nlcd_function(synusb, CPAD_W_LIGHT, 0) < 0)
		err("error on timed_light_off");
	mutex_unlock(&synusb->io_mutex);
}

static int cpad_flash(struct synusb *synusb, int time)
{
	struct delayed_work *flash = &synusb->flash;
	int res;

	if (time <= 0)
		return -EINVAL;
	cancel_delayed_work(&synusb->flash);
	res = cpad_nlcd_function(synusb, CPAD_W_LIGHT, 1);
	if (res < 0)
		return res;
	time = min(time, 1000);
	schedule_delayed_work(flash, (HZ * (unsigned long) time) / 100);
	return 0;
}

int cpad_driver_num = CPAD_DRIVER_NUM;

static int cpad_ioctl(struct inode *inode, struct file  *file,
		      unsigned int  cmd, unsigned long arg)
{
	struct synusb *synusb;
	u8 cval = 0;
	int ival = 0;
	void *rval = NULL;
	int res = 0;

	synusb = (struct synusb *)file->private_data;

	res = cpad_lock(synusb, file);
	if (res)
		return res;

	/* read data from user */
	if (cmd & IOC_IN) {
		res = -EFAULT;
		switch (_IOC_SIZE(cmd)) {
		case sizeof(u8):
			if (get_user(cval, (u8 *) arg))
				goto error;
			break;
		case sizeof(int):
			if (get_user(ival, (int *) arg))
				goto error;
			break;
		default:
			res = -ENOIOCTLCMD;
			goto error;
		}
		res = 0;
	}

	switch (cmd) {
	case CPAD_VERSION:
		rval = &cpad_driver_num;
		break;

	case CPAD_CGID:
		rval = &synusb->idev->id;
		break;

	case CPAD_WLIGHT:
		res = cpad_nlcd_function(synusb, CPAD_W_LIGHT, cval);
		break;

	case CPAD_FLASH:
		res = cpad_flash(synusb, ival);
		break;

	case CPAD_WLCD:
		res = cpad_nlcd_function(synusb, CPAD_W_LCD, cval);
		break;

	case CPAD_RLIGHT:
		res = cpad_nlcd_function(synusb, CPAD_R_LIGHT, 0);
		break;

	case CPAD_RLCD:
		res = cpad_nlcd_function(synusb, CPAD_R_LCD, 0);
		break;

	case CPAD_RESET:
		err("CPAD_RESET deprecated. Use libusb to reset cPad.");
		res = -ENOIOCTLCMD;
		// usb_reset_device(synusb->udev);
		break;

	case CPAD_REEPROM:
		res = cpad_nlcd_function(synusb, CPAD_R_ROM, 0);
		break;

	default:
		res = -ENOIOCTLCMD;
	}
error:
	mutex_unlock(&synusb->io_mutex);
	if (res < 0)
		return res;

	/* write data to user */
	if ((cmd & IOC_OUT) && (rval != NULL))
		if (copy_to_user((void *) arg, rval, _IOC_SIZE(cmd)))
			return -EFAULT;

	return 0;
}

static struct file_operations cpad_fops = {
	.owner =	THIS_MODULE,
	.read =		cpad_read,
	.write =	cpad_write,
	.ioctl =	cpad_ioctl,
	.open =		cpad_open,
	.release =	cpad_release,
};

static struct usb_class_driver cpad_class = {
	.name =		"usb/cpad%d",
	.fops =		&cpad_fops,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)
	.mode =		S_IFCHR | S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP,
#endif
	.minor_base =	USB_CPAD_MINOR_BASE,
};

static int cpad_init_bulk(struct synusb *synusb, struct urb **urb,
			  size_t size, usb_complete_t bulk_callback, int pipe)
{
	char *buf;

	*urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!*urb)
		return -ENOMEM;
	buf = usb_buffer_alloc(synusb->udev, size, GFP_KERNEL,
			       &(*urb)->transfer_dma);
	if (!buf)
		return -ENOMEM;
	usb_fill_bulk_urb(*urb, synusb->udev, pipe, buf, size,
			  bulk_callback, synusb);
	(*urb)->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	return 0;
}

static int cpad_setup_in(struct synusb *synusb,
			 struct usb_endpoint_descriptor *endpoint)
{
	if ((synusb->in) || (endpoint->wMaxPacketSize != 32))
		return -ENODEV;

	return cpad_init_bulk(synusb, &synusb->in, 32, cpad_in_callback,
			      usb_rcvbulkpipe(synusb->udev,
					      endpoint->bEndpointAddress));
}

static int cpad_setup_out(struct synusb *synusb,
			  struct usb_endpoint_descriptor *endpoint)
{
	if ((synusb->out) || (endpoint->wMaxPacketSize != 32))
		return -ENODEV;

	return cpad_init_bulk(synusb, &synusb->out, 274*32, cpad_out_callback,
			      usb_sndbulkpipe(synusb->udev,
					      endpoint->bEndpointAddress));
}

#endif /* CONFIG_USB_CPADDEV */


/*
 * init etc.
 */

static int synusb_setup_iurb(struct synusb *synusb,
			     struct usb_endpoint_descriptor *endpoint)
{
	char *buf;

	if (synusb->iurb)
		return -ENODEV;
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

static struct synusb_endpoint_table {
	__u8 dir;
	__u8 xfer_type;
	int  (*setup)(struct synusb *,
		      struct usb_endpoint_descriptor *);
} synusb_endpoints [] = {
#ifdef CONFIG_USB_CPADDEV
	{ USB_DIR_IN,  USB_ENDPOINT_XFER_BULK, cpad_setup_in },
	{ USB_DIR_OUT, USB_ENDPOINT_XFER_BULK, cpad_setup_out },
#endif /* CONFIG_USB_CPADDEV */
	{ USB_DIR_IN,  USB_ENDPOINT_XFER_INT, synusb_setup_iurb },
	{ }
};

static inline int synusb_match_endpoint(struct usb_endpoint_descriptor *ep)
{
	int i;

	for (i=0; synusb_endpoints[i].setup; i++)
		if (( (ep->bEndpointAddress & USB_DIR_IN)
				== synusb_endpoints[i].dir		) &&
		    ( (ep->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
				== synusb_endpoints[i].xfer_type	))
			return i;
	return -1;
}

static int synusb_setup_endpoints(struct synusb *synusb)
{
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	int i, j, res;
	int num = 0;

	iface_desc = synusb->interface->cur_altsetting;
	for (i = 0; i < iface_desc->desc.bNumEndpoints; i++) {
		endpoint = &iface_desc->endpoint[i].desc;

		j = synusb_match_endpoint(endpoint);
		if (j >= 0) {
			res = synusb_endpoints[j].setup(synusb, endpoint);
			if (res)
				return res;
			num++;
		}
	}

#ifdef CONFIG_USB_CPADDEV
	if (synusb->is_cpad) {
		if (num != 3)
			return -ENODEV;
		else
			return 0;
	}
#endif /* CONFIG_USB_CPADDEV */

	if (num != 1)
		return -ENODEV;

	return 0;
}

static int synusb_probe(struct usb_interface *interface,
			const struct usb_device_id *id)
{
	struct synusb *synusb = NULL;
	struct usb_device *udev = interface_to_usbdev(interface);
	int ifnum, retval = -ENOMEM;
	int is_cpad;

	is_cpad = (id->idProduct == USB_DID_SYN_CPAD) ? 1 : 0;

	ifnum = interface->cur_altsetting->desc.bInterfaceNumber;
	if ((id->idProduct == USB_DID_SYN_COMP_TP) && (ifnum != 0))
		return -ENODEV;
	if (usb_set_interface (udev, ifnum, is_cpad ? 2 : 1))
		return -ENODEV;

	synusb = kzalloc(sizeof(*synusb), GFP_KERNEL);
	if (synusb == NULL) {
		err("Out of memory");
		goto error;
	}
	synusb->is_cpad = is_cpad;
	kref_init(&synusb->kref);

	synusb->udev = usb_get_dev(udev);
	synusb->interface = interface;
	retval = synusb_setup_endpoints(synusb);
	if (retval) {
		err("Could not set up endpoints, error: %i", retval);
		goto error;
	}
	usb_set_intfdata(interface, synusb);
	interface->needs_remote_wakeup = 1;

	retval = synusb_init_input(synusb);
	if (retval) {
		goto error;
	}

#ifdef CONFIG_USB_CPADDEV
	if (!is_cpad)
		return 0;

	retval = usb_register_dev(interface, &cpad_class);
	if (retval) {
		err("Not able to get a minor for this device.");
		usb_set_intfdata(interface, NULL);
		goto error;
	}
	printk(KERN_INFO "cpad registered on minor: %d\n", interface->minor);

	init_completion(&synusb->cpad_done);
	INIT_DELAYED_WORK(&synusb->flash, cpad_light_off);
	init_usb_anchor(&synusb->submitted);
	mutex_init(&synusb->io_mutex);
	mutex_init(&synusb->autoput_mutex);
#endif /* CONFIG_USB_CPADDEV */
	return 0;

error:
	if (synusb)
		kref_put(&synusb->kref, synusb_delete);
	return retval;
}

static void synusb_disconnect(struct usb_interface *interface)
{
	struct synusb *synusb;

	synusb = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);

#ifdef CONFIG_USB_CPADDEV
	if (synusb->is_cpad) {
		usb_deregister_dev(interface, &cpad_class);

		/* prevent more I/O from starting */
		mutex_lock(&synusb->io_mutex);
		mutex_lock(&synusb->autoput_mutex);
		synusb->interface = NULL;
		mutex_unlock(&synusb->autoput_mutex);
		mutex_unlock(&synusb->io_mutex);

		cancel_delayed_work(&synusb->flash);
	}
#endif /* CONFIG_USB_CPADDEV */

	cancel_delayed_work(&synusb->isubmit);
	flush_scheduled_work();

	usb_kill_urb(synusb->iurb);

	kref_put(&synusb->kref, synusb_delete);

	printk(KERN_INFO "Synaptics touchpad disconnected\n");
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,23)

#ifdef CONFIG_USB_CPADDEV
static void cpad_draw_down(struct synusb *synusb)
{
	int time;

	if (!synusb->is_cpad)
		return;
	time = usb_wait_anchor_empty_timeout(&synusb->submitted, 1000);
	if (!time)
		usb_kill_anchored_urbs(&synusb->submitted);
}
#endif /* CONFIG_USB_CPADDEV */

static void synusb_draw_down(struct synusb *synusb)
{
	cancel_delayed_work_sync(&synusb->isubmit);
	usb_kill_urb(synusb->iurb);
}

static int synusb_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct synusb *synusb = usb_get_intfdata(intf);
#ifdef CONFIG_USB_CPADDEV
	int turn_light_off = 0;
#endif /* CONFIG_USB_CPADDEV */

	if (!synusb)
		return 0;

#ifdef CONFIG_USB_CPADDEV
	if (synusb->is_cpad) {
		turn_light_off = delayed_work_pending(&synusb->flash);
		cancel_delayed_work_sync(&synusb->flash);
	}
	if (turn_light_off) {
		if (cpad_nlcd_function(synusb, CPAD_W_LIGHT, 0) < 0)
			err("unable turn backlight off before suspend");
		cpad_draw_down(synusb);
	}
#endif /* CONFIG_USB_CPADDEV */

	synusb_draw_down(synusb);

	return 0;
}

static int synusb_resume(struct usb_interface *intf)
{
	struct synusb *synusb = usb_get_intfdata(intf);
	int res;

	res = usb_submit_urb(synusb->iurb, GFP_ATOMIC);
	if (res)
		err("usb_submit_urb int in failed during resume with result %d", res);

	return res;
}

static int synusb_reset_resume(struct usb_interface *intf)
{
	struct synusb *synusb = usb_get_intfdata(intf);
	int res;

#ifdef CONFIG_USB_CPADDEV
	if (synusb->is_cpad) {
		synusb->error = -EPIPE;
		synusb->has_indata = 0;
	}
#endif /* CONFIG_USB_CPADDEV */

	res = usb_submit_urb(synusb->iurb, GFP_ATOMIC);
	if (res)
		err("usb_submit_urb int in failed during resume with result %d", res);

	return res;
}

static int synusb_pre_reset(struct usb_interface *intf)
{
	struct synusb *synusb = usb_get_intfdata(intf);

#ifdef CONFIG_USB_CPADDEV
	if (synusb->is_cpad) {
		mutex_lock(&synusb->io_mutex);
		cancel_delayed_work(&synusb->flash);
	}
#endif /* CONFIG_USB_CPADDEV */
	synusb_draw_down(synusb);

	return 0;
}

static int synusb_post_reset(struct usb_interface *intf)
{
	struct synusb *synusb = usb_get_intfdata(intf);
	int res;

#ifdef CONFIG_USB_CPADDEV
	if (synusb->is_cpad) {
		synusb->error = -EPIPE;
		synusb->has_indata = 0;
		mutex_unlock(&synusb->io_mutex);
	}
#endif /* CONFIG_USB_CPADDEV */

	res = usb_submit_urb(synusb->iurb, GFP_KERNEL);
	if (res)
		err("usb_submit_urb int in failed during post_reset with result %d", res);

	return res;
}

#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,23) */

static struct usb_driver synusb_driver = {
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
		printk(KERN_INFO DRIVER_DESC " " DRIVER_VERSION "\n");

	return result;
}

static void __exit synusb_exit(void)
{
	usb_deregister(&synusb_driver);
}

module_init(synusb_init);
module_exit(synusb_exit);

MODULE_AUTHOR (DRIVER_AUTHOR);
MODULE_DESCRIPTION (DRIVER_DESC);
MODULE_LICENSE("GPL");
