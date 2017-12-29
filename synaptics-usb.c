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
 * cpad_light_off and suspend: actual_length & inbuffer backup
 * has_indata in write and ioctl
 * check coding style and comments
 * repalce enum
 * module param for exclusive open
 * add error and debug messages
 * review suspend code
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


#define DRIVER_VERSION	"v1.5"
#define DRIVER_AUTHOR	"Rob Miller (rob@inpharmatica . co . uk), "\
			"Ron Lee (ron@debian.org), "\
			"Jan Steinhoff <jan.steinhoff@uni-jena.de>"
#define DRIVER_DESC	"USB Synaptics device driver"

/* vendor and device IDs */
#define USB_VID_SYNAPTICS	0x06cb	/* Synaptics vendor ID */
#define USB_DID_SYN_TP		0x0001	/* Synaptics USB TouchPad */
#define USB_DID_SYN_INT_TP	0x0002	/* Synaptics Integrated USB TouchPad */
#define USB_DID_SYN_CPAD	0x0003	/* Synaptics cPad */
#define USB_DID_SYN_TS		0x0006	/* Synaptics TouchScreen */
#define USB_DID_SYN_STICK	0x0007	/* Synaptics USB Styk */
#define USB_DID_SYN_WP		0x0008	/* Synaptics USB WheelPad */
#define USB_DID_SYN_COMP_TP	0x0009	/* Synaptics Composite USB TouchPad */
#define USB_DID_SYN_WTP		0x0010	/* Synaptics Wireless TouchPad */
#define USB_DID_SYN_DP		0x0013	/* Synaptics DisplayPad */

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


struct synusb {
	u32 check;

	struct usb_device *	udev;
	struct usb_interface *	interface;
	struct kref		kref;

	/* input device */
	enum input_type 	{pad, stick, screen}
				input_type;
	int			buttons;
	struct input_dev *	idev;
	char			iphys[64];
	struct delayed_work	isubmit;
	struct urb *		iurb;

	/* display character device (cPad device) */
#ifdef CONFIG_USB_CPADDEV
	struct syndisplay {
		struct synusb *		parent;
		struct mutex		io_mutex;
		struct mutex		open_mutex;
		int			open_count;
		struct usb_anchor	submitted;
		struct urb		*in, *out;
		struct completion	cpad_done;
		int			has_indata;
		int			error;
		struct delayed_work	flash;
	} *			display;
#else /* CONFIG_USB_CPADDEV */
	int			display;
#endif /* CONFIG_USB_CPADDEV */
};

#define SYNUSB_CHECK 0x12345678
#define SYNUSB_CHECK_INIT 0x87654321
#define SYNUSB_CHECK_DELETED 0x56781234
int inline synusb_check(struct synusb *synusb)
{
	if (unlikely(synusb == NULL)) {
		err("synaptics-usb data is NULL pointer!");
		goto error;
	}

	if (unlikely(synusb->check != SYNUSB_CHECK)) {
		err("synaptics-usb data corrupted! Code: %x", synusb->check);
		goto error;
	}

#ifdef CONFIG_USB_CPADDEV
	if (synusb->display) {
		if (unlikely(synusb->display->parent != synusb)) {
			err("synaptics-usb display data corrupted!");
			goto error;
		}
	} else {
		err("synaptics-usb display data is NULL pointer!");
		goto error;
	}
#endif

	return 0;
error:
	BUG();
	return -EFAULT;
}
#define SYNCHECK(_synusb)			\
	do{					\
		if (synusb_check(_synusb))	\
			return -EFAULT;		\
	} while (0)
#define SYNCHECK_NR(_synusb)			\
	do{					\
		if (synusb_check(_synusb))	\
			return;		\
	} while (0)

struct completion test_completion;
static int test;

static int synusb_set_test(const char *val, struct kernel_param *kp)
{
	complete(&test_completion);
	return param_set_int(val, kp);
}

module_param_call(test, synusb_set_test, param_get_int, &test, 0664);


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

	dbg("synusb_delete() called.");

	if (synusb->check != SYNUSB_CHECK_INIT)
		SYNCHECK_NR(synusb);
	synusb->check = SYNUSB_CHECK_DELETED;

	synusb_free_urb(synusb->iurb);
	if (synusb->idev)
		input_unregister_device(synusb->idev);

#ifdef CONFIG_USB_CPADDEV
	if (synusb->display) {
		if (synusb->display->out)
			synusb->display->out->transfer_buffer_length = 274*32;
		synusb_free_urb(synusb->display->in);
		synusb_free_urb(synusb->display->out);
		kfree(synusb->display);
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

static const char synusb_pad_name[] = "Synaptics USB TouchPad";
static const char synusb_stick_name[] = "Synaptics USB Styk";
static const char synusb_screen_name[] = "Synaptics USB TouchScreen";

static const char* synusb_get_name(struct synusb *synusb)
{
	switch (synusb->input_type) {
	case pad:
		return synusb_pad_name;
	case stick:
		return synusb_stick_name;
	case screen:
		return synusb_screen_name;
	}
	return NULL;
}

static inline void synusb_report_width(struct input_dev *idev,
					 int pressure, int w)
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
	int is_stick = (synusb->input_type == stick) ? 1 : 0;

	SYNCHECK_NR(synusb);

	switch (urb->status) {
	case 0:
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		dbg("Stopping int urb.");
		return;
	default:
		goto resubmit;
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
		input_report_abs (idev, ABS_X, x);
		input_report_abs (idev, ABS_Y, y);
	}
	input_report_abs(idev, ABS_PRESSURE, pressure);

	input_report_key(idev, BTN_LEFT, data[1] & 0x04);
	input_report_key(idev, BTN_RIGHT, data[1] & 0x01);
	input_report_key(idev, BTN_MIDDLE, data[1] & 0x02);
	if (synusb->display)
		input_report_key(idev, btn_middle ? BTN_MIDDLE : BTN_MISC,
				 data[1] & 0x08);

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

	dbg("synusb_submit_int() called.");
	SYNCHECK_NR(synusb);

	res = usb_submit_urb(synusb->iurb, GFP_KERNEL);
	if (res)
		err ("usb_submit_urb int in failed with result %d", res);
}

static int synusb_init_input(struct synusb *synusb)
{
	struct input_dev *idev;
	struct usb_device *udev = synusb->udev;
	int is_stick = (synusb->input_type == stick) ? 1 : 0;
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
	if (synusb->display)
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


#ifdef CONFIG_USB_CPADDEV

/*
 * display character device for cPad
 */

/****
 * the cPad is an USB touchpad with background display (240x160 mono)
 * it has one interface with three possible alternate settings
 *	setting 0: one int endpoint for relative movement (used by usbhid.ko)
 *	setting 1: one int endpoint for absolute finger position
 *	setting 2: one int endpoint for absolute finger position and
 *		   two bulk endpoints for the display (in/out)
 * The Synaptics touchpads without display only have settings 0 and 1, so
 * this driver uses setting 2 for the cPad and setting 1 for other touchpads.
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
	int subminor = iminor(inode);
	int retval = 0;

	dbg("cpad_open() called.");

	interface = usb_find_interface(&synusb_driver, subminor);
	if (!interface) {
		err("Can not find device for minor %d.", subminor);
		return -ENODEV;
	}

	synusb = usb_get_intfdata(interface);
	if (!synusb) {
		dbg("Device is gone.");
		return -ENODEV;
	}
	SYNCHECK(synusb);

	kref_get(&synusb->kref);

	mutex_lock(&synusb->display->open_mutex);

	if (!synusb->display->open_count++) {
		retval = usb_autopm_get_interface(interface);
			if (retval) {
				dbg("Can not resume.");
				synusb->display->open_count--;
				mutex_unlock(&synusb->display->open_mutex);
				kref_put(&synusb->kref, synusb_delete);
				goto exit;
			}
	} /* else {
		dbg("Device already open.");
		retval = -EBUSY;
		synusb->display->open_count--;
		mutex_unlock(&synusb->display->open_mutex);
		kref_put(&synusb->kref, synusb_delete);
		goto exit;
	} */

	file->private_data = synusb;
	mutex_unlock(&synusb->display->open_mutex);
	dbg("cpad_open() successful.");
exit:
	return retval;
}

static int cpad_release(struct inode *inode, struct file *file)
{
	struct synusb *synusb;

	dbg("cpad_release() called.");

	synusb = (struct synusb *)file->private_data;
	if (synusb == NULL) {
		dbg("private_data is NULL.");
		return -ENODEV;
	}
	SYNCHECK(synusb);

	mutex_lock(&synusb->display->open_mutex);
	if (!--synusb->display->open_count && synusb->interface)
		usb_autopm_put_interface(synusb->interface);
	mutex_unlock(&synusb->display->open_mutex);

	kref_put(&synusb->kref, synusb_delete);
	dbg("cpad_release() successful.");
	return 0;
}

static int cpad_lock(struct synusb* synusb, struct file *file)
{
	int retval = 0;

	SYNCHECK(synusb);

	if (!mutex_trylock(&synusb->display->io_mutex)) {
		dbg("Device is locked.");
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		if (mutex_lock_interruptible(&synusb->display->io_mutex))
			return -ERESTARTSYS;
	}
	if (!synusb->interface) {
		dbg("Device is gone.");
		retval = -ENODEV;
		goto error;
	}
	if (synusb->display->error) {
		retval = synusb->display->error;
		dbg("Reporting error %i.", retval);
		synusb->display->error = 0;
		goto error;
	}
	return 0;
error:
	mutex_unlock(&synusb->display->io_mutex);
	return retval;
}

static ssize_t cpad_read(struct file *file, char *buffer,
			 size_t count, loff_t *ppos)
{
	struct synusb *synusb = (struct synusb *)file->private_data;
	int retval;

	dbg("cpad_read() called.");

	retval = cpad_lock(synusb, file);
	if (retval)
		return retval;

	if (!synusb->display->has_indata) {
		dbg("No data available.");
		retval = -ENODATA;
		goto error;
	}

	count = min(count, (size_t)synusb->display->in->actual_length);
	if (copy_to_user(buffer, synusb->display->in->transfer_buffer, count))
		retval = -EFAULT;
	else
		retval = count;
error:
	dbg("cpad_read() is returning %i.", retval);
	mutex_unlock(&synusb->display->io_mutex);
	return retval;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
static void cpad_in_callback(struct urb *urb)
#else
static void cpad_in_callback(struct urb *urb, struct pt_regs *regs)
#endif
{
	struct synusb *synusb = (struct synusb *)urb->context;

	dbg("cpad_in_callback() called.");
	SYNCHECK_NR(synusb);

	if (urb->status) {
		if(!(urb->status == -ENOENT ||
		    urb->status == -ECONNRESET ||
		    urb->status == -ESHUTDOWN))
			err("Bulk in urb returned status %i.", urb->status);
		else
			dbg("Bulk in urb returned status %i.", urb->status);
	}

	dbg("Completing IO.");
	complete(&synusb->display->cpad_done);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
static void cpad_out_callback(struct urb *urb)
#else
static void cpad_out_callback(struct urb *urb, struct pt_regs *regs)
#endif
{
	struct synusb *synusb = (struct synusb *)urb->context;

	dbg("cpad_out_callback() called.");
	SYNCHECK_NR(synusb);

	if (urb->status) {
		if(!(urb->status == -ENOENT ||
		    urb->status == -ECONNRESET ||
		    urb->status == -ESHUTDOWN))
			err("Bulk out urb returned status %i.", urb->status);
		else
			dbg("Bulk out urb returned status %i.", urb->status);
		goto error;
	}

	usb_anchor_urb(synusb->display->in, &synusb->display->submitted);
	synusb->display->error = usb_submit_urb(synusb->display->in, GFP_ATOMIC);
	if (!synusb->display->error) {
		dbg("Submitted bulk in urb.");
		return;
	}
	err("usb_submit_urb bulk in failed, error %d", synusb->display->error);
	usb_unanchor_urb(synusb->display->in);
	if (synusb->display->error == -EPIPE)
		synusb->display->error = -EIO;
error:
	dbg("Unachoring in urb and completing IO.");
	complete(&synusb->display->cpad_done);
}

/* send out and in urbs synchronously */
static int cpad_submit_bulk(struct syndisplay* display)
{
	int retval;

	dbg("cpad_submit_bulk() called.");

	if (unlikely(display == NULL)) {
		err("display data is NULL pointer!");
		BUG();
		return -EFAULT;
	}
	SYNCHECK(display->parent);

	INIT_COMPLETION(display->cpad_done);
	usb_anchor_urb(display->out, &display->submitted);
	retval = usb_submit_urb(display->out, GFP_KERNEL);
	if (retval) {
		err("usb_submit_urb bulk out failed, error %d", retval);
		usb_unanchor_urb(display->out);
		goto error;
	}
	dbg("Submitted bulk out urb.");

	retval = wait_for_completion_interruptible_timeout
				(&display->cpad_done, HZ*2);
	dbg("wait_for_completion_interruptible_timeout() returned %i.", retval);
	if (retval <= 0) {
		retval = retval ? retval : -ETIMEDOUT;
		usb_kill_anchored_urbs(&display->submitted);
		goto error;
	}

	if (display->out->status) {
		retval = display->out->status;
	} else if (display->error) {
		retval = display->error;
	} else
		retval = display->in->status;
error:
	display->error = 0;
	display->has_indata = retval < 0 ? 0 : 1;
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
	u8 *out_buf = synusb->display->out->transfer_buffer;
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
		synusb->display->out->transfer_buffer_length = 2;
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

	synusb->display->out->transfer_buffer_length = actual_length;
	return ubuffer-ubuffer_orig;
}

static ssize_t cpad_write(struct file *file, const char *user_buffer,
			  size_t count, loff_t *ppos)
{
	struct synusb *synusb;
	int length, retval;

	dbg("cpad_write() called.");

	synusb = (struct synusb *)file->private_data;

	if (count == 0)
		return 0;

	retval = cpad_lock(synusb, file);
	if (retval)
		return retval;

	length = cpad_write_fillbuffer(synusb, (u8*) user_buffer, count);
	if (length < 0) {
		retval = length;
		goto error;
	}

	retval = cpad_submit_bulk(synusb->display);
	if (!retval)
		retval = length;
error:
	dbg("cpad_write() is returning %i.", retval);
	mutex_unlock(&synusb->display->io_mutex);
	return retval;
}

static int cpad_nlcd_function(struct syndisplay* display, u8 func, u8 val)
{
	u8 *out_buf = display->out->transfer_buffer;
	int retval;

	dbg("cpad_nlcd_function() called, function %i.", func);

	if ((func < (u8) 2) || (func > (u8) 6)) {
		err("Invalid nlcd command.");
		return -EINVAL;
	}

	*(out_buf++) = SEL_CPAD;
	*(out_buf++) = func;
	*(out_buf++) = val;
	display->out->transfer_buffer_length = 3;
	((u8 *)display->in->transfer_buffer)[2] = 0;

	retval = cpad_submit_bulk(display);
	if (!retval)
		retval = ((u8 *)display->in->transfer_buffer)[2];

	return retval;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void cpad_light_off(void *work)
#else
static void cpad_light_off(struct work_struct *work)
#endif
{
	struct syndisplay *display = container_of(work, struct syndisplay, flash.work);
	struct synusb *synusb = display->parent;

	dbg("cpad_light_off() called.");

	SYNCHECK_NR(synusb);

	mutex_lock(&display->io_mutex);
	if (!synusb->interface) {
		dbg("Device is gone.");
		return;
	}
	if (cpad_nlcd_function(display, CPAD_W_LIGHT, 0) < 0)
		err("error on cpad_light_off");
	dbg("cpad_light_off() is returning.");
	mutex_unlock(&display->io_mutex);
}

static int cpad_flash(struct synusb *synusb, int time)
{
	struct delayed_work *flash = &synusb->display->flash;
	int res;

	if (time <= 0)
		return -EINVAL;
	cancel_delayed_work(flash);
	res = cpad_nlcd_function(synusb->display, CPAD_W_LIGHT, 1);
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

	dbg("cpad_ioctl() called.");

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
		res = cpad_nlcd_function(synusb->display, CPAD_W_LIGHT, cval);
		break;

	case CPAD_FLASH:
		res = cpad_flash(synusb, ival);
		break;

	case CPAD_WLCD:
		res = cpad_nlcd_function(synusb->display, CPAD_W_LCD, cval);
		break;

	case CPAD_RLIGHT:
		res = cpad_nlcd_function(synusb->display, CPAD_R_LIGHT, 0);
		break;

	case CPAD_RLCD:
		res = cpad_nlcd_function(synusb->display, CPAD_R_LCD, 0);
		break;

	case CPAD_RESET:
		err("CPAD_RESET deprecated. Use libusb to reset cPad.");
		res = -ENOIOCTLCMD;
		// usb_reset_device(synusb->udev);
		break;

	case CPAD_REEPROM:
		res = cpad_nlcd_function(synusb->display, CPAD_R_ROM, 0);
		break;

	default:
		res = -ENOIOCTLCMD;
	}
error:
	mutex_unlock(&synusb->display->io_mutex);
	if (res < 0)
		goto done;

	/* write data to user */
	if ((cmd & IOC_OUT) && (rval != NULL))
		if (copy_to_user((void *) arg, rval, _IOC_SIZE(cmd)))
			res = -EFAULT;
done:
	dbg("Result of cpad_ioctl() is %i.", res);
	return res < 0 ? res : 0;
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
	if (!synusb->display)
		return 0;
	if ((synusb->display->in) || (endpoint->wMaxPacketSize != 32))
		return -ENODEV;

	return cpad_init_bulk(synusb, &synusb->display->in, 32, cpad_in_callback,
			      usb_rcvbulkpipe(synusb->udev,
					      endpoint->bEndpointAddress));
}

static int cpad_setup_out(struct synusb *synusb,
			  struct usb_endpoint_descriptor *endpoint)
{
	if (!synusb->display)
		return 0;
	if ((synusb->display->out) || (endpoint->wMaxPacketSize != 32))
		return -ENODEV;

	return cpad_init_bulk(synusb, &synusb->display->out, 274*32, cpad_out_callback,
			      usb_sndbulkpipe(synusb->udev,
					      endpoint->bEndpointAddress));
}

static int cpad_init(struct synusb *synusb)
{
	struct syndisplay *display = synusb->display;
	int retval;

	if (!display)
		return 0;

	retval = usb_register_dev(synusb->interface, &cpad_class);
	if (retval) {
		err("Not able to get a minor for this device.");
		return retval;
	}
	printk(KERN_INFO "cpad registered on minor: %d\n", synusb->interface->minor);

	init_completion(&display->cpad_done);
	INIT_DELAYED_WORK(&display->flash, cpad_light_off);
	init_usb_anchor(&display->submitted);
	mutex_init(&display->io_mutex);
	mutex_init(&display->open_mutex);

	return 0;
}

#endif /* CONFIG_USB_CPADDEV */


/*
 * init etc.
 */

static int synusb_setup_iurb(struct synusb *synusb,
			     struct usb_endpoint_descriptor *endpoint)
{
	char *buf;

	if ((synusb->iurb) || (endpoint->wMaxPacketSize < 8))
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
	if (synusb->display) {
		if (num != 3) {
			err("Found %i device(s), should be 3.", num);
			return -ENODEV;
		} else
			return 0;
	}
#endif /* CONFIG_USB_CPADDEV */

	if (num != 1) {
		err("Found %i device(s), should be 1.", num);
		return -ENODEV;
	}

	return 0;
}

static int synusb_detect_type(struct synusb *synusb,
			      const struct usb_device_id *id)
{
	int int_num = synusb->interface->cur_altsetting->desc.bInterfaceNumber;
	unsigned altsetting = min((unsigned) 1, synusb->interface->num_altsetting);
	int has_display = 0;
	int retval = 0;

	if (id->idVendor == USB_VID_SYNAPTICS) {
		switch (id->idProduct) {
		case USB_DID_SYN_TS:
			synusb->input_type = screen;
			break;
		case USB_DID_SYN_STICK:
			synusb->input_type = stick;
			break;
		case USB_DID_SYN_COMP_TP:
			if (int_num == 1)
				synusb->input_type = stick;
			else
				synusb->input_type = pad;
			break;
		/* FIXME: Are the displays of cPad and DisplayPad compatible?
			   If yes, uncomment the following line: */
		//case USB_DID_SYN_DP:
		case USB_DID_SYN_CPAD:
			has_display = 1;
			altsetting = min((unsigned) 2,
					 synusb->interface->num_altsetting);
		default:
			synusb->input_type = pad;
		}
	} else {
		synusb->input_type = pad;
	}

	retval = usb_set_interface(synusb->udev, int_num, altsetting);
	if (retval) {
		err("Can not set alternate setting to %i, error: %i",
		    altsetting, retval);
		goto error;
	}

#ifdef CONFIG_USB_CPADDEV
	if (has_display) {
		synusb->display = kzalloc(sizeof(*(synusb->display)), GFP_KERNEL);
		if (synusb->display == NULL) {
			err("Out of memory");
			goto error;
		}
		synusb->display->parent = synusb;
	}
#else
	synusb->display = has_display;
#endif
error:
	return retval;
}

static int synusb_probe(struct usb_interface *interface,
			const struct usb_device_id *id)
{
	struct synusb *synusb = NULL;
	struct usb_device *udev = interface_to_usbdev(interface);
	int retval = -ENOMEM;

	dbg("synusb_probe() called.");

	synusb = kzalloc(sizeof(*synusb), GFP_KERNEL);
	if (synusb == NULL) {
		err("Out of memory");
		goto error;
	}

	synusb->check = SYNUSB_CHECK_INIT;
	synusb->udev = usb_get_dev(udev);
	synusb->interface = interface;
	kref_init(&synusb->kref);
	usb_set_intfdata(interface, synusb);

	retval = synusb_detect_type(synusb, id);
	if (retval)
		goto error;

	retval = synusb_setup_endpoints(synusb);
	if (retval) {
		err("Can not set up endpoints, error: %i", retval);
		goto error;
	}

	retval = synusb_init_input(synusb);
	if (retval)
		goto error;

#ifdef CONFIG_USB_CPADDEV
	retval = cpad_init(synusb);
	if (retval)
		goto error;
#endif /* CONFIG_USB_CPADDEV */

	synusb->check = SYNUSB_CHECK;

	INIT_DELAYED_WORK(&synusb->isubmit, synusb_submit_int);
	schedule_delayed_work(&synusb->isubmit, HZ/100);

	dbg("synusb_probe() succeded.");

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

	dbg("synusb_disconnect() called.");

	synusb = usb_get_intfdata(interface);
	SYNCHECK_NR(synusb);
	usb_set_intfdata(interface, NULL);

#ifdef CONFIG_USB_CPADDEV
	if (synusb->display) {
		usb_deregister_dev(interface, &cpad_class);

		/* prevent more I/O from starting */
		mutex_lock(&synusb->display->io_mutex);
		mutex_lock(&synusb->display->open_mutex);
		synusb->interface = NULL;
		mutex_unlock(&synusb->display->open_mutex);
		mutex_unlock(&synusb->display->io_mutex);

		cancel_delayed_work(&synusb->display->flash);
	}
#endif /* CONFIG_USB_CPADDEV */

	cancel_delayed_work(&synusb->isubmit);
	flush_scheduled_work();

	usb_kill_urb(synusb->iurb);
	input_unregister_device(synusb->idev);
	synusb->idev = 0;

	kref_put(&synusb->kref, synusb_delete);

	printk(KERN_INFO "Synaptics device disconnected\n");
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,23)

#ifdef CONFIG_USB_CPADDEV
static void cpad_draw_down(struct syndisplay *display)
{
	int time;

	if (!display)
		return;
	time = usb_wait_anchor_empty_timeout(&display->submitted, 1000);
	if (!time)
		usb_kill_anchored_urbs(&display->submitted);
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

	dbg("synusb_suspend() called.");

	if (!synusb)
		return 0;
	SYNCHECK(synusb);

#ifdef CONFIG_USB_CPADDEV
	if (synusb->display) {
		turn_light_off = delayed_work_pending(&synusb->display->flash);
		//cancel_delayed_work_sync(&synusb->display->flash);
		cancel_delayed_work(&synusb->display->flash);
	}
	if (turn_light_off) {
		//dbg("Turning backlight off before suspend.");
		//cpad_draw_down(synusb->display);
		//if (cpad_nlcd_function(synusb->display, CPAD_W_LIGHT, 0) < 0)
			err("FIXME: turn backlight off before suspend");
	}
	cpad_draw_down(synusb->display);
#endif /* CONFIG_USB_CPADDEV */

	synusb_draw_down(synusb);

	dbg("synusb_suspend() succeded.");

	return 0;
}

static int synusb_resume(struct usb_interface *intf)
{
	struct synusb *synusb = usb_get_intfdata(intf);
	int res;

	dbg("synusb_resume() called.");

	SYNCHECK(synusb);

	res = usb_submit_urb(synusb->iurb, GFP_ATOMIC);
	if (res)
		err("usb_submit_urb int in failed during resume with result %d", res);

	dbg("synusb_resume() done.");

	return res;
}

static int synusb_reset_resume(struct usb_interface *intf)
{
	struct synusb *synusb = usb_get_intfdata(intf);
	int res;

	dbg("synusb_reset_resume() called.");

	SYNCHECK(synusb);

#ifdef CONFIG_USB_CPADDEV
	if (synusb->display) {
		synusb->display->error = -EPIPE;
		synusb->display->has_indata = 0;
	}
#endif /* CONFIG_USB_CPADDEV */

	res = usb_submit_urb(synusb->iurb, GFP_ATOMIC);
	if (res)
		err("usb_submit_urb int in failed during resume with result %d", res);

	dbg("synusb_reset_resume() done.");

	return res;
}

static int synusb_pre_reset(struct usb_interface *intf)
{
	struct synusb *synusb = usb_get_intfdata(intf);

	dbg("synusb_pre_reset() called.");

	SYNCHECK(synusb);

#ifdef CONFIG_USB_CPADDEV
	if (synusb->display) {
		mutex_lock(&synusb->display->io_mutex);
		cancel_delayed_work(&synusb->display->flash);
	}
#endif /* CONFIG_USB_CPADDEV */
	synusb_draw_down(synusb);

	dbg("synusb_pre_reset() succeded.");

	return 0;
}

static int synusb_post_reset(struct usb_interface *intf)
{
	struct synusb *synusb = usb_get_intfdata(intf);
	int res;

	dbg("synusb_post_reset() called.");

	SYNCHECK(synusb);

#ifdef CONFIG_USB_CPADDEV
	if (synusb->display) {
		synusb->display->error = -EPIPE;
		synusb->display->has_indata = 0;
		mutex_unlock(&synusb->display->io_mutex);
	}
#endif /* CONFIG_USB_CPADDEV */

	res = usb_submit_urb(synusb->iurb, GFP_KERNEL);
	if (res)
		err("usb_submit_urb int in failed during post_reset with result %d", res);

	dbg("synusb_post_reset() done.");

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

	init_completion(&test_completion);

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
