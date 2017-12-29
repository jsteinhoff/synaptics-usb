/****
 * cPad character device part of the synaptics-usb driver
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
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * Trademarks are the property of their respective owners.
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
 * The cPad display is controlled by a Seiko/Epson 1335 LCD Controller IC.
 * In order to send commands to the sed1335, each packet in the urb must look
 * like this:
 *	02 <1335 command> [<data in reversed order> ...]
 * Possible commands for the sed1335 are listed in linux/cpad.h. The data must
 * be in reversed order as stated in the sed1335 data sheet.
 * The urb must be send to the bulk out endpoint. Because the packet size of
 * this endoint is 32 bytes, "02 <1335 command>" must be repeated after 30
 * bytes of data. The data must be in reversed order in each of these 30 bytes
 * blocks. All this is done automatically when writing to the character device.
 *
 * Functions that are not controlled by the sed1335, like the backlight, can
 * be accessed by
 *	01 <function> <state>
 * The packet must be send to the bulk out endpoint. These functions can be
 * accessed via ioctls. Possible functions are listed in cpad.h.
 *
 * An urb to the bulk out endpoint should be followed by an urb to the bulk in
 * endpoint. This gives the answer of the cpad/sed1335, accessible by reading
 * the character device */

#include "kconfig.h"

#ifdef CONFIG_USB_CPADDEV

#include "synusb-kcompat.h"

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,5)
#include <linux/kref.h>
#endif
#include <asm/uaccess.h>
#include <linux/usb.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/workqueue.h>
#include <linux/cpad.h>

#include "synapticsusb.h"
#include "cpad.h"

/*
 * helper functions
 */

static inline void cpad_unlock(struct syndisplay* display)
{
	mutex_unlock(&display->io_mutex);
}

/* lock io_mutex, notify about reset, interruptible */
static int cpad_lock(struct syndisplay* display)
{
	int retval;

	if (mutex_lock_interruptible(&display->io_mutex))
		return -ERESTARTSYS;
	if (display->gone || display->suspended) {
		retval = -ENODEV;
		goto error;
	}
	if (display->reset_in_progress || display->reset_notify) {
		display->reset_notify = 0;
		retval = -EPIPE;
		goto error;
	}
	return 0;
error:
	cpad_unlock(display);
	return retval;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
static void cpad_in_callback(struct urb *urb)
#else
static void cpad_in_callback(struct urb *urb, struct pt_regs *regs)
#endif
{
	struct cpad_urb *curb = (struct cpad_urb *)urb->context;

	if (synusb_urb_status_error(urb))
		err("nonzero read bulk status received: %d", urb->status);

	complete(&curb->display->done);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)
static void cpad_out_callback(struct urb *urb)
#else
static void cpad_out_callback(struct urb *urb, struct pt_regs *regs)
#endif
{
	struct cpad_urb *curb = (struct cpad_urb *)urb->context;
	struct syndisplay *display = curb->display;

	if (urb->status) {
		if (synusb_urb_status_error(urb))
			err("nonzero write bulk status received: %d", urb->status);
		goto complete;
	}

	display->error = usb_submit_urb(curb->in, GFP_ATOMIC);
	if (!display->error)
		return;
	err("usb_submit_urb bulk in failed, error %d", display->error);
complete:
	complete(&display->done);
}

/* Send out and in urbs synchronously. Call with io_mutex hold. */
static int cpad_submit(struct cpad_urb* curb)
{
	struct syndisplay *display = curb->display;
	int retval;

	INIT_COMPLETION(display->done);
	display->error = 0;

	retval = usb_submit_urb(curb->out, GFP_KERNEL);
	if (retval) {
		err("usb_submit_urb bulk out failed, error %d", retval);
		goto error;
	}

	retval = wait_for_completion_interruptible_timeout(&display->done, HZ*2);
	if (retval <= 0) {
		retval = retval ? retval : -ETIMEDOUT;
		usb_kill_urb(curb->out);
		usb_kill_urb(curb->in);
		goto error;
	}

	/* report error that occured first */
	if (curb->out->status) {
		retval = curb->out->status;
	} else if (display->error) {
		retval = display->error;
	} else
		retval = curb->in->status;
error:
	return retval;
}

/* as cpad_submit, but preserve notification about reset and update has_indata */
static int cpad_submit_user(struct syndisplay *display)
{
	int retval;

	retval = cpad_submit(display->user_urb);
	display->has_indata = retval < 0 ? 0 : 1;
	/* to preserve notifications about reset */
	if (retval)
		retval = (retval == -EPIPE) ? -EIO : retval;

	return retval;
}

static void cpad_urb_free(struct cpad_urb *curb)
{
	synusb_free_urb(curb->out);
	synusb_free_urb(curb->in);

	kfree(curb);
}

static struct urb* cpad_alloc_bulk(struct cpad_urb *curb, size_t size, usb_complete_t callback, int pipe)
{
	struct usb_device *udev = curb->display->parent->udev;
	struct urb *urb;
	char *buffer;

	/* create a urb, and a buffer for it */
	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb)
		return NULL;

	buffer = usb_buffer_alloc(udev, size, GFP_KERNEL, &urb->transfer_dma);
	if (!buffer)
		goto error;

	/* initialize the urb properly */
	usb_fill_bulk_urb(urb, udev, pipe, buffer, size, callback, curb);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	return urb;
error:
	usb_free_urb(urb);
	return NULL;
}

static struct cpad_urb* cpad_urb_alloc(struct syndisplay *display, int out_size)
{
	struct usb_device *udev = display->parent->udev;
	struct cpad_urb *curb;

	curb = kzalloc(sizeof(*curb), GFP_KERNEL);
	if (curb == NULL) {
		err("Out of memory");
		return NULL;
	}
	curb->display = display;

	curb->out = cpad_alloc_bulk(curb, out_size, cpad_out_callback,
				    usb_sndbulkpipe(udev, display->out_endpointAddr));
	if (curb->out == NULL)
		goto error;

	curb->in = cpad_alloc_bulk(curb, CPAD_PACKET_SIZE, cpad_in_callback,
				   usb_rcvbulkpipe(udev, display->in_endpointAddr));
	if (curb->in == NULL)
		goto error;

	return curb;
error:
	err("Out of memory");
	cpad_urb_free(curb);
	return NULL;
}

/*
 * cPad display character device code
 */

MODULE_PARM_DESC(exclusive_open,
	"if set, cPad character device can only be opened by one program at a time");
static int exclusive_open = 0;
module_param(exclusive_open, int, 0644);

static int cpad_open(struct inode *inode, struct file *file)
{
	struct synusb *synusb;
	struct syndisplay *display;
	struct usb_interface *interface;
	int subminor;
	int retval = 0;

	subminor = iminor(inode);

	interface = usb_find_interface(&synusb_driver, subminor);
	if (!interface) {
		err ("%s - error, can't find device for minor %d",
		     __func__, subminor);
		return -ENODEV;
	}

	synusb = usb_get_intfdata(interface);
	if (!synusb) {
		return -ENODEV;
	}
	display = synusb->display;

	/* increment our usage count for the device */
	kref_get(&synusb->kref);

	mutex_lock(&display->open_mutex);

	if (!display->open_count++) {
		/* prevent the device from being autosuspended */
		retval = usb_autopm_get_interface(interface);
			if (retval) {
				display->open_count--;
				mutex_unlock(&display->open_mutex);
				kref_put(&synusb->kref, synusb_delete);
				goto exit;
			}
	} else if (exclusive_open) {
		retval = -EBUSY;
		display->open_count--;
		mutex_unlock(&display->open_mutex);
		kref_put(&synusb->kref, synusb_delete);
		goto exit;
	}

	/* save our object in the file's private structure */
	file->private_data = display;
	mutex_unlock(&display->open_mutex);
exit:
	return retval;
}

static int cpad_release(struct inode *inode, struct file *file)
{
	struct synusb *synusb;
	struct syndisplay *display;

	display = (struct syndisplay *)file->private_data;
	if (display == NULL)
		return -ENODEV;
	synusb = display->parent;

	/* allow the device to be autosuspended */
	mutex_lock(&display->open_mutex);
	if (!--display->open_count && synusb->interface)
		usb_autopm_put_interface(synusb->interface);
	mutex_unlock(&display->open_mutex);

	/* decrement the count on our device */
	kref_put(&synusb->kref, synusb_delete);
	return 0;
}

static int cpad_flush(struct file *file, fl_owner_t id)
{
	struct syndisplay *display;
	int res;

	display = (struct syndisplay *)file->private_data;
	if (display == NULL)
		return -ENODEV;

	/* wait for io to stop */
	res = cpad_lock(display);
	if (!res)
		cpad_unlock(display);

	return res;
}

static ssize_t cpad_read(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
	struct syndisplay *display;
	int retval;
	size_t bytes_read;

	display = (struct syndisplay *)file->private_data;

	retval = cpad_lock(display);
	if (retval)
		return retval;

	if (!display->has_indata) {
		retval = -ENODATA;
		goto error;
	}

	/* copy the data to userspace */
	bytes_read = min(count, (size_t)display->user_urb->in->actual_length);
	if (copy_to_user(buffer, display->user_urb->in->transfer_buffer, count))
		retval = -EFAULT;
	else
		retval = bytes_read;
error:
	cpad_unlock(display);
	return retval;
}

static inline u8 *cpad_1335_fillpacket(u8 cmd, u8 *param, size_t param_size, u8 *out_buf)
{
	u8 *point;

	/* select 1335, set 1335 command, reverse params */
	*(out_buf++) = SEL_1335;
	*(out_buf++) = cmd;
	for (point=param+param_size-1; point>=param; point--)
		*(out_buf++) = *point;
	return out_buf;
}

static ssize_t cpad_write_fillbuffer(struct urb *out, const u8 *ubuffer, size_t count)
{
	u8 *out_buf = out->transfer_buffer;
	const u8 *ubuffer_orig = ubuffer;
	u8 param[CPAD_MAX_PARAM_SIZE];
	size_t param_size, actual_length;
	u8 cmd;

	/* get 1335 command first */
	if (get_user(cmd, ubuffer))
		return -EFAULT;
	ubuffer++;
	if (cmd == SLEEP_1335)
		pr_warning("Sleeping sed1335 might damage the display.\n");

	if (count == 1) {
		/* 1335 command without params */
		*(out_buf++) = SEL_1335;
		*(out_buf++) = cmd;
		actual_length = 2;
		goto exit;
	}

	actual_length = 0;
	count--;
	while (count > 0) {
		param_size = min(count, (size_t)30);
		if (actual_length+param_size+2 > CPAD_MAX_TRANSFER)
			break;

		if (copy_from_user(param, ubuffer, param_size))
			return -EFAULT;

		ubuffer += param_size;
		count -= param_size;

		out_buf = cpad_1335_fillpacket(cmd, param,
					       param_size, out_buf);
		actual_length += param_size + 2;
	}
exit:
	out->transfer_buffer_length = actual_length;
	return (ssize_t)(ubuffer-ubuffer_orig);
}

static ssize_t cpad_write(struct file *file, const char *user_buffer,
			  size_t count, loff_t *ppos)
{
	struct syndisplay *display;
	ssize_t retval = 0;
	ssize_t writesize;

	display = (struct syndisplay *)file->private_data;

	/* verify that we actually have some data to write */
	if (count == 0)
		return 0;

	retval = (ssize_t)cpad_lock(display);
	if (retval)
		return retval;

	writesize = cpad_write_fillbuffer(display->user_urb->out, (u8*) user_buffer, count);
	if (writesize < 0) {
		retval = writesize;
		goto error;
	}

	retval = (ssize_t)cpad_submit_user(display);
	if (!retval)
		retval = writesize;
error:
	cpad_unlock(display);
	return retval;
}

static int cpad_nlcd_function(struct syndisplay *display, u8 func, u8 val, int which_urb)
{
	struct cpad_urb *curb;
	u8 *out_buf;
	int retval = 0;

	if ((func <= (u8) CPAD_W_ROM) || (func >= (u8) CPAD_RSRVD)) {
		err("Invalid nlcd command.");
		return -EINVAL;
	}

	if (which_urb == CPAD_USER_URB) {
		curb = display->user_urb;
		curb->out->transfer_buffer_length = 3;
	} else
		curb = display->nlcd_urb;

	out_buf = curb->out->transfer_buffer;
	*(out_buf++) = SEL_CPAD;
	*(out_buf++) = func;
	*(out_buf++) = val;
	((u8 *)curb->in->transfer_buffer)[2] = 0;

	if (which_urb == CPAD_USER_URB)
		retval = cpad_submit_user(display);
	else
		retval = cpad_submit(curb);

	if (!retval) {
		retval = ((u8 *)curb->in->transfer_buffer)[2];
		if (func == CPAD_W_LIGHT)
			display->backlight_on = val ? 1 : 0;
	}

	return retval;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void cpad_light_off(void *work)
#else
static void cpad_light_off(struct work_struct *work)
#endif
{
	struct syndisplay *display = container_of(work, struct syndisplay, flash.work);

	mutex_lock(&display->io_mutex);
	if (display->gone || display->suspended || display->reset_in_progress || display->reset_notify)
		return;
	if (!display->backlight_on)
		return;
	if (cpad_nlcd_function(display, CPAD_W_LIGHT, 0, CPAD_NLCD_URB) < 0)
		err("error on cpad_light_off");
	mutex_unlock(&display->io_mutex);
}

static int cpad_flash(struct syndisplay *display, int time)
{
	struct delayed_work *flash = &display->flash;
	int res;

	if (time <= 0)
		return -EINVAL;
	cancel_delayed_work(flash);
	res = cpad_nlcd_function(display, CPAD_W_LIGHT, 1, CPAD_USER_URB);
	if (res)
		return res;
	time = min(time, 1000);
	schedule_delayed_work(flash, (HZ * (unsigned long) time) / 100);
	return 0;
}

static int cpad_driver_num = CPAD_DRIVER_NUM;

static int cpad_ioctl(struct inode *inode, struct file  *file,
		      unsigned int  cmd, unsigned long arg)
{
	struct syndisplay *display;
	u8 cval = 0;
	int ival = 0;
	void *rval = NULL;
	int res = 0;

	display = (struct syndisplay *)file->private_data;

	res = cpad_lock(display);
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
		rval = &display->parent->idev->id;
		break;

	case CPAD_WLIGHT:
		res = cpad_nlcd_function(display, CPAD_W_LIGHT, cval, CPAD_USER_URB);
		break;

	case CPAD_FLASH:
		res = cpad_flash(display, ival);
		break;

	case CPAD_WLCD:
		res = cpad_nlcd_function(display, CPAD_W_LCD, cval, CPAD_USER_URB);
		break;

	case CPAD_RLIGHT:
		res = cpad_nlcd_function(display, CPAD_R_LIGHT, 0, CPAD_USER_URB);
		break;

	case CPAD_RLCD:
		res = cpad_nlcd_function(display, CPAD_R_LCD, 0, CPAD_USER_URB);
		break;

	case CPAD_RESET:
		err("CPAD_RESET deprecated. Use libusb to reset cPad.");
		res = -ENOIOCTLCMD;
		break;

	case CPAD_REEPROM:
		res = cpad_nlcd_function(display, CPAD_R_ROM, 0, CPAD_USER_URB);
		break;

	default:
		res = -ENOIOCTLCMD;
	}
error:
	cpad_unlock(display);
	if (res < 0)
		goto exit;

	/* write data to user */
	if ((cmd & IOC_OUT) && (rval != NULL))
		if (copy_to_user((void *) arg, rval, _IOC_SIZE(cmd)))
			res = -EFAULT;
exit:
	return res < 0 ? res : 0;
}

static struct file_operations cpad_fops = {
	.owner =	THIS_MODULE,
	.read =		cpad_read,
	.write =	cpad_write,
	.ioctl =	cpad_ioctl,
	.open =		cpad_open,
	.release =	cpad_release,
	.flush =	cpad_flush,
};

/*
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with the driver core
 */
struct usb_class_driver cpad_class = {
	.name =		"usb/cpad%d",
	.fops =		&cpad_fops,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)
	.mode =		S_IFCHR | S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP,
#endif
	.minor_base =	USB_CPAD_MINOR_BASE,
};

/*
 * initialization and removal of data structures
 */

int cpad_alloc(struct synusb *synusb)
{
	if (!synusb->has_display)
		return 0;

	synusb->display = kzalloc(sizeof(*(synusb->display)), GFP_KERNEL);
	if (synusb->display == NULL) {
		err("Out of memory");
		return -ENOMEM;
	}
	synusb->display->parent = synusb;

	return 0;
}

void cpad_free(struct synusb *synusb)
{
	struct syndisplay *display = synusb->display;

	if (!display)
		return;

	if (display->user_urb->out)
		display->user_urb->out->transfer_buffer_length = CPAD_MAX_TRANSFER;
	cpad_urb_free(display->user_urb);
	cpad_urb_free(display->nlcd_urb);

	kfree(display);
	synusb->display = NULL;
}

static void cpad_disable(struct synusb *synusb)
{
	pr_warning("Disabling cPad display character device.\n");
	cpad_free(synusb);
}

int cpad_setup_in(struct synusb *synusb,
			 struct usb_endpoint_descriptor *endpoint)
{
	struct syndisplay *display = synusb->display;

	if (!display)
		return 0;
	if ((display->in_endpointAddr) || (endpoint->wMaxPacketSize != CPAD_PACKET_SIZE)) {
		cpad_disable(synusb);
		return 0;
	}

	display->in_endpointAddr = endpoint->bEndpointAddress;

	return 0;
}

int cpad_setup_out(struct synusb *synusb,
			  struct usb_endpoint_descriptor *endpoint)
{
	struct syndisplay *display = synusb->display;

	if (!display)
		return 0;
	if ((display->out_endpointAddr) || (endpoint->wMaxPacketSize != CPAD_PACKET_SIZE)) {
		cpad_disable(synusb);
		return 0;
	}

	display->out_endpointAddr = endpoint->bEndpointAddress;

	return 0;
}

int cpad_check_setup(struct synusb *synusb)
{
	if ((synusb->display->in_endpointAddr) && (synusb->display->out_endpointAddr))
		return 0;

	cpad_disable(synusb);
	return 0;
}

int cpad_init(struct synusb *synusb)
{
	struct syndisplay *display = synusb->display;
	int retval;

	if (!display)
		return 0;

	display->user_urb = cpad_urb_alloc(display, CPAD_MAX_TRANSFER);
	display->nlcd_urb = cpad_urb_alloc(display, 3);
	if (!(display->user_urb && display->nlcd_urb)) {
		retval = -ENOMEM;
		goto error;
	}

	mutex_init(&display->io_mutex);
	mutex_init(&display->open_mutex);
	init_completion(&display->done);
	INIT_DELAYED_WORK(&display->flash, cpad_light_off);

	retval = usb_register_dev(synusb->interface, &cpad_class);
	if (retval) {
		err("Not able to get a minor for this device.");
		goto error;
	}
	/* let the user know what node this device is now attached to */
	pr_info("USB cpad device now attached to cpad-%d.\n", synusb->interface->minor);

	return 0;
error:
	cpad_disable(synusb);
	return 0;
}

void cpad_disconnect(struct synusb *synusb)
{
	struct syndisplay *display = synusb->display;

	if (!display)
		return;

	/* give back our minor */
	usb_deregister_dev(synusb->interface, &cpad_class);

	/* prevent more I/O from starting */
	mutex_lock(&display->io_mutex);
	mutex_lock(&display->open_mutex);
	synusb->interface = NULL;
	mutex_unlock(&display->open_mutex);
	mutex_unlock(&display->io_mutex);
	cancel_delayed_work_sync(&display->flash);
}

/*
 * suspend code
 */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,23)

int cpad_suspend(struct synusb *synusb)
{
	struct syndisplay *display = synusb->display;
	int retval = 0;

	if (!display)
		return 0;

	if (mutex_lock_interruptible(&display->io_mutex))
		return -ERESTARTSYS;

	display->suspended = 1;
	cancel_delayed_work(&display->flash);
	if (display->backlight_on) {
		retval = cpad_nlcd_function(display, CPAD_W_LIGHT, 0, CPAD_NLCD_URB);
		if (retval < 0)
			err("Could not turn backlight off before suspend");
		else
			retval = 0;
	}

	mutex_unlock(&display->io_mutex);

	return retval;
}

int cpad_resume(struct synusb *synusb)
{
	struct syndisplay *display = synusb->display;

	if (!display)
		return 0;

	mutex_lock(&display->io_mutex);
	display->suspended = 0;
	mutex_unlock(&display->io_mutex);

	return 0;
}

int cpad_reset_resume(struct synusb *synusb)
{
	struct syndisplay *display = synusb->display;

	if (!display)
		return 0;

	mutex_lock(&display->io_mutex);
	display->reset_notify = 1;
	display->suspended = 0;
	mutex_unlock(&display->io_mutex);

	return 0;
}

int cpad_pre_reset(struct synusb *synusb)
{
	struct syndisplay *display = synusb->display;

	if (!display)
		return 0;

	if (mutex_lock_interruptible(&display->io_mutex))
		return -ERESTARTSYS;

	display->reset_in_progress = 1;
	display->reset_notify = 1;
	cancel_delayed_work(&synusb->display->flash);
	mutex_unlock(&display->io_mutex);

	return 0;
}

int cpad_post_reset(struct synusb *synusb)
{
	struct syndisplay *display = synusb->display;

	if (display)
		return 0;

	mutex_lock(&display->io_mutex);
	display->reset_in_progress = 0;
	mutex_unlock(&display->io_mutex);

	return 0;
}

#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,23) */

#endif /* CONFIG_USB_CPADDEV */
