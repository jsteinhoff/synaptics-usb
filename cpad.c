/****
 * USB Synaptics Touchpad/cPad driver
 *
 *  Copyright (c) 2002 Rob Miller (rob@inpharmatica . co . uk)
 *  Copyright (c) 2003 Ron Lee (ron@debian.org)
 *	cPad driver for kernel 2.4
 *
 *  Copyright (c) 2004 Jan Steinhoff <jan.steinhoff@uni-jena.de>
 *  Copyright (c) 2004 Ron Lee (ron@debian.org)
 *	rewritten for kernel 2.6
 *
 * Bases on: 	usb_skeleton.c v2.0 by Greg Kroah-Hartman (greg@kroah.com)
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

/****
 * the cPad is an USB touchpad with background display (240x160 mono)
 * it has one interface with three possible alternate settings
 * 	setting 0: one int endpoint for rel movement (used by usbhid.ko)
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


#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <asm/uaccess.h>
#include <linux/usb.h>
#include <linux/moduleparam.h>

#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,5)
#include <linux/kref.h>
#endif
#include "kernel-compatibility.h"
#include "cpad.h"

#undef err
#undef info
#undef warn
/* redefine them to be a little less noisy in the log */
#define err(format, arg...) printk(KERN_ERR "cpad: " format "\n", ## arg)
#define info(format, arg...) printk(KERN_INFO "cpad: " format "\n", ## arg)
#define warn(format, arg...) printk(KERN_WARNING "cpad: " format "\n", ## arg)

#define DRIVER_VERSION "v1.2"
#define DRIVER_AUTHOR	"Rob Miller (rob@inpharmatica . co . uk), "\
			"Ron Lee (ron@debian.org), "\
			"Jan Steinhoff <jan.steinhoff@uni-jena.de>"
#define DRIVER_DESC "USB Synaptics cPad Driver"

#define USB_CPAD_MINOR_BASE	192
#define CPAD_DRIVER_NUM 	8

#define USB_VENDOR_ID_SYNAPTICS 0x06cb
#define USB_DEVICE_ID_SYN_USB   0x0002
#define USB_DEVICE_ID_CPAD      0x0003

static struct usb_device_id cpad_idtable [] = {
	{ USB_DEVICE(USB_VENDOR_ID_SYNAPTICS, USB_DEVICE_ID_CPAD) },
	{ USB_DEVICE(USB_VENDOR_ID_SYNAPTICS, USB_DEVICE_ID_SYN_USB) },
	{ }
};
MODULE_DEVICE_TABLE (usb, cpad_idtable);


struct cpad_context {
	struct usb_device *	udev;
	struct usb_interface *	interface;
	struct kref		kref;

	/* character device */
	int			no_display;
	struct semaphore	sem;
	int			gone;
	struct urb		*in, *out;
	wait_queue_head_t	wait;
	int			done;
	int			error;
	struct work_struct	flash;

	/* input device */
	struct input_dev	idev;
	char			iphys[64];
	struct work_struct	isubmit;
	struct urb *		iurb;
};
#define to_cpad_dev(d) container_of(d, struct cpad_context, kref)

static struct usb_driver cpad_driver;

static inline void cpad_free_urb(struct urb *urb)
{
	if (!urb)
		return;
	usb_buffer_free(urb->dev, urb->transfer_buffer_length,
			urb->transfer_buffer, urb->transfer_dma);
	usb_free_urb(urb);
}

static void cpad_delete(struct kref *kref)
{
	struct cpad_context *cpad = to_cpad_dev(kref);

	cpad_free_urb(cpad->iurb);
	cpad_free_urb(cpad->in);
	cpad_free_urb(cpad->out);

	usb_put_dev(cpad->udev);
	kfree (cpad);
}


/*
 * character device
 */

static int cpad_open(struct inode *inode, struct file *file)
{
	struct cpad_context *cpad;
	struct usb_interface *interface;
	int subminor;

	subminor = iminor(inode);

	interface = usb_find_interface(&cpad_driver, subminor);
	if (!interface) {
		err ("Can not find device for minor %d", subminor);
		return -ENODEV;
	}

	cpad = usb_get_intfdata(interface);
	if (!cpad)
		return -ENODEV;

	kref_get(&cpad->kref);

	file->private_data = cpad;

	return 0;
}

static int cpad_release(struct inode *inode, struct file *file)
{
	struct cpad_context *cpad;

	cpad = (struct cpad_context *)file->private_data;
	if (cpad == NULL)
		return -ENODEV;

	kref_put(&cpad->kref, cpad_delete);
	return 0;
}

static inline int cpad_down(struct cpad_context* cpad, struct file *file)
{
	if (down_trylock(&cpad->sem)) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		if (down_interruptible(&cpad->sem))
			return -ERESTARTSYS;
	}
	if (!cpad->gone)
		return 0;
	up(&cpad->sem);
	return -ENODEV;
}

static ssize_t cpad_read(struct file *file, char *buffer,
			 size_t count, loff_t *ppos)
{
	struct cpad_context *cpad;
	int retval;

	cpad = (struct cpad_context *)file->private_data;

	retval = cpad_down(cpad, file);
	if (retval)
		return retval;

	if (cpad->error) {
		retval = cpad->error;
		goto error;
	}

	count = min(count, (size_t)cpad->in->actual_length);
	if (copy_to_user(buffer, cpad->in->transfer_buffer, count))
		retval = -EFAULT;
	else
		retval = count;
error:
	up(&cpad->sem);
	return retval;
}

static void cpad_in_callback(struct urb *urb, struct pt_regs *regs)
{
	struct cpad_context *cpad;

	cpad = (struct cpad_context *)urb->context;

	cpad->done = 1;
	wake_up_interruptible(&cpad->wait);
}

static void cpad_out_callback(struct urb *urb, struct pt_regs *regs)
{
	struct cpad_context *cpad;

	cpad = (struct cpad_context *)urb->context;

	if (urb->status)
		goto error;

	cpad->error = usb_submit_urb(cpad->in, GFP_ATOMIC);
	if (!cpad->error)
		return;
	err("usb_submit_urb bulk in failed, error %d", cpad->error);
error:
	cpad_in_callback(urb, regs);
}

/* send out and in urbs synchronously */
static int cpad_submit_bulk(struct cpad_context* cpad)
{
	int retval;

	cpad->error = 0;
	cpad->done = 0;
	retval = usb_submit_urb(cpad->out, GFP_KERNEL);
	if (retval) {
		err("usb_submit_urb bulk out failed, error %d", cpad->error);
		goto error;
	}

	retval = wait_event_interruptible_timeout(cpad->wait, cpad->done, 2*HZ);
	if (retval <= 0) {
		usb_kill_urb(cpad->out);
		usb_kill_urb(cpad->in);
		retval = retval ? retval : -ETIMEDOUT;
		goto error;
	}

	if (cpad->out->status) {
		retval =  cpad->out->status;
	} else if (cpad->error) {
		retval = cpad->error;
	} else
		retval = cpad->in->status;
error:
	if (retval < 0)
		cpad->error = retval;
	return retval;
}

static inline unsigned char *cpad_1335_fillpacket
		(unsigned char cmd, unsigned char *param,
		 size_t param_size, unsigned char *out_buf)
{
	unsigned char *point;

	/* select 1335, set 1335 command, reverse params */
	*(out_buf++) = SEL_1335;
	*(out_buf++) = cmd;
	for (point=param+param_size-1; point>=param; point--)
		*(out_buf++) = *point;
	return out_buf;
}

static int cpad_write_fillbuffer(struct cpad_context *cpad,
				 const unsigned char *ubuffer, size_t count)
{
	unsigned char *out_buf = cpad->out->transfer_buffer;
	const unsigned char *ubuffer_orig = ubuffer;
	unsigned char param[30];
	size_t param_size, actual_length;
	unsigned char cmd;

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
		cpad->out->transfer_buffer_length = 2;
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

	cpad->out->transfer_buffer_length = actual_length;
	return ubuffer-ubuffer_orig;
}

static ssize_t cpad_write(struct file *file, const char *user_buffer,
			  size_t count, loff_t *ppos)
{
	struct cpad_context *cpad;
	int length, retval;

	cpad = (struct cpad_context *)file->private_data;

	if (count == 0)
		return 0;

	retval = cpad_down(cpad, file);
	if (retval)
		return retval;

	length = cpad_write_fillbuffer(cpad, user_buffer, count);
	if (length < 0) {
		retval = length;
		goto error;
	}

	retval = cpad_submit_bulk(cpad);
	if (!retval)
		retval = length;
error:
	up(&cpad->sem);
	return retval;
}

static int cpad_nlcd_function(struct cpad_context *cpad, unsigned char func,
			      unsigned char val)
{
	unsigned char *out_buf = cpad->out->transfer_buffer;
	int retval;

	if ((func < (char) 2) || (func > (char) 6)) {
		err("1335 nlcd invalid cmd");
		return -EINVAL;
	}

	*(out_buf++) = SEL_CPAD;
	*(out_buf++) = func;
	*(out_buf++) = val;
	cpad->out->transfer_buffer_length = 3;
	((unsigned char *)cpad->in->transfer_buffer)[2] = 0;

	retval = cpad_submit_bulk(cpad);
	if (!retval)
		retval = ((unsigned char *)cpad->in->transfer_buffer)[2];

	return retval;
}

static void cpad_light_off(void *arg)
{
	struct cpad_context *cpad;

	cpad = (struct cpad_context *)arg;

	down (&cpad->sem);
	if (cpad_nlcd_function(cpad, CPAD_W_LIGHT, 0) < 0)
		err("error on timed_light_off");
	up (&cpad->sem);
}

static int cpad_flash(struct cpad_context *cpad, int time)
{
	struct work_struct *flash = &cpad->flash;
	int res;

	if (time <= 0)
		return -EINVAL;
	cancel_delayed_work(&cpad->flash);
	res = cpad_nlcd_function(cpad, CPAD_W_LIGHT, 1);
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
	struct cpad_context *cpad;
	unsigned char cval = 0;
	int ival = 0;
	void *rval = NULL;
	int res = 0;

	cpad = (struct cpad_context *)file->private_data;

	res = cpad_down(cpad, file);
	if (res)
		return res;

	/* read data from user */
	if (cmd & IOC_IN) {
		res = -EFAULT;
		switch (_IOC_SIZE(cmd)) {
		case sizeof(char):
			if (get_user(cval, (char *) arg))
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
		rval = &cpad->idev.id;
		break;

	case CPAD_WLIGHT:
		res = cpad_nlcd_function(cpad, CPAD_W_LIGHT, cval);
		break;

	case CPAD_FLASH:
		res = cpad_flash(cpad, ival);
		break;

	case CPAD_WLCD:
		res = cpad_nlcd_function(cpad, CPAD_W_LCD, cval);
		break;

	case CPAD_RLIGHT:
		res = cpad_nlcd_function(cpad, CPAD_R_LIGHT, 0);
		break;

	case CPAD_RLCD:
		res = cpad_nlcd_function(cpad, CPAD_R_LCD, 0);
		break;

	case CPAD_RESET:
		usb_reset_device(cpad->udev);
		break;

	case CPAD_REEPROM:
		res = cpad_nlcd_function(cpad, CPAD_R_ROM, 0);
		break;

	default:
		res = -ENOIOCTLCMD;
	}
error:
	up (&cpad->sem);
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
	.mode =		S_IFCHR | S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP,
	.minor_base =	USB_CPAD_MINOR_BASE,
};


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

static char cpad_input_name[] = "Synaptics cPad";

static void cpad_input_callback(struct urb *urb, struct pt_regs *regs)
{
	struct cpad_context *cpad = (struct cpad_context *)urb->context;
	unsigned char *data = urb->transfer_buffer;
	struct input_dev *idev = &cpad->idev;
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

	input_regs (idev, regs);

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
	input_report_key (idev, BTN_MIDDLE, data[1] & 0x08);
	input_sync (idev);
resubmit:
	res = usb_submit_urb (urb, GFP_ATOMIC);
	if (res)
		err ("usb_submit_urb int in failed with result %d", res);
}

/* data must always be fetched from the int endpoint, otherwise the cpad would
 * reconnect to force driver reload, so this is always scheduled by probe
 */
static void cpad_submit_int(void *arg)
{
	struct cpad_context *cpad = (struct cpad_context *)arg;
	int res;

	res = usb_submit_urb (cpad->iurb, GFP_KERNEL);
	if (res)
		err ("usb_submit_urb int in failed with result %d", res);
}

static void cpad_init_input(struct cpad_context *cpad)
{
	struct input_dev *idev = &cpad->idev;
	struct usb_device *udev = cpad->udev;
	char path[64];

	set_bit (EV_ABS, idev->evbit);
	set_bit (ABS_X, idev->absbit);
	set_bit (ABS_Y, idev->absbit);
	set_bit (ABS_PRESSURE, idev->absbit);
	set_bit (ABS_TOOL_WIDTH, idev->absbit);
	idev->absmin[ABS_X] = xmin;
	idev->absmax[ABS_X] = xmax;
	idev->absmin[ABS_Y] = ymin;
	idev->absmax[ABS_Y] = ymax;
	idev->absmax[ABS_PRESSURE] = 255;
	idev->absmax[ABS_TOOL_WIDTH] = 15;

	set_bit (EV_KEY, idev->evbit);
	set_bit (BTN_TOUCH, idev->keybit);
	set_bit (BTN_TOOL_FINGER, idev->keybit);
	set_bit (BTN_TOOL_DOUBLETAP, idev->keybit);
	set_bit (BTN_TOOL_TRIPLETAP, idev->keybit);
	set_bit (BTN_LEFT, idev->keybit);
	set_bit (BTN_RIGHT, idev->keybit);
	set_bit (BTN_MIDDLE, idev->keybit);

	usb_make_path (udev, path, 56);
	sprintf (cpad->iphys, "%s/input0", path);
	idev->phys = cpad->iphys;
	idev->name = cpad_input_name;
	idev->id.bustype = BUS_USB;
	idev->id.vendor = udev->descriptor.idVendor;
	idev->id.product = udev->descriptor.idProduct;
	idev->id.version = udev->descriptor.bcdDevice;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,2)
	idev->dev = &cpad->interface->dev;
#endif

	input_register_device(idev);

	INIT_WORK (&cpad->isubmit, cpad_submit_int, cpad);
	schedule_delayed_work (&cpad->isubmit, HZ/100);
}


/*
 * init etc.
 */

static int cpad_init_bulk(struct cpad_context *cpad, struct urb **urb,
			  size_t size, usb_complete_t bulk_callback, int pipe)
{
	char *buf;

	*urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!*urb)
		return -ENOMEM;
	buf = usb_buffer_alloc(cpad->udev, size, GFP_KERNEL,
			       &(*urb)->transfer_dma);
	if (!buf)
		return -ENOMEM;
	usb_fill_bulk_urb(*urb, cpad->udev, pipe, buf, size,
			  bulk_callback, cpad);
	(*urb)->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	return 0;
}

static int cpad_setup_in(struct cpad_context *cpad,
			 struct usb_endpoint_descriptor *endpoint)
{
	if ((cpad->in) || (endpoint->wMaxPacketSize != 32))
		return -ENODEV;

	return cpad_init_bulk(cpad, &cpad->in, 32, cpad_in_callback,
			      usb_rcvbulkpipe(cpad->udev,
					      endpoint->bEndpointAddress));
}

static int cpad_setup_out(struct cpad_context *cpad,
			  struct usb_endpoint_descriptor *endpoint)
{
	if ((cpad->out) || (endpoint->wMaxPacketSize != 32))
		return -ENODEV;

	return cpad_init_bulk(cpad, &cpad->out, 274*32, cpad_out_callback,
			      usb_sndbulkpipe(cpad->udev,
					      endpoint->bEndpointAddress));
}

static int cpad_setup_iurb(struct cpad_context *cpad,
			   struct usb_endpoint_descriptor *endpoint)
{
	char *buf;

	if (cpad->iurb)
		return -ENODEV;
	cpad->iurb = usb_alloc_urb(0, GFP_KERNEL);
	if (!cpad->iurb)
		return -ENOMEM;
	buf = usb_buffer_alloc(cpad->udev, 8, SLAB_ATOMIC,
			       &cpad->iurb->transfer_dma);
	if (!buf)
		return -ENOMEM;
	usb_fill_int_urb(cpad->iurb, cpad->udev,
			 usb_rcvintpipe(cpad->udev, endpoint->bEndpointAddress),
			 buf, 8, cpad_input_callback,
			 cpad, endpoint->bInterval);
	cpad->iurb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	return 0;
}

static struct cpad_endpoint_table {
	__u8 dir;
	__u8 xfer_type;
	int  (*setup)(struct cpad_context *, struct usb_endpoint_descriptor *);
} cpad_endpoints [] = {
	{ USB_DIR_IN,  USB_ENDPOINT_XFER_BULK, cpad_setup_in },
	{ USB_DIR_OUT, USB_ENDPOINT_XFER_BULK, cpad_setup_out },
	{ USB_DIR_IN,  USB_ENDPOINT_XFER_INT, cpad_setup_iurb },
	{ }
};

static inline int cpad_match_endpoint(struct usb_endpoint_descriptor *ep)
{
	int i;

	for (i=0; cpad_endpoints[i].setup; i++)
		if (( (ep->bEndpointAddress & USB_DIR_IN)
				== cpad_endpoints[i].dir		) &&
		    ( (ep->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
				== cpad_endpoints[i].xfer_type		))
			return i;
	return -1;
}

static int cpad_setup_endpoints(struct cpad_context *cpad)
{
	struct usb_host_interface *iface_desc = cpad->interface->cur_altsetting;
	struct usb_endpoint_descriptor *endpoint;
	int i, j, res;
	int num = 0;

	for (i = 0; i < iface_desc->desc.bNumEndpoints; i++) {
		endpoint = &iface_desc->endpoint[i].desc;

		j = cpad_match_endpoint(endpoint);
		if (j >= 0) {
			res = cpad_endpoints[j].setup(cpad, endpoint);
			if (res)
				return res;
			num++;
		}
	}

	if (cpad->no_display) {
		if (num != 1)
			return -ENODEV;
	} else {
		if (num != 3)
			return -ENODEV;
	}

	return 0;
}

static int cpad_probe(struct usb_interface *interface,
		      const struct usb_device_id *id)
{
	struct cpad_context *cpad = NULL;
	struct usb_device *udev = interface_to_usbdev(interface);
	int ifnum, no_display, retval = -ENOMEM;

	no_display = (id->idProduct == USB_DEVICE_ID_CPAD) ? 0 : 1;

	ifnum = interface->cur_altsetting->desc.bInterfaceNumber;
	if (usb_set_interface (udev, ifnum, no_display ? 1 : 2))
		return -ENODEV;

	cpad = kmalloc(sizeof(*cpad), GFP_KERNEL);
	if (cpad == NULL) {
		err("Out of memory");
		goto error;
	}
	memset(cpad, 0x00, sizeof(*cpad));
	cpad->no_display = no_display;
	kref_init(&cpad->kref);

	cpad->udev = usb_get_dev(udev);
	cpad->interface = interface;
	retval = cpad_setup_endpoints(cpad);
	if (retval) {
		err("Could not set up endpoints, error: %i", retval);
		goto error;
	}
	usb_set_intfdata(interface, cpad);
	cpad_init_input(cpad);

	if (no_display)
		return 0;

	retval = usb_register_dev(interface, &cpad_class);
	if (retval) {
		err("Not able to get a minor for this device.");
		usb_set_intfdata(interface, NULL);
		goto error;
	}
	info("cpad registered on minor: %d", interface->minor);

	init_waitqueue_head(&cpad->wait);
	INIT_WORK(&cpad->flash, cpad_light_off, cpad);
	init_MUTEX(&cpad->sem);
	return 0;

error:
	if (cpad)
		kref_put(&cpad->kref, cpad_delete);
	return retval;
}

static void cpad_disconnect(struct usb_interface *interface)
{
	struct cpad_context *cpad;

	lock_kernel();

	cpad = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);

	if (cpad->no_display) {
		unlock_kernel();
	} else {
		usb_deregister_dev(interface, &cpad_class);

		unlock_kernel();

		/* make sure no more work is scheduled */
		down(&cpad->sem);
		cpad->gone = 1;
		up(&cpad->sem);
		/* cancel the rest */
		cancel_delayed_work(&cpad->flash);
	}

	cancel_delayed_work(&cpad->isubmit);
	flush_scheduled_work();

	usb_kill_urb(cpad->iurb);
	input_unregister_device(&cpad->idev);

	kref_put(&cpad->kref, cpad_delete);

	info("cpad disconnected");
}

static struct usb_driver cpad_driver = {
	.owner =	THIS_MODULE,
	.name =		"cpad",
	.probe =	cpad_probe,
	.disconnect =	cpad_disconnect,
	.id_table =	cpad_idtable,
};

static int steal = 1;

static void repossess_devices( struct usb_driver *driver, struct usb_device_id *id )
{
	if (!steal)
		return;

	while( id->idVendor )
	{
		// XXX Not enough, we need to find ALL devices, not just the
		//     first of any particular model.
		struct usb_device *udev = usb_find_device(id->idVendor,id->idProduct);

		if (udev)
		{
			struct usb_interface    *interface;

			down_write( &udev->dev.bus->subsys.rwsem );
			usb_lock_device( udev );

			interface = usb_ifnum_to_if(udev, 0);

			if ( interface->dev.driver != &driver->driver )
			{
			    if (usb_interface_claimed(interface))
			    {
				    info("releasing '%s' from generic driver '%s'.",
					 interface->dev.kobj.k_name,
					 interface->dev.driver->name);
				    device_release_driver(&interface->dev);
			    }

			    driver_attach(&driver->driver);
			}
			else
			    info( "already attached" );

			usb_unlock_device( udev );
			up_write( &udev->dev.bus->subsys.rwsem );

			usb_put_dev(udev);
		}
		++id;
	}
}

static int rebind_in;

static int cpad_set_rebind(const char *val, struct kernel_param *kp)
{
	int retval;

	retval = param_set_int(val, kp);
        info("user rebind request %d", rebind_in);
	repossess_devices( &cpad_driver, cpad_idtable );
	return retval;
}

module_param_call(rebind, cpad_set_rebind, param_get_int, &rebind_in, 0664);
module_param_call(steal, cpad_set_rebind, param_get_int, &steal, 0664);

static int __init cpad_init(void)
{
	int result;

	result = usb_register(&cpad_driver);
	if (result)
		err("usb_register failed. Error number %d", result);
	else {
		info(DRIVER_DESC " " DRIVER_VERSION);
		repossess_devices( &cpad_driver, cpad_idtable );
	}

	return result;
}

static void __exit cpad_exit(void)
{
	usb_deregister(&cpad_driver);
}

module_init (cpad_init);
module_exit (cpad_exit);

MODULE_AUTHOR (DRIVER_AUTHOR);
MODULE_DESCRIPTION (DRIVER_DESC);
MODULE_LICENSE("GPL");
