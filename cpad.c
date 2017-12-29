/*
 * USB Synaptics cPad driver
 *
 *  Copyright (c) 2002 Rob Miller (rob@inpharmatica . co . uk)
 *  Copyright (c) 2003 Ron Lee (ron@debian.org)
 *	cPad driver for kernel 2.4
 *
 *  Copyright (c) 2004 Jan Steinhoff <jan.steinhoff@uni-jena.de>
 *	rewritten for kernel 2.6
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * Trademarks are the property of their respective owners.
 */

/*
 * the cPad is an USB touchpad with background display (240x160 mono)
 * it has one interface with three possible alternate settings
 * 	setting 0: one int endpoint for rel movement (used by hid.ko)
 *	setting 1: one int endpoint for abs finger position
 *	setting 2: one int endpoint for abs finger position and
 *		   two bulk endpoints for the display (in/out)
 * this driver uses setting 2
 * see cpad.h for details on the bulk endpoints
 */

/* includes */
#include <linux/usb.h>
#include <linux/input.h>
#include <linux/proc_fs.h>
#include <linux/fb.h>
#include <asm/uaccess.h>
#include <linux/mm.h>

#include "cpadconfig.h"
#include "cpad.h"

/* version information */
#define DRIVER_VERSION "v0.2"
#define DRIVER_AUTHOR "Rob Miller (rob@inpharmatica . co . uk), Ron Lee (ron@debian.org), Jan Steinhoff <jan.steinhoff@uni-jena.de>"
#define DRIVER_DESC "USB Synaptics cPad Driver"


/*****************************************************************************
 *	data types							     *
 *****************************************************************************/

struct cpad_endpoint {
	struct urb *		urb;			/* urb for this endpoint */
	struct usb_endpoint_descriptor * endpoint_desc;	/* descriptor of this endpoint */
	unsigned char *		buffer;			/* buffer to send/receive data */
	size_t			buffer_size;
};

struct cpad_context {
	struct usb_device *	udev;			/* usb device pointer */
	struct usb_interface *	interface;		/* interface for this device */
	struct semaphore	sem;			/* locks this structure */
	int			present;		/* if the device is not disconnected */

	/* display data */
	struct cpad_display {
		struct cpad_endpoint	in;		/* bulk in endpoint */
		struct cpad_endpoint	out;		/* bulk out endpoint */

		atomic_t		busy;		/* true if urb is busy */
		struct completion	finished;	/* wait for urb to finish */
		wait_queue_head_t	queue;		/* queue for interruptible wait */
		struct timer_list	timer;		/* timeout for urbs */
		int			timed_out;	/* true if urbs timed out */
		int			submit_res;	/* result of usb_submit_urb */

		unsigned char *		tmpbuf;		/* needed for copy_from_user */
		size_t			tmpbuf_size;
		struct work_struct	flash;		/* used to schedule flash */
	} display;

	/* touchpad data */
	struct cpad_touchpad {
		struct cpad_endpoint	in;		/* int endpoint */
		struct input_dev	idev;		/* input subsystem device */
		char			phys[64];
		int			open;		/* touchpad input device open counter */
		struct work_struct	submit_urb;	/* used to submit urb after probe */
	} touchpad;

	/* cpad character device data */
	struct cpad_char_dev {
		int			disable;
		unsigned char		minor;		/* the starting minor number for this device */
		int			open;		/* if the port is open or not */
	} char_dev;

	/* procfs data */
	struct cpad_procfs {
		int			num;		/* number of this device */
		struct proc_dir_entry *	entry;		/* procfs entry */
		char			name[16];	/* procfs filename */
	} procfs;

	/* framebuffer data */
	struct cpad_fb {
		int			disable;
		void *			videomemory;
		u32 			pseudo_palette[17];
		struct work_struct	drawimage;
		struct completion	draw_finished;
		struct fb_info		info;
		int			active;
		int			rate;
		int			dither;
		int 			brightness;
		int			invert;
	} fb;
};


/*****************************************************************************
 *	declarations and prototypes					     *
 *****************************************************************************/

/* usb */
static struct usb_driver cpad_driver;

static int cpad_probe(struct usb_interface *interface, const struct usb_device_id *id);
static void cpad_disconnect(struct usb_interface *interface);
static void cpad_display_out_callback(struct urb *urb, struct pt_regs *regs);
static void cpad_display_in_callback (struct urb *urb, struct pt_regs *regs);

static void cpad_free_context(struct cpad_context *cpad);
static int cpad_setup_endpoints(struct cpad_context *cpad);
static void cpad_submit_int_urb(void *arg);

static inline int cpad_check_display_urb_errors (struct cpad_context *cpad);
static int cpad_nlcd_function (struct cpad_context *cpad, const unsigned char func, const unsigned char val);

/* input */
static inline void cpad_input_init(struct cpad_context *cpad);
static inline void cpad_input_remove(struct cpad_context *cpad);
static void cpad_input_callback(struct urb *urb, struct pt_regs *regs);

/* character device */
static inline void cpad_dev_init(struct cpad_context *cpad);
static inline void cpad_dev_remove(struct cpad_context *cpad);

/* procfs */
static inline void cpad_procfs_init(void);
static inline void cpad_procfs_remove(void);
static inline void cpad_procfs_add(struct cpad_context *cpad);
static inline void cpad_procfs_del(struct cpad_context *cpad);

/* framebuffer */
static inline void cpad_fb_init(struct cpad_context *cpad);
static inline void cpad_fb_remove(struct cpad_context *cpad);
static void cpad_fb_activate(struct cpad_context *cpad, int rate);
static void cpad_fb_deactivate(struct cpad_context *cpad);


/*****************************************************************************
 *	module initialisation						     *
 *****************************************************************************/

static int __init cpad_init(void)
{
	cpad_procfs_init();
	int result = usb_register(&cpad_driver);
	if (result) {
		err("usb_register failed. Error number %d", result);
		cpad_procfs_remove();
		return result;
	}
	info(DRIVER_DESC " " DRIVER_VERSION);	
	return 0;
}

static void __exit cpad_exit(void)
{
	usb_deregister(&cpad_driver);
	cpad_procfs_remove();
}

module_init (cpad_init);
module_exit (cpad_exit);

MODULE_AUTHOR (DRIVER_AUTHOR);
MODULE_DESCRIPTION (DRIVER_DESC);
MODULE_LICENSE ("GPL");


/*****************************************************************************
 *	usb routines, based on:						     *
 *		drivers/usb/usb-skeleton.c v1.1				     *
 *****************************************************************************/

#define USB_VENDOR_ID_SYNAPTICS	0x06cb
#define USB_DEVICE_ID_CPAD	0x0003

static struct usb_device_id cpad_table [] = {
	{ USB_DEVICE(USB_VENDOR_ID_SYNAPTICS, USB_DEVICE_ID_CPAD) },
	{ 0, 0 }
};
MODULE_DEVICE_TABLE (usb, cpad_table);

static struct usb_driver cpad_driver = {
	.owner =	THIS_MODULE,
	.name =		"cpad",
	.probe =	cpad_probe,
	.disconnect =	cpad_disconnect,
	.id_table =	cpad_table,
};

static int cpad_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(interface);
	struct cpad_context *cpad;
	int res;

	/* activate alternate setting 2 */
	if (usb_set_interface (udev, interface->altsetting[0].desc.bInterfaceNumber, 2)) {
		return -ENODEV;
	}

	/* initialize cpad structure */
	cpad = kmalloc (sizeof(struct cpad_context), GFP_KERNEL);
	if (cpad == NULL) {
		err ("Out of memory");
		cpad_free_context(cpad);
		return -ENOMEM;
	}
	memset (cpad, 0x00, sizeof (struct cpad_context));

	init_MUTEX (&cpad->sem);
	init_waitqueue_head (&cpad->display.queue);
	cpad->udev = udev;
	cpad->interface = interface;

	res = cpad_setup_endpoints(cpad);
	if (res) {
		cpad_free_context(cpad);
		return res;
	}

	cpad->present = 1;
	usb_set_intfdata (interface, cpad);

	cpad_input_init(cpad);
	INIT_WORK (&cpad->touchpad.submit_urb, cpad_submit_int_urb, cpad);
	schedule_work (&cpad->touchpad.submit_urb);
	cpad_dev_init(cpad);
	cpad_procfs_add(cpad);
	cpad_fb_init(cpad);

	return 0;
}

/* prevent races between cpad_dev_open() and cpad_disconnect() */
static DECLARE_MUTEX (disconnect_sem);

static void cpad_disconnect(struct usb_interface *interface)
{
	struct cpad_context *cpad;

	/* prevent races with open() */
	down (&disconnect_sem);

	cpad = usb_get_intfdata (interface);
	usb_set_intfdata (interface, NULL);

	down (&cpad->sem);

	usb_unlink_urb(cpad->touchpad.in.urb);
	cpad_input_remove(cpad);
	cpad_dev_remove(cpad);
	cpad_procfs_del(cpad);
	cpad_fb_remove(cpad);

	/* unlink bulk urbs */
	if (atomic_read (&cpad->display.busy)) {
		usb_unlink_urb (cpad->display.out.urb);
		usb_unlink_urb (cpad->display.in.urb);
		wait_for_completion (&cpad->display.finished);
	}

	/* prevent device read, write and ioctl */
	cpad->present = 0;

	up (&cpad->sem);

	/* if the character device is opened, cpad_release will clean this up */
	if (!cpad->char_dev.open)
		cpad_free_context (cpad);

	up (&disconnect_sem);
}

static void cpad_display_out_callback (struct urb *urb, struct pt_regs *regs)
{
	struct cpad_context *cpad = (struct cpad_context *)urb->context;
	int res;

	if (!urb->status) {
		res = usb_submit_urb (cpad->display.in.urb, GFP_ATOMIC);
		cpad->display.submit_res = res;
		if (!res)
			return;
		err ("usb_submit_urb bulk in failed with result %d", res);
	}
	del_timer_sync(&cpad->display.timer);
	atomic_set (&cpad->display.busy, 0);
	complete (&cpad->display.finished);
	wake_up (&cpad->display.queue);
}

static void cpad_display_in_callback (struct urb *urb, struct pt_regs *regs)
{
	struct cpad_context *cpad = (struct cpad_context *)urb->context;
	del_timer_sync(&cpad->display.timer);
	atomic_set (&cpad->display.busy, 0);
	complete (&cpad->display.finished);
	wake_up (&cpad->display.queue);
}

/*
 * probe/disconnect helper functions
 */

static void cpad_free_context(struct cpad_context *cpad)
{
	if (!cpad)
		return;
	if (cpad->display.in.urb)
		usb_buffer_free (cpad->udev, cpad->display.in.buffer_size,
				 cpad->display.in.buffer,
				 cpad->display.in.urb->transfer_dma);
	usb_free_urb (cpad->display.in.urb);
	if (cpad->display.out.urb)
		usb_buffer_free (cpad->udev, cpad->display.out.buffer_size,
				 cpad->display.out.buffer,
				 cpad->display.out.urb->transfer_dma);
	usb_free_urb (cpad->display.out.urb);
	if (cpad->touchpad.in.urb)
		usb_buffer_free (cpad->udev, cpad->touchpad.in.buffer_size,
				 cpad->touchpad.in.buffer,
				 cpad->touchpad.in.urb->transfer_dma);
	usb_free_urb (cpad->touchpad.in.urb);
	kfree (cpad);
}

static int cpad_setup_bulk_endpoint(struct cpad_endpoint *bulk, struct cpad_context *cpad)
{
	int pipe;
	size_t buffer_size;
	usb_complete_t bulk_callback;

	if (bulk->endpoint_desc->bEndpointAddress & USB_DIR_IN) {
		pipe = usb_rcvbulkpipe (cpad->udev, bulk->endpoint_desc->bEndpointAddress);
		buffer_size = bulk->endpoint_desc->wMaxPacketSize;
		bulk_callback = cpad_display_in_callback;
	}
	else {
		pipe = usb_sndbulkpipe (cpad->udev, bulk->endpoint_desc->bEndpointAddress);
		buffer_size = 160*bulk->endpoint_desc->wMaxPacketSize;
		bulk_callback = cpad_display_out_callback;
	}

	bulk->buffer_size = buffer_size;
	bulk->urb->transfer_flags = (URB_NO_TRANSFER_DMA_MAP | URB_ASYNC_UNLINK);
	bulk->buffer = usb_buffer_alloc (cpad->udev, buffer_size, GFP_KERNEL,
					 &bulk->urb->transfer_dma);
	if (!bulk->buffer) {
		err("Couldn't allocate bulk buffer");
		return -ENOMEM;
	}
	usb_fill_bulk_urb (bulk->urb, cpad->udev, pipe, bulk->buffer, buffer_size,
			   bulk_callback, cpad);
	return 0;
}

static int cpad_setup_int_endpoint(struct cpad_endpoint *tp, struct cpad_context *cpad)
{
	size_t buffer_size;
	struct usb_endpoint_descriptor *endpoint_desc;
	int pipe, maxp;

	endpoint_desc = tp->endpoint_desc;
	pipe = usb_rcvintpipe (cpad->udev, endpoint_desc->bEndpointAddress);
	maxp = usb_maxpacket (cpad->udev, pipe, usb_pipeout(pipe));

	buffer_size = (maxp > 8 ? 8 : maxp);
	tp->buffer_size = buffer_size;
	tp->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	tp->buffer = usb_buffer_alloc (cpad->udev, 8, SLAB_ATOMIC,
				       &tp->urb->transfer_dma);
	if (!cpad->touchpad.in.buffer) {
		err("Couldn't allocate irq_in_buffer");
		return -ENOMEM;
	}
	usb_fill_int_urb (tp->urb, cpad->udev, pipe, tp->buffer, buffer_size,
			  cpad_input_callback, cpad, endpoint_desc->bInterval);
	return 0;
}

static int cpad_setup_endpoints(struct cpad_context *cpad)
{
	struct usb_host_interface *iface_desc = &cpad->interface->altsetting[2];
	struct usb_endpoint_descriptor *endpoint_desc;
	int i, j, res;

	struct cpad_endpoint_list {
		__u8 			dir;
		__u8 			xfer_type;
		struct cpad_endpoint *	endpoint;
		int (*setup) (struct cpad_endpoint *, struct cpad_context *);
	} endpoints [] = {
		{ USB_DIR_IN,  USB_ENDPOINT_XFER_BULK, &cpad->display.in,  cpad_setup_bulk_endpoint },
		{ USB_DIR_OUT, USB_ENDPOINT_XFER_BULK, &cpad->display.out, cpad_setup_bulk_endpoint },
		{ USB_DIR_IN,  USB_ENDPOINT_XFER_INT,  &cpad->touchpad.in, cpad_setup_int_endpoint  },
		{ 0, 0, 0, 0 }
	};

	for (i = 0; i < iface_desc->desc.bNumEndpoints; i++) {
		endpoint_desc = &iface_desc->endpoint[i].desc;
		for (j=0; endpoints[j].endpoint; j++)
			if (! endpoints[j].endpoint->endpoint_desc &&
			    ((endpoint_desc->bEndpointAddress & USB_DIR_IN) == endpoints[j].dir) &&
			    ((endpoint_desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
							== endpoints[j].xfer_type))
			{
				endpoints[j].endpoint->endpoint_desc = endpoint_desc;
				endpoints[j].endpoint->urb = usb_alloc_urb (0, GFP_KERNEL);
				if (!endpoints[j].endpoint->urb) {
					err("No free urbs available");
					return -ENOMEM;
				}
				res = endpoints[j].setup (endpoints[j].endpoint, cpad);
				if (res)
					return res;
			}
	}

	if (!(cpad->display.in.endpoint_desc && 
	      cpad->display.out.endpoint_desc &&
	      cpad->touchpad.in.endpoint_desc))
	{
		err("Could not find all cPad endpoints");
		return -ENODEV;
	}

	cpad->display.tmpbuf_size = cpad->display.out.endpoint_desc->wMaxPacketSize - 2;
	cpad->display.tmpbuf = kmalloc (cpad->display.tmpbuf_size, GFP_KERNEL);
	if (cpad->display.tmpbuf == NULL) {
		err("Out of memory");
		return -ENOMEM;
	}

	return 0;
}

static void cpad_submit_int_urb(void *arg)
/* data must always be fetched from the int endpoint, otherwise the cpad would
 * reconnect to force driver reload, so this is always scheduled by probe
 */
{
	struct cpad_context *cpad = (struct cpad_context *) arg;
	int retval;

	retval = usb_submit_urb (cpad->touchpad.in.urb, GFP_KERNEL);
	if (retval)
		err ("usb_submit_urb int in failed with result %d", retval);

	/* when doing rmmod cpad and then insmod cpad immediately,
	 * the first write urb to the bulk endpoints always times 
	 * out (bug?). The following line makes sure the first urb
	 * has no use. */
	cpad_nlcd_function(cpad, CPAD_R_LIGHT, 0);
}

/*
 * general helper functions
 */

static inline int cpad_check_display_urb_errors(struct cpad_context *cpad)
/* call with structure locked and after both display urbs finished */
{
	if (cpad->display.timed_out)
		return -ETIMEDOUT;
	if (cpad->display.out.urb->status)
		return cpad->display.out.urb->status;
	if (cpad->display.submit_res)
		return cpad->display.submit_res;
	return cpad->display.in.urb->status;
}

static int cpad_wait_interruptible(struct cpad_context *cpad)
/* wait for bulk urbs to finish. call with structure locked */
{
	atomic_t *busy = &cpad->display.busy;
	wait_queue_head_t *queue = &cpad->display.queue;

	while (atomic_read (busy)) {
		interruptible_sleep_on_timeout (queue, HZ/2);
		if (signal_pending (current)) {
			return -ERESTARTSYS;
		}
	}

	return 0;
}

static void cpad_timeout_kill(unsigned long data)
{
	struct cpad_context *cpad = (struct cpad_context *) data;
	usb_unlink_urb (cpad->display.out.urb);
	usb_unlink_urb (cpad->display.in.urb);
	cpad->display.timed_out = 1;
	err("cpad display urb timed out");
}

static void cpad_setup_timeout(struct cpad_context *cpad)
{
	struct timer_list *timer = &cpad->display.timer;
	init_timer(timer);
	timer->expires = jiffies + 2*HZ;
	timer->data = (unsigned long)cpad;
	timer->function = cpad_timeout_kill;
	add_timer(timer);
	cpad->display.timed_out = 0;
}

static int cpad_nlcd_function(struct cpad_context *cpad, const unsigned char func, const unsigned char val)
/* structure must be locked before calling this */
{
	unsigned char *tbuffer_pos = cpad->display.out.buffer;
	int res;

	if ((func < (char) 2) || (func > (char) 6)) {
		err("1335 nlcd invalid cmd");
		return 0;
	}

	if (!cpad->present)
		return -ENODEV;

	res = cpad_wait_interruptible (cpad);
	if (res)
		return res;

	*(tbuffer_pos++) = SEL_CPAD;
	*(tbuffer_pos++) = func;
	*(tbuffer_pos++) = val;
	cpad->display.out.urb->transfer_buffer_length = 3;

	cpad->display.in.buffer[2] = 0;
	res = usb_submit_urb(cpad->display.out.urb, GFP_KERNEL);
	if (res) {
		err ("failed submitting nlcd urb, error %d", res);
		return res;
	}
	init_completion (&cpad->display.finished);
	atomic_set (&cpad->display.busy, 1);
	cpad_setup_timeout (cpad);

	res = cpad_wait_interruptible (cpad);
	if (res)
		return res;
	res = cpad_check_display_urb_errors(cpad);
	if (res)
		return res;

	return cpad->display.in.buffer[2];
}

static void cpad_light_off(void *arg)
/* don't call this, only used by cpad_flash */
{
	struct cpad_context *cpad = (struct cpad_context *) arg;

	down (&cpad->sem);
	if (cpad_nlcd_function (cpad, CPAD_W_LIGHT, 0))
		err("error on timed_light_off");
	up (&cpad->sem);
}

static int cpad_flash(struct cpad_context *cpad, int time)
/* flash the display, time in 10ms. call with structure locked */
{
	struct work_struct *flash = &cpad->display.flash;
	int res;

	if (time <= 0)
		return -EINVAL;
	if (flash->pending != 0)
		return -EBUSY;
	res = cpad_nlcd_function(cpad, CPAD_W_LIGHT, 1);
	if (res)
		err("cpad flash (maybe) failed to turn on");
	time = min(time, CPAD_MAX_FLASH);
	INIT_WORK (flash, cpad_light_off, cpad);
	schedule_delayed_work (flash, (HZ * (unsigned long) time) / 100);
	return res;
}


/*****************************************************************************
 *	input routines, based on:					     *
 *		drivers/input/mouse/synaptics.c				     *
 *		drivers/usb/input/usbmouse.c				     *
 *****************************************************************************/

#ifdef CONFIG_USB_CPADINPUT

static int disable_input = 0;
MODULE_PARM(disable_input, "i");
MODULE_PARM_DESC(disable_input, "disable cPad input device");

#define XMIN_NOMINAL 1472
#define XMAX_NOMINAL 5472
#define YMIN_NOMINAL 1408
#define YMAX_NOMINAL 4448

static char cpad_input_name[] = "Synaptics cPad";

/* input prototypes */
static int cpad_input_open(struct input_dev *idev);
static void cpad_input_close(struct input_dev *idev);

static inline void cpad_input_init(struct cpad_context *cpad)
{
	struct input_dev *idev = &cpad->touchpad.idev;
	struct usb_device *udev = cpad->udev;
	char path[64];

	if (disable_input)
		return;

	/* setup abs */
	set_bit (EV_ABS, idev->evbit);
	set_bit (ABS_X, idev->absbit);
	set_bit (ABS_Y, idev->absbit);
	set_bit (ABS_PRESSURE, idev->absbit);
	set_bit (ABS_TOOL_WIDTH, idev->absbit);
	idev->absmin[ABS_X] = XMIN_NOMINAL;
	idev->absmax[ABS_X] = XMAX_NOMINAL;
	idev->absmin[ABS_Y] = YMIN_NOMINAL;
	idev->absmax[ABS_Y] = YMAX_NOMINAL;
	idev->absmax[ABS_PRESSURE] = 255;

	/* setup buttons */
	set_bit (EV_KEY, idev->evbit);
	set_bit (BTN_TOUCH, idev->keybit);
	set_bit (BTN_TOOL_FINGER, idev->keybit);
	set_bit (BTN_TOOL_DOUBLETAP, idev->keybit);
	set_bit (BTN_TOOL_TRIPLETAP, idev->keybit);
	set_bit (BTN_LEFT, idev->keybit);
	set_bit (BTN_RIGHT, idev->keybit);
	set_bit (BTN_MIDDLE, idev->keybit);

	/* setup device fops */
	idev->private = cpad;
	idev->open = cpad_input_open;
	idev->close = cpad_input_close;

	/* setup device information */
	usb_make_path (udev, path, 56);
	sprintf (cpad->touchpad.phys, "%s/input0", path);
	idev->phys = cpad->touchpad.phys;
	idev->name = cpad_input_name;
	idev->id.bustype = BUS_USB;
	idev->id.vendor = udev->descriptor.idVendor;
	idev->id.product = udev->descriptor.idProduct;
	idev->id.version = udev->descriptor.bcdDevice;
	idev->dev = &cpad->interface->dev;

	input_register_device(idev);
}

static inline void cpad_input_remove(struct cpad_context *cpad)
{
	if (!disable_input)
		input_unregister_device(&cpad->touchpad.idev);
}

static void cpad_input_callback(struct urb *urb, struct pt_regs *regs)
{
	struct cpad_context *cpad = urb->context;
	unsigned char *data = cpad->touchpad.in.buffer;
	struct input_dev *idev = &cpad->touchpad.idev;
	int retval;
	int w, pressure, finger_width, num_fingers;

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

	if (disable_input || !cpad->touchpad.open)
		goto resubmit;

	w = data[0] & 0x0f;
	pressure = data[6];
	if (pressure > 0) {
		finger_width = 5;
		num_fingers = 1;
		switch (w) {
		case 0:
			num_fingers = 2;
			break;
		case 1:
			num_fingers = 3;
			break;
		case 2:
			break;
		case 4 ... 15:
			finger_width = w;
			break;
		}
	}
	else {
		num_fingers = 0;
		finger_width = 0;
	}

	input_regs (idev, regs);

	if (pressure > 30) input_report_key (idev, BTN_TOUCH, 1);
	if (pressure < 25) input_report_key (idev, BTN_TOUCH, 0);

	if (pressure > 0) {
		input_report_abs (idev, ABS_X, (data[2] << 8) | data[3]);
		input_report_abs (idev, ABS_Y, YMIN_NOMINAL + YMAX_NOMINAL - ((data[4] << 8) | data[5]));
	}
	input_report_abs (idev, ABS_PRESSURE, pressure);

	input_report_abs (idev, ABS_TOOL_WIDTH, finger_width);
	input_report_key (idev, BTN_TOOL_FINGER, num_fingers == 1);
	input_report_key (idev, BTN_TOOL_DOUBLETAP, num_fingers == 2);
	input_report_key (idev, BTN_TOOL_TRIPLETAP, num_fingers == 3);
	input_report_key (idev, BTN_LEFT, data[1] & 0x04);
	input_report_key (idev, BTN_RIGHT, data[1] & 0x01);
	input_report_key (idev, BTN_MIDDLE, data[1] & 0x08);
	input_sync (idev);
resubmit:
	retval = usb_submit_urb (urb, GFP_ATOMIC);
	if (retval)
		err ("usb_submit_urb int in failed with result %d", retval);
}

static int cpad_input_open(struct input_dev *idev)
{
	struct cpad_context *cpad = idev->private;
	cpad->touchpad.open++;
	return 0;
}

static void cpad_input_close(struct input_dev *idev)
{
	struct cpad_context *cpad = idev->private;
	cpad->touchpad.open--;
}

#else /* CONFIG_USB_CPADINPUT */

static inline void cpad_input_init(struct cpad_context *cpad) { }
static inline void cpad_input_remove(struct cpad_context *cpad) { }

static void cpad_input_callback(struct urb *urb, struct pt_regs *regs)
/* data must always be fechted, otherwise cpad reconnects */
{
	int retval;

	switch (urb->status) {
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		return;
	}

	retval = usb_submit_urb(urb, GFP_ATOMIC);
	if (retval)
		err ("usb_submit_urb int in failed with result %d", retval);
}

#endif /* CONFIG_USB_CPADINPUT */


/*****************************************************************************
 *	cpad character device routines, based on:			     *
 *		drivers/usb/usb-skeleton.c				     *
 *****************************************************************************/

#ifdef CONFIG_USB_CPADDEV

static int disable_cdev = 0;
MODULE_PARM(disable_cdev, "i");
MODULE_PARM_DESC(disable_cdev, "disable cPad character device");

/* character device prototypes */
static int cpad_dev_open (struct inode *inode, struct file *file);
static int cpad_dev_release (struct inode *inode, struct file *file);
static ssize_t cpad_dev_read (struct file *file, char *buffer, size_t count, loff_t *ppos);
static ssize_t cpad_dev_write (struct file *file, const char *ubuffer, size_t count, loff_t *ppos);
static int cpad_dev_ioctl (struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);

static struct file_operations cpad_fops = {
	.owner =	THIS_MODULE,
	.read =		cpad_dev_read,
	.write =	cpad_dev_write,
	.ioctl =	cpad_dev_ioctl,
	.open =		cpad_dev_open,
	.release =	cpad_dev_release,
};

static struct usb_class_driver cpad_class = {
	.name =		"usb/cpad%d",
	.fops =		&cpad_fops,
	.mode =		S_IFCHR | S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP,
	.minor_base =	USB_CPAD_MINOR_BASE,
};

static inline void cpad_dev_init(struct cpad_context *cpad)
{
	struct usb_interface *interface = cpad->interface;

	if (disable_cdev)
		return;

	if (usb_register_dev(interface, &cpad_class)) {
		err ("Not able to get a minor for this device.");
		cpad->char_dev.disable = 1;
		return;
	}
	cpad->char_dev.minor = interface->minor;
	info ("USB Synaptics device now attached to USBcpad-%d", cpad->char_dev.minor);
}

static inline void cpad_dev_remove(struct cpad_context *cpad)
{
	struct usb_interface *interface = cpad->interface;

	if (disable_cdev || cpad->char_dev.disable)
		return;

	usb_deregister_dev (interface, &cpad_class);
	info("USB Synaptics cPad #%d now disconnected", cpad->char_dev.minor);
}

static int cpad_dev_open (struct inode *inode, struct file *file)
{
	struct cpad_context *cpad = NULL;
	struct usb_interface *interface;
	int minor;

	minor = iminor(inode);

	/* prevent disconnects */
	if (down_trylock (&disconnect_sem)) {
		if (file->f_flags & O_NONBLOCK)
			return -EBUSY;
		else if (down_interruptible (&disconnect_sem))
			return -EAGAIN;
	}

	interface = usb_find_interface (&cpad_driver, minor);
	if (!interface) {
		err ("error, can't find device for minor %d", minor);
		up (&disconnect_sem);
		return -ENODEV;
	}

	cpad = usb_get_intfdata(interface);
	if (!cpad) {
		up (&disconnect_sem);
		return -ENODEV;
	}

	if (down_trylock (&cpad->sem)) {
		if (file->f_flags & O_NONBLOCK) {
			up (&disconnect_sem);
			return -EBUSY;
		}
		else if (down_interruptible (&cpad->sem)) {
			up (&disconnect_sem);
			return -EAGAIN;
		}
	}
	++cpad->char_dev.open;
	file->private_data = cpad;
	up (&cpad->sem);

	up (&disconnect_sem);
	return 0;
}

static int cpad_dev_release (struct inode *inode, struct file *file)
{
	struct cpad_context *cpad = (struct cpad_context *)file->private_data;

	if (cpad == NULL)
		return -ENODEV;

	down (&cpad->sem);

	if (cpad->char_dev.open <= 0) {
		up (&cpad->sem);
		return -ENODEV;
	}

	if (atomic_read (&cpad->display.busy))
		wait_for_completion (&cpad->display.finished);

	--cpad->char_dev.open;

	if (!cpad->present && !cpad->char_dev.open) {
		up (&cpad->sem);
		cpad_free_context (cpad);
		return 0;
	}

	up (&cpad->sem);
	return 0;
}

static ssize_t cpad_dev_read (struct file *file, char *buffer, size_t count, loff_t *ppos)
{
	struct cpad_context *cpad = (struct cpad_context *)file->private_data;
	ssize_t bytes_read;
	int res;

	if (down_trylock (&cpad->sem)) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		else if (down_interruptible (&cpad->sem))
			return -ERESTARTSYS;
	}

	if (!cpad->present) {
		up (&cpad->sem);
		return -ENODEV;
	}

	if (atomic_read (&cpad->display.busy)) {
		if (file->f_flags & O_NONBLOCK) {
			up (&cpad->sem);
			return -EAGAIN;
		}
		res = cpad_wait_interruptible (cpad);
		if (res) {
			up (&cpad->sem);
			return res;
		}
	}

	res = cpad_check_display_urb_errors(cpad);
	if (res) {
		up (&cpad->sem);
		return res;
	}

	bytes_read = min ((int)count, cpad->display.in.urb->actual_length);
	if (copy_to_user (buffer, cpad->display.in.buffer, bytes_read)) {
		up (&cpad->sem);
		return -EFAULT;
	}

	up (&cpad->sem);
	return bytes_read;
}

static ssize_t cpad_dev_write (struct file *file, const char *ubuffer, size_t count, loff_t *ppos)
{
	struct cpad_context *cpad = (struct cpad_context *)file->private_data;
	struct cpad_endpoint *out = &cpad->display.out;
	unsigned char *tmpbuf = cpad->display.tmpbuf;
	unsigned char *tmpbuf_pos;
	unsigned char *tbuffer_pos = out->buffer;
	const char *ubuffer_pos = ubuffer;
	size_t ulength, uremaining, actual_length;
	unsigned char cmd;
	int res;

	if (down_trylock (&cpad->sem)) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		else if (down_interruptible (&cpad->sem))
			return -ERESTARTSYS;
	}

	if (!cpad->present) {
		up (&cpad->sem);
		return -ENODEV;
	}

	if (count == 0) {
		up (&cpad->sem);
		return 0;
	}

	if (atomic_read (&cpad->display.busy)) {
		if (file->f_flags & O_NONBLOCK) {
			up (&cpad->sem);
			return -EAGAIN;
		}
		res = cpad_wait_interruptible (cpad);
		if (res) {
			up (&cpad->sem);
			return res;
		}
	}

	cpad_fb_deactivate (cpad);

	/* get 1335 command first */
	get_user(cmd, ubuffer);
	ubuffer_pos++;
	if (count == 1) {
		/* no params */
		*(tbuffer_pos++) = SEL_1335;
		*(tbuffer_pos++) = cmd;
		actual_length = 2;
	}
	else {
		actual_length = 0;
		uremaining = count - 1;
		while (uremaining > 0) {
			ulength = min (cpad->display.tmpbuf_size, uremaining);
			if (actual_length+ulength > out->buffer_size)
				break;

			/* copy the data from userspace into our buffer */
			if (copy_from_user(tmpbuf, ubuffer_pos, ulength)) {
				up (&cpad->sem);
				return -EFAULT;
			}
			ubuffer_pos += ulength;
			uremaining -= ulength;

			/* here we set the 1335 select, 1335 command, and reverse the params */
			*(tbuffer_pos++) = SEL_1335;
			*(tbuffer_pos++) = cmd;
			for (tmpbuf_pos=tmpbuf+ulength-1; tmpbuf_pos>=tmpbuf; tmpbuf_pos--)
				*(tbuffer_pos++) = *tmpbuf_pos;
			actual_length += ulength + 2;
		}
	}

	/* this urb was already set up, except for this write size */
	out->urb->transfer_buffer_length = actual_length;

	res = usb_submit_urb(out->urb, GFP_KERNEL);
	cpad->display.submit_res = res;
	if (res) {
		err ("failed submitting write urb, error %d", res);
	} else {
		init_completion (&cpad->display.finished);
		atomic_set (&cpad->display.busy, 1);
		cpad_setup_timeout (cpad);
		res = ubuffer_pos - ubuffer;
	}

	up (&cpad->sem);
	return res;
}

int cpad_driver_num = CPAD_DRIVER_NUM;

static int cpad_dev_ioctl (struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct cpad_context *cpad = (struct cpad_context *)file->private_data;
	unsigned char cval = 0;
	int ival = 0;
	void *rval = NULL;
	int res = 0;

	if (down_interruptible (&cpad->sem))
		return -ERESTARTSYS;

	if (!cpad->present) {
		up (&cpad->sem);
		return -ENODEV;
	}

	/* read data from user */
	if (cmd & IOC_IN) {
		switch (_IOC_SIZE(cmd)) {
		case sizeof(char):
			get_user(cval, (char *) arg);
			break;
		case sizeof(int):
			get_user(ival, (int *) arg);
			break;
		default:
			up (&cpad->sem);
			return -ENOIOCTLCMD;
		}
	}

	switch (cmd) {
	case CPAD_VERSION:
		rval = &cpad_driver_num;
		break;

	case CPAD_CGID:
		rval = &cpad->touchpad.idev.id;
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

	/* not yet supported */
	case CPAD_WIMAGEL:
	/* no longer supported, only for compatibility */
	case CPAD_SET_SENS:
	case CPAD_SET_STROKE:
	case CPAD_SET_ABS:
		break;

	default:
		up (&cpad->sem);
		return -ENOIOCTLCMD;
	}
	up (&cpad->sem);

	if (res < 0)
		return res;

	/* write data to user */
	if ((cmd & IOC_OUT) && (rval != NULL))
		if (copy_to_user((void *) arg, rval, _IOC_SIZE(cmd)))
			return -EFAULT;

	return 0;
}

#else /* CONFIG_USB_CPADDEV */

static inline void cpad_dev_init(struct cpad_context *cpad) { }
static inline void cpad_dev_remove(struct cpad_context *cpad) { }

#endif /* CONFIG_USB_CPADDEV */


/*****************************************************************************
 *	procfs routines							     *
 *****************************************************************************/

#ifdef CONFIG_USB_CPADPROCFS

static int disable_procfs = 0;
MODULE_PARM(disable_procfs, "i");
MODULE_PARM_DESC(disable_procfs, "disable cPad procfs interface");

static struct proc_dir_entry *cpad_procfs_root;
static atomic_t cpad_procfs_count;

/* procfs prototypes */
static read_proc_t cpad_procfs_read;
static write_proc_t cpad_procfs_write;

static inline void cpad_procfs_init(void)
{
	if (disable_procfs)
		return;

	cpad_procfs_root = proc_mkdir("driver/cpad", NULL);
	if (! cpad_procfs_root)
	{
		disable_procfs = 1;
		err("can't create /proc/driver/cpad\n");
		return;
	}
	cpad_procfs_root->owner = THIS_MODULE;
	atomic_set(&cpad_procfs_count, 0);
}

static inline void cpad_procfs_remove(void)
{
	if (disable_procfs)
		return;
	remove_proc_entry("driver/cpad", NULL);
}

static inline void cpad_procfs_add(struct cpad_context *cpad)
{
	struct cpad_procfs *procfs = &cpad->procfs;

	if (disable_procfs)
		return;

	procfs->num = atomic_inc_and_test(&cpad_procfs_count);
	sprintf(procfs->name, "%i", procfs->num);
	procfs->entry = create_proc_read_entry(procfs->name,
				S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP,
				cpad_procfs_root, cpad_procfs_read, cpad);
	if (! procfs->entry)
	{
		err("can't create /proc/driver/cpad/%s\n", procfs->name);
		return;
	}
	procfs->entry->owner = THIS_MODULE;
	procfs->entry->write_proc = cpad_procfs_write;
}

static inline void cpad_procfs_del(struct cpad_context *cpad)
{
	if (disable_procfs)
		return;
	if (cpad->procfs.entry)
		remove_proc_entry(cpad->procfs.name, cpad_procfs_root);
}

static int cpad_procfs_read(char *page, char **start, off_t off, 
			    int count, int *eof, void *data)
{
	struct cpad_context *cpad = (struct cpad_context *) data;
	char *page_pos = page;

	if (down_interruptible (&cpad->sem))
		return -ERESTARTSYS;

	page_pos += sprintf(page_pos, "minor:      %i\n", (int) cpad->char_dev.minor);
	page_pos += sprintf(page_pos, "lcd:        %i\n",
			    cpad_nlcd_function(cpad, CPAD_R_LCD, 0));
	page_pos += sprintf(page_pos, "backlight:  %i\n",
			    cpad_nlcd_function(cpad, CPAD_R_LIGHT, 0));
	page_pos += sprintf(page_pos, "framerate:  %i\n", cpad->fb.rate);
	page_pos += sprintf(page_pos, "dithering:  %i\n", cpad->fb.dither);
	page_pos += sprintf(page_pos, "brightness: %i\n", cpad->fb.brightness);
	page_pos += sprintf(page_pos, "invert:     %i\n", cpad->fb.invert);

	up(&cpad->sem);

	*eof = 1;
	return page_pos - page;
}

static int cpad_procfs_write(struct file *file, const char *buffer,
			     unsigned long count, void *data)
{
	struct cpad_context *cpad = (struct cpad_context *) data;
	char *page = (char*) __get_free_page(GFP_KERNEL);
	char *page_pos = page;
	int remain = count;
	int value = 0;

	if (!page)
		return -ENOMEM;
	count = min(count, PAGE_SIZE-1);
	if (copy_from_user(page, buffer, count))
		return -EFAULT;
	page[count] = '\0';

	if (down_interruptible (&cpad->sem))
		return -ERESTARTSYS;

	while (remain) {
		if (sscanf(page_pos, " flash : %i", &value) == 1)
			cpad_flash(cpad, value);
		else if (sscanf(page_pos, " framerate : %i", &value) == 1) {
			if (value <= 0)
				cpad_fb_deactivate (cpad);
			else
				cpad_fb_activate (cpad, value);
		}
		else if (sscanf(page_pos, " dithering : %i", &value) == 1) {
			value = max(value, 0);
			cpad->fb.dither = min(value, 3);
		}
		else if (sscanf(page_pos, " brightness : %i", &value) == 1) {
			value = max(value, 0);
			cpad->fb.brightness = min(value, 10000);
		}
		else if (sscanf(page_pos, " invert : %i", &value) == 1)
			cpad->fb.invert = (value <= 0) ? 0 : 1;

		do {
			++page_pos;
			--remain;
		}
		while (remain && *(page_pos-1) != '\n' && *(page_pos-1) != ';');
	}

	up(&cpad->sem);

	free_page((unsigned long) page);
	return count;
}

#else /* CONFIG_USB_CPADPROCFS */

static inline void cpad_procfs_init(void) { }
static inline void cpad_procfs_remove(void) { }
static inline void cpad_procfs_add(strstatic struct fb_infouct cpad_context *cpad) { }
static inline void cpad_procfs_del(struct cpad_context *cpad) { }

#endif /* CONFIG_USB_CPADPROCFS */


/*****************************************************************************
 *	framebuffer routines, based on:					     *
 *		drivers/video/vfb.c					     *
 *		drivers/usb/media/vicam.c	(mmap functions)	     *
 *****************************************************************************/

#ifdef CONFIG_USB_CPADFB

/*
 * TODO:
 *	support for FB_VISUAL_MONO01
 *	only send changed pixels (probably with pte_young and pte_mkyoung)
 *	include fb_check_var, fb_set_par, fb_pan_display functions in cpad_fb_ops
 *	support different bpp
 */

static int disable_fb = 0;
MODULE_PARM(disable_fb, "i");
MODULE_PARM_DESC(disable_fb, "disable cPad framebuffer device");

static struct fb_var_screeninfo cpad_fb_default_var = {
	.xres =		240,
	.yres =		160,
	.xres_virtual =	240,
	.yres_virtual =	160,
	.bits_per_pixel = 24,
	.red =		{ 0, 8, 0 },
      	.green =	{ 8, 8, 0 },
      	.blue =		{ 16, 8, 0 },
      	.activate =	FB_ACTIVATE_TEST,
      	.height =	38,
      	.width =	46,
      	.pixclock =	200000,
      	.left_margin =	16,
      	.right_margin =	16,
      	.upper_margin =	8,
      	.lower_margin =	8,
      	.hsync_len =	16,
      	.vsync_len =	2,
      	.vmode =	FB_VMODE_NONINTERLACED,
};

static struct fb_fix_screeninfo cpad_fb_default_fix = {
	.id =		"cPad FB",
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	FB_VISUAL_TRUECOLOR,
	.accel =	FB_ACCEL_NONE,
	.line_length =	240*3,
	.smem_len = 	155648,
};

static int cpad_fb_dither2[2][2] = 	{{1, 3},
					 {4, 2}};

static int cpad_fb_dither4[4][4] = 	{{ 1,  9,  3, 11},
					 {13,  5, 15,  7},
					 { 4, 12,  2, 10},
					 {16,  8, 14,  6}};

static int cpad_fb_dither8[8][8] = 	{{ 1, 33,  9, 41,  3, 35, 11, 43},
					 {49, 17, 57, 25, 51, 19, 59, 27},
					 {13, 45,  5, 37, 15, 47,  7, 39},
					 {61, 29, 53, 21, 63, 31, 55, 23},
					 { 4, 36, 12, 44,  2, 34, 10, 42},
					 {52, 20, 60, 28, 50, 18, 58, 26},
					 {16, 48,  8, 40, 14, 46,  6, 38},
					 {64, 32, 56, 24, 62, 30, 54, 22}};

/* framebuffer device prototypes */
static int cpad_fb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			     u_int transp, struct fb_info *info);
static int cpad_fb_mmap(struct fb_info *info, struct file *file,
		    struct vm_area_struct *vma);
static void cpad_fb_drawimage(void *arg);

static struct fb_ops cpad_fb_ops = {
	.owner		= THIS_MODULE,
	.fb_setcolreg	= cpad_fb_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_cursor	= soft_cursor,
	.fb_mmap	= cpad_fb_mmap,
};

static inline void cpad_fb_init(struct cpad_context *cpad)
{
	struct fb_info *info = &cpad->fb.info;
	u_long vsize, size;
	void *adr;

	if (disable_fb)
		return;

	vsize = cpad_fb_default_fix.smem_len;
	if (!(cpad->fb.videomemory = vmalloc(vsize))) {
		cpad->fb.disable = 1;
		err("not enough memory for framebuffer");
		return;
	}

	memset(cpad->fb.videomemory, 0, vsize);
	adr = cpad->fb.videomemory;
	size = vsize;
	while (size > 0) {
		SetPageReserved(vmalloc_to_page(adr));
		adr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	info->screen_base = cpad->fb.videomemory;
	info->fbops = &cpad_fb_ops;
	info->var = cpad_fb_default_var;
	info->fix = cpad_fb_default_fix;
	info->pseudo_palette = &cpad->fb.pseudo_palette;
	info->flags = FBINFO_FLAG_DEFAULT;
	fb_alloc_cmap(&cpad->fb.info.cmap, 16, 0);
	info->par = cpad;

	if (register_framebuffer(&cpad->fb.info) < 0) {
		vfree(cpad->fb.videomemory);
		cpad->fb.disable = 1;
		err("couldn't register framebuffer");
		return;
	}

	cpad->fb.brightness = 200;
	cpad->fb.dither = 3;
}

static inline void cpad_fb_remove(struct cpad_context *cpad)
{
	unsigned long adr, size;

	if (disable_fb || cpad->fb.disable)
		return;

	cpad_fb_deactivate (cpad);

	unregister_framebuffer(&cpad->fb.info);
	adr = (unsigned long) cpad->fb.videomemory;
	size = cpad->fb.info.fix.smem_len;
	while ((long) size > 0) {
		ClearPageReserved(vmalloc_to_page((void *)adr));
		adr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
	vfree(cpad->fb.videomemory);
}

static void cpad_fb_activate(struct cpad_context *cpad, int rate)
{
	if (disable_fb || cpad->fb.disable)
		return;

	cpad->fb.rate = min(rate, 60);

	if (cpad->fb.active)
		return;

	cpad->fb.active = 1;
	init_completion (&cpad->fb.draw_finished);
	INIT_WORK (&cpad->fb.drawimage, cpad_fb_drawimage, cpad);
	schedule_work (&cpad->fb.drawimage);
}

static void cpad_fb_deactivate(struct cpad_context *cpad)
{
	if (cpad->fb.active) {
		cpad->fb.active = 0;
		wait_for_completion(&cpad->fb.draw_finished);
		cpad->fb.rate = 0;
	}
}

static int cpad_fb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			     u_int transp, struct fb_info *info)
{
	if (regno >= 16)
		return 1;

	red   >>= 8;
	green >>= 8;
	blue  >>= 8;
	((u32 *)(info->pseudo_palette))[regno] =
		(red   << info->var.red.offset)   |
		(green << info->var.green.offset) |
		(blue  << info->var.blue.offset);
	return 0;
}

static int cpad_fb_mmap(struct fb_info *info, struct file *file,
			struct vm_area_struct *vma)
{
	unsigned long page, kva, pos;
	unsigned long start = vma->vm_start;
	unsigned long size  = vma->vm_end-vma->vm_start;

	if (!info)
		return -ENODEV;

	if (size > info->fix.smem_len)
		return -EINVAL;

	pos = (unsigned long) info->screen_base;
	while (size > 0) {
		kva = (unsigned long) page_address(vmalloc_to_page((void *)pos));
		kva |= pos & (PAGE_SIZE-1);
		page = __pa(kva);
		if (remap_page_range(vma, start, page, PAGE_SIZE, PAGE_SHARED))
			return -EAGAIN;

		start += PAGE_SIZE;
		pos += PAGE_SIZE;
		if (size > PAGE_SIZE)
			size -= PAGE_SIZE;
		else
			size = 0;
	}

	return 0;
}

static void cpad_fb_drawimage(void *arg) {
	struct cpad_context *cpad = (struct cpad_context *) arg;
	struct cpad_endpoint *out = &cpad->display.out;
	unsigned char *tmpbuf = cpad->display.tmpbuf;
	unsigned char *tmpbuf_pos;
	unsigned char *tbuffer_pos = out->buffer;
	unsigned char *vbuffer_pos = cpad->fb.videomemory;
	size_t llength, vremaining, actual_length;
	int res, i, j, grey, maxgrey, black, x, y;

	if (!cpad->fb.active) {
		complete(&cpad->fb.draw_finished);
		return;
	}

	if (down_trylock (&cpad->sem)) {
		INIT_WORK (&cpad->fb.drawimage, cpad_fb_drawimage, cpad);
		schedule_delayed_work (&cpad->fb.drawimage, HZ/60);
		return;
	}

	if (atomic_read (&cpad->display.busy))
		wait_for_completion (&cpad->display.finished);

	*(tbuffer_pos++) = SEL_1335;
	*(tbuffer_pos++) = CSRW_1335;
	*(tbuffer_pos++) = 0;
	*(tbuffer_pos++) = 0;
	out->urb->transfer_buffer_length = 4;
	res = usb_submit_urb(out->urb, GFP_KERNEL);
	cpad->display.submit_res = res;
	if (res) {
		err("usb_submit_urb failed, error %d. deactivating framebuffer", res);
		cpad->fb.active = 0;
		cpad->fb.rate = 0;
		up (&cpad->sem);
		return;
	}
	init_completion (&cpad->display.finished);
	atomic_set (&cpad->display.busy, 1);
	cpad_setup_timeout (cpad);
	wait_for_completion (&cpad->display.finished);
	res = cpad_check_display_urb_errors(cpad);
	if (res) {
		err("sending urb failed, error %d. deactivating framebuffer", res);
		cpad->fb.active = 0;
		cpad->fb.rate = 0;
		up (&cpad->sem);
		return;
	}

	tbuffer_pos = out->buffer;
	actual_length = 0;
	vremaining = 160*30;
	maxgrey = 255*10;
	black = 0;
	y = 0;
	while (vremaining > 0) {
		llength = min (cpad->display.tmpbuf_size, vremaining);
		if (actual_length+llength > out->buffer_size)
			break;

		/* convert line */
		x = 0;
		for (i=0; i<30; i++)
			for (j=7; j>=0; j--) {
				grey = 3*vbuffer_pos[0] + 6*vbuffer_pos[1] + vbuffer_pos[2];
				grey = min((grey*cpad->fb.brightness)/100, maxgrey);

				switch (cpad->fb.dither) {
				case 0:
					black = (2*grey > maxgrey) ? 0 : 1;
					break;
				case 1:
					black = (5*grey > cpad_fb_dither2[x%2][y%2]*maxgrey) ? 0 : 1;
					break;
				case 2:
					black = (17*grey > cpad_fb_dither4[x%4][y%4]*maxgrey) ? 0 : 1;
					break;
				case 3:
					black = (65*grey > cpad_fb_dither8[x%8][y%8]*maxgrey) ? 0 : 1;
				}

				if (cpad->fb.invert)
					black = !black;

				if (black) {
					tmpbuf[i] |= 1 << j;
				}
				else {
					tmpbuf[i] &= ~(1 << j);
				}
				vbuffer_pos += 3;
				x++;
			}
		vremaining -= llength;
		y ++;

		/* here we set the 1335 select, 1335 command, and reverse the params */
		*(tbuffer_pos++) = SEL_1335;
		*(tbuffer_pos++) = MWRITE_1335;
		for (tmpbuf_pos=tmpbuf+llength-1; tmpbuf_pos>=tmpbuf; tmpbuf_pos--)
			*(tbuffer_pos++) = *tmpbuf_pos;
		actual_length += llength + 2;
	}

	/* this urb was already set up, except for this write size */
	out->urb->transfer_buffer_length = actual_length;

	res = usb_submit_urb(out->urb, GFP_KERNEL);
	cpad->display.submit_res = res;
	if (res) {
		err("usb_submit_urb failed, error %d. deactivating framebuffer", res);
		cpad->fb.active = 0;
		cpad->fb.rate = 0;
		up (&cpad->sem);
		return;
	}
	init_completion (&cpad->display.finished);
	atomic_set (&cpad->display.busy, 1);
	cpad_setup_timeout (cpad);
	wait_for_completion (&cpad->display.finished);
	res = cpad_check_display_urb_errors(cpad);
	if (res) {
		err("sending urb failed, error %d. deactivating framebuffer", res);
		cpad->fb.active = 0;
		cpad->fb.rate = 0;
		up (&cpad->sem);
		return;
	}

	INIT_WORK (&cpad->fb.drawimage, cpad_fb_drawimage, cpad);
	schedule_delayed_work (&cpad->fb.drawimage, HZ/cpad->fb.rate);

	up (&cpad->sem);
}

#else /* CONFIG_USB_CPADFB */

static inline void cpad_fb_init(struct cpad_context *cpad) { }
static inline void cpad_fb_remove(struct cpad_context *cpad) { }
static void cpad_fb_activate(struct cpad_context *cpad, int rate) { }
static void cpad_fb_deactivate(struct cpad_context *cpad) { }

#endif /* CONFIG_USB_CPADFB */
