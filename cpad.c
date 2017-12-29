/****
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

/****
 * the cPad is an USB touchpad with background display (240x160 mono)
 * it has one interface with three possible alternate settings
 * 	setting 0: one int endpoint for rel movement (used by usbhid.ko)
 *	setting 1: one int endpoint for abs finger position
 *	setting 2: one int endpoint for abs finger position and
 *		   two bulk endpoints for the display (in/out)
 * this driver uses setting 2
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
 * accessed via ioctls or procfs.
 *
 * observed functions are: */
#define CPAD_W_ROM	0x01	/* write EEPROM (not supported) */
#define CPAD_R_ROM	0x02	/* read EEPROM (not supported via ioctl) */
#define CPAD_W_LIGHT	0x03	/* write backlight state (on/off) */
#define CPAD_R_LIGHT	0x04	/* read backlight state (on/off) */
#define CPAD_W_LCD	0x05	/* write lcd state (on/off) */
#define CPAD_R_LCD	0x06	/* read lcd state (on/off) */
#define CPAD_RSRVD	0x07

/* possible values for the first byte of a packet */
#define SEL_CPAD	0x01	/* cPad not-lcd-controller select */
#define SEL_1335	0x02	/* lcd controller select */

/* an urb to the bulk out endpoint must be followed by an urb to the bulk in
 * endpoint. this gives the answer of the cpad/sed1335. */


#include <linux/usb.h>
#include <linux/input.h>
#include <linux/proc_fs.h>
#include <linux/fb.h>
#include <asm/uaccess.h>
#include <linux/mm.h>
#include <linux/version.h>

#include "cpadconfig.h"
#include "cpad.h"

#define DRIVER_VERSION "v0.4"
#define DRIVER_AUTHOR	"Rob Miller (rob@inpharmatica . co . uk), "\
			"Ron Lee (ron@debian.org), "\
			"Jan Steinhoff <jan.steinhoff@uni-jena.de>"
#define DRIVER_DESC "USB Synaptics cPad Driver"


/*****************************************************************************
 *	data types							     *
 *****************************************************************************/

struct cpad_endpoint {
	struct urb *		urb;
	struct usb_endpoint_descriptor * endpoint_desc;
	unsigned char *		buffer;
	size_t			buffer_size;
};

/* Locking:
 *	sem locks everything in cpad_context, exept open_count, procfs.open and
 *	fb.open. open_count_sem locks open_count, procfs.open, fb.open and
 *	present.
 */
struct cpad_context {
	struct usb_device *	udev;
	struct usb_interface *	interface;
	struct semaphore	sem;

	int			present;
	int			open_count;
	struct semaphore	open_count_sem;

	int			table_index;

	/* display data */
	struct cpad_display {
		struct cpad_endpoint	in;
		struct cpad_endpoint	out;

		atomic_t		busy;
		struct completion	finished;
		wait_queue_head_t	queue;
		struct timer_list	timer;
		int			timed_out;
		int			submit_res;

		unsigned char *		tmpbuf;
		size_t			tmpbuf_size;
		struct work_struct	flash;
	} display;

	/* touchpad data */
	struct cpad_touchpad {
		struct cpad_endpoint	in;
		struct input_dev	idev;
		int			evdev_num;
		char			phys[64];
		struct work_struct	submit_urb;
	} touchpad;

	/* cpad character device data */
	struct cpad_char_dev {
		int			disable;
		unsigned char		minor;
	} char_dev;

	/* procfs data */
	struct cpad_procfs {
		int			disable;
		int			open;
		struct proc_dir_entry *	entry;
		char			name[8];
	} procfs;

	/* frame buffer data */
	struct cpad_fb {
		int			disable;
		int			node;
		int			open;
		void *			videomemory;
		unsigned char *		buffer;
		u32 			pseudo_palette[17];
		struct work_struct	sendimage;
		struct completion	finished;
		struct fb_info		info;
		int			active;
		int			rate;
		int			dither;
		int 			brightness;
		int			invert;
		int 			onlychanged;
		int			onlyvisible;
	} fb;
};


/*****************************************************************************
 *	declarations and prototypes					     *
 *****************************************************************************/

/* usb */
static struct usb_driver cpad_driver;

static int cpad_probe(struct usb_interface *interface,
				const struct usb_device_id *id);
static void cpad_disconnect(struct usb_interface *interface);
static void cpad_display_out_callback(struct urb *urb, struct pt_regs *regs);
static void cpad_display_in_callback (struct urb *urb, struct pt_regs *regs);

static void cpad_free_context(struct cpad_context *cpad);
static int cpad_setup_endpoints(struct cpad_context *cpad);

static void cpad_timeout_kill(unsigned long data);
static void cpad_light_off(void *arg);

/* input */
static void cpad_input_add(struct cpad_context *cpad);
static void cpad_input_remove(struct cpad_context *cpad);
static void cpad_input_callback(struct urb *urb, struct pt_regs *regs);
static void cpad_submit_int_urb(void *arg);

/* character device */
static void cpad_dev_add(struct cpad_context *cpad);
static void cpad_dev_remove(struct cpad_context *cpad);

/* procfs */
static void cpad_procfs_init(void);
static void cpad_procfs_exit(void);
static void cpad_procfs_add(struct cpad_context *cpad);
static void cpad_procfs_remove(struct cpad_context *cpad);

/* frame buffer */
static void cpad_fb_add(struct cpad_context *cpad);
static void cpad_fb_remove(struct cpad_context *cpad);
static void cpad_fb_free (struct cpad_context *cpad);
static void cpad_fb_activate(struct cpad_context *cpad, int rate);
static void cpad_fb_deactivate(struct cpad_context *cpad);
static int cpad_fb_oneframe(struct cpad_context *cpad);
static int disable_fb;


/*****************************************************************************
 *	module initialisation						     *
 *****************************************************************************/

static int __init cpad_init(void)
{
	int res;

	cpad_procfs_init();
	res = usb_register(&cpad_driver);
	if (res) {
		err("usb_register failed. Error number %d", res);
		cpad_procfs_exit();
		return res;
	}
	info(DRIVER_DESC " " DRIVER_VERSION);
	return 0;
}

static void __exit cpad_exit(void)
{
	usb_deregister(&cpad_driver);
	cpad_procfs_exit();
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

static struct usb_device_id cpad_idtable [] = {
	{ USB_DEVICE(USB_VENDOR_ID_SYNAPTICS, USB_DEVICE_ID_CPAD) },
	{ 0, 0 }
};
MODULE_DEVICE_TABLE (usb, cpad_idtable);

static struct usb_driver cpad_driver = {
	.owner =	THIS_MODULE,
	.name =		"cpad",
	.probe =	cpad_probe,
	.disconnect =	cpad_disconnect,
	.id_table =	cpad_idtable,
};

#define MAX_DEVICES	16
static struct cpad_context *cpad_table[MAX_DEVICES] = { 0 };
static int cpad_table_index = MAX_DEVICES - 1;
static DECLARE_MUTEX (cpad_table_sem);

static int cpad_probe(struct usb_interface *interface,
				const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(interface);
	struct cpad_context *cpad;
	int retval, i;

	/* activate alternate setting 2 */
	if (usb_set_interface (udev,
			interface->altsetting[0].desc.bInterfaceNumber, 2))
		return -ENODEV;

	/* initialize cpad data structure */
	cpad = kmalloc (sizeof(struct cpad_context), GFP_KERNEL);
	if (cpad == NULL) {
		err ("Out of memory");
		return -ENOMEM;
	}
	memset (cpad, 0x00, sizeof (struct cpad_context));

	init_MUTEX (&cpad->sem);
	init_MUTEX (&cpad->open_count_sem);
	init_waitqueue_head (&cpad->display.queue);
	INIT_WORK (&cpad->display.flash, cpad_light_off, cpad);

	init_timer(&cpad->display.timer);
	cpad->display.timer.data = (unsigned long)cpad;
	cpad->display.timer.function = cpad_timeout_kill;

	cpad->udev = udev;
	cpad->interface = interface;

	retval = cpad_setup_endpoints(cpad);
	if (retval)
		goto error;

	cpad->display.tmpbuf_size =
			cpad->display.out.endpoint_desc->wMaxPacketSize - 2;
	cpad->display.tmpbuf = kmalloc (cpad->display.tmpbuf_size, GFP_KERNEL);
	cpad->fb.buffer = kmalloc (274*32, GFP_KERNEL);
	if ((cpad->display.tmpbuf == NULL) || (cpad->fb.buffer == NULL)) {
		err("Out of memory");
		retval = -ENOMEM;
		goto error;
	}

	cpad->present = 1;
	cpad->table_index = -1;
	down(&cpad_table_sem);
	for (i=0; i<MAX_DEVICES; i++) {
		if (++cpad_table_index == MAX_DEVICES)
			cpad_table_index = 0;
		if (cpad_table[cpad_table_index] == 0) {
			cpad->table_index = cpad_table_index;
			cpad_table[cpad_table_index] = cpad;
			break;
		}
	}
	down(&cpad->sem);
	up(&cpad_table_sem);
	usb_set_intfdata (interface, cpad);

	/* initialize input, chardev, fb and procfs */
	cpad_input_add(cpad);
	cpad_dev_add(cpad);
	cpad_procfs_add(cpad);
	cpad_fb_add(cpad);
	up(&cpad->sem);

	return 0;
error:
	cpad_free_context(cpad);
	return retval;
}

/* prevent races between ...open() and cpad_disconnect() */
static DECLARE_MUTEX (disconnect_sem);

static void cpad_disconnect(struct usb_interface *interface)
{
	struct cpad_context *cpad;

	/* prevent races with ...open() */
	down (&disconnect_sem);

	cpad = usb_get_intfdata (interface);
	usb_set_intfdata (interface, NULL);

	down (&cpad->sem);

	cpad_input_remove(cpad);
	cpad_dev_remove(cpad);
	cpad_fb_deactivate (cpad);

	down (&cpad->open_count_sem);

	/* prevent device read, write, ioctl and creation of new works */
	cpad->present = 0;
	if (cpad->table_index >= 0)
		cpad_table[cpad->table_index] = 0;
	up (&cpad->open_count_sem);

	cancel_delayed_work (&cpad->touchpad.submit_urb);
	cancel_delayed_work (&cpad->display.flash);
	up (&cpad->sem);
	flush_scheduled_work ();
	down (&cpad->sem);

	del_timer_sync(&cpad->display.timer);

	/* unlink urbs */
	usb_unlink_urb(cpad->touchpad.in.urb);
	if (atomic_read (&cpad->display.busy)) {
		usb_unlink_urb (cpad->display.out.urb);
		usb_unlink_urb (cpad->display.in.urb);
		wait_for_completion (&cpad->display.finished);
	}

	up (&cpad->sem);

	/* if a file is opened, this is done in a release function */
	if (!cpad->procfs.open)
		cpad_procfs_remove (cpad);
	if (!cpad->fb.open)
		cpad_fb_remove (cpad);
	if (!cpad->open_count)
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
	del_timer (&cpad->display.timer);
	atomic_set (&cpad->display.busy, 0);
	complete (&cpad->display.finished);
	wake_up_interruptible (&cpad->display.queue);
}

static void cpad_display_in_callback (struct urb *urb, struct pt_regs *regs)
{
	struct cpad_context *cpad = (struct cpad_context *)urb->context;

	del_timer (&cpad->display.timer);
	atomic_set (&cpad->display.busy, 0);
	complete (&cpad->display.finished);
	wake_up_interruptible (&cpad->display.queue);
}

static void cpad_timeout_kill(unsigned long data)
{
	struct cpad_context *cpad = (struct cpad_context *) data;

	cpad->display.timed_out = 1;
	usb_unlink_urb (cpad->display.out.urb);
	usb_unlink_urb (cpad->display.in.urb);
	err("cpad display urb timed out");
}

/*
 * probe/disconnect helper functions
 */

static void cpad_free_endpoint(struct cpad_endpoint *endpoint,
					struct cpad_context *cpad)
{
	if (endpoint->urb) {
		usb_buffer_free (cpad->udev, endpoint->buffer_size,
				endpoint->buffer, endpoint->urb->transfer_dma);
		usb_free_urb (endpoint->urb);
	}
}

static void cpad_free_context(struct cpad_context *cpad)
{
	if (!cpad)
		return;

	cpad_free_endpoint(&cpad->display.in, cpad);
	cpad_free_endpoint(&cpad->display.out, cpad);
	cpad_free_endpoint(&cpad->touchpad.in, cpad);
	if (cpad->display.tmpbuf)
		kfree (cpad->display.tmpbuf);
	if (cpad->fb.buffer)
		kfree(cpad->fb.buffer);
	cpad_procfs_remove(cpad);
	kfree (cpad);
}

static int cpad_setup_bulk_endpoint(struct cpad_endpoint *bulk,
					struct cpad_context *cpad)
{
	int pipe;
	size_t buffer_size;
	usb_complete_t bulk_callback;

	if (bulk->endpoint_desc->bEndpointAddress & USB_DIR_IN) {
		pipe = usb_rcvbulkpipe (cpad->udev,
					bulk->endpoint_desc->bEndpointAddress);
		buffer_size = bulk->endpoint_desc->wMaxPacketSize;
		bulk_callback = cpad_display_in_callback;
	}
	else {
		pipe = usb_sndbulkpipe (cpad->udev,
					bulk->endpoint_desc->bEndpointAddress);
		buffer_size = 274*bulk->endpoint_desc->wMaxPacketSize;
		bulk_callback = cpad_display_out_callback;
	}

	bulk->buffer_size = buffer_size;
	bulk->urb->transfer_flags = (URB_NO_TRANSFER_DMA_MAP |
					URB_ASYNC_UNLINK);
	bulk->buffer = usb_buffer_alloc (cpad->udev, buffer_size, GFP_KERNEL,
					&bulk->urb->transfer_dma);
	if (!bulk->buffer) {
		err("Could not allocate bulk buffer");
		return -ENOMEM;
	}
	usb_fill_bulk_urb (bulk->urb, cpad->udev, pipe, bulk->buffer,
					buffer_size, bulk_callback, cpad);
	return 0;
}

static int cpad_setup_int_endpoint(struct cpad_endpoint *tp, 
					struct cpad_context *cpad)
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
		err("Could not allocate irq_in_buffer");
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
		{ USB_DIR_IN,  USB_ENDPOINT_XFER_BULK, &cpad->display.in,
						cpad_setup_bulk_endpoint },
		{ USB_DIR_OUT, USB_ENDPOINT_XFER_BULK, &cpad->display.out,
						cpad_setup_bulk_endpoint },
		{ USB_DIR_IN,  USB_ENDPOINT_XFER_INT,  &cpad->touchpad.in,
						cpad_setup_int_endpoint  },
		{ 0, 0, 0, 0 }
	};

	for (i = 0; i < iface_desc->desc.bNumEndpoints; i++) {
		endpoint_desc = &iface_desc->endpoint[i].desc;
		for (j=0; endpoints[j].endpoint; j++)
			if (!endpoints[j].endpoint->endpoint_desc &&
			    ((endpoint_desc->bEndpointAddress & USB_DIR_IN)
						== endpoints[j].dir) &&
			    ((endpoint_desc->bmAttributes &
			      USB_ENDPOINT_XFERTYPE_MASK)
						== endpoints[j].xfer_type)) {

				endpoints[j].endpoint->endpoint_desc =
						endpoint_desc;

				endpoints[j].endpoint->urb =
						usb_alloc_urb (0, GFP_KERNEL);
				if (!endpoints[j].endpoint->urb) {
					err("No free urbs available");
					return -ENOMEM;
				}

				res = endpoints[j].setup (endpoints[j].endpoint,
						cpad);
				if (res)
					return res;
			}
	}

	if (!(cpad->display.in.endpoint_desc && 
	      cpad->display.out.endpoint_desc &&
	      cpad->touchpad.in.endpoint_desc)) {
		err("Could not find all cPad endpoints");
		return -ENODEV;
	}

	return 0;
}

/*
 * general helper functions
 *
 * call cpad_down before using any of these
 */

/* call with structure locked and after both display urbs finished */
static inline int cpad_check_display_urb_errors(struct cpad_context *cpad)
{
	if (cpad->display.timed_out)
		return -ETIMEDOUT;
	if (cpad->display.out.urb->status)
		return cpad->display.out.urb->status;
	if (cpad->display.submit_res)
		return cpad->display.submit_res;
	return cpad->display.in.urb->status;
}

/* wait for bulk urbs to finish. call with structure locked */
static int cpad_wait_interruptible(struct cpad_context *cpad)
{
	atomic_t *busy = &cpad->display.busy;
	int res;

	while (atomic_read (busy)) {
		res = wait_event_interruptible_timeout(cpad->display.queue,
						!atomic_read (busy), HZ/2);
		if (res<=0)
			return res;
	}

	return 0;
}

static inline int cpad_wait (struct cpad_context *cpad, struct file *file)
{
	int res;

	if (atomic_read (&cpad->display.busy)) {
		if (file->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		}
		res = cpad_wait_interruptible (cpad);
		if (res) {
			return res;
		}
	}
	return 0;
}

static inline int cpad_down(struct semaphore *sem, struct file *file)
{
	if (down_trylock (sem)) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		else if (down_interruptible (sem))
			return -ERESTARTSYS;
	}
	return 0;
}


static inline int cpad_submit_display_urb(struct cpad_context *cpad)
{
	int res;

	init_completion (&cpad->display.finished);
	atomic_set (&cpad->display.busy, 1);

	cpad->display.timer.expires = jiffies + 2*HZ;
	add_timer(&cpad->display.timer);
	cpad->display.timed_out = 0;

	res = usb_submit_urb(cpad->display.out.urb, GFP_KERNEL);
	cpad->display.submit_res = res;
	if (res) {
		del_timer_sync(&cpad->display.timer);
		atomic_set (&cpad->display.busy, 0);
		complete (&cpad->display.finished);
	}
	return res;
}

/* structure must be locked before calling this */
static int cpad_nlcd_function(struct cpad_context *cpad, unsigned char func,
				unsigned char val)
{
	unsigned char *out_buf = cpad->display.out.buffer;
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

	*(out_buf++) = SEL_CPAD;
	*(out_buf++) = func;
	*(out_buf++) = val;
	cpad->display.out.urb->transfer_buffer_length = 3;

	cpad->display.in.buffer[2] = 0;

	res = cpad_submit_display_urb (cpad);
	if (res) {
		err("failed submitting nlcd urb, error %d", res);
		return res;
	}
	res = cpad_wait_interruptible (cpad);
	if (res)
		return res;
	res = cpad_check_display_urb_errors(cpad);
	if (res)
		return res;

	return cpad->display.in.buffer[2];
}

static inline unsigned char *cpad_1335_fillpacket(unsigned char cmd,
		unsigned char *param, size_t param_size, unsigned char *out_buf)
{
	unsigned char *point;

	/* select 1335, set 1335 command, reverse params */
	*(out_buf++) = SEL_1335;
	*(out_buf++) = cmd;
	for (point=param+param_size-1; point>=param; point--)
		*(out_buf++) = *point;
	return out_buf;
}

/* don't call this, only used by cpad_flash */
static void cpad_light_off(void *arg)
{
	struct cpad_context *cpad = (struct cpad_context *) arg;

	down (&cpad->sem);
	if (cpad_nlcd_function (cpad, CPAD_W_LIGHT, 0))
		err("error on timed_light_off");
	up (&cpad->sem);
}

/* flash the backlight, time in 10ms. call with structure locked */
static int cpad_flash(struct cpad_context *cpad, int time)
{
	struct work_struct *flash = &cpad->display.flash;
	int retval;

	if (time <= 0)
		return -EINVAL;
	if (test_bit (0, &flash->pending))
		return -EBUSY;
	retval = cpad_nlcd_function(cpad, CPAD_W_LIGHT, 1);
	if (retval)
		err("cpad flash (maybe) failed to turn on");
	time = min(time, 1000);
	schedule_delayed_work (flash, (HZ * (unsigned long) time) / 100);
	return retval;
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

static void cpad_input_add(struct cpad_context *cpad)
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

	INIT_WORK (&cpad->touchpad.submit_urb, cpad_submit_int_urb, cpad);
	schedule_delayed_work (&cpad->touchpad.submit_urb, HZ/4);
}

static void cpad_input_remove(struct cpad_context *cpad)
{
	if (!disable_input)
		input_unregister_device(&cpad->touchpad.idev);
}

static void cpad_input_callback(struct urb *urb, struct pt_regs *regs)
{
	struct cpad_context *cpad = urb->context;
	unsigned char *data = cpad->touchpad.in.buffer;
	struct input_dev *idev = &cpad->touchpad.idev;
	int res, w, pressure, finger_width, num_fingers;

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

	if (disable_input)
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
		case 2:	/* pen detected, thread it as a finger */
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
		input_report_abs (idev, ABS_Y, YMIN_NOMINAL + YMAX_NOMINAL -
					((data[4] << 8) | data[5]));
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
	res = usb_submit_urb (urb, GFP_ATOMIC);
	if (res)
		err ("usb_submit_urb int in failed with result %d", res);
}

#else /* CONFIG_USB_CPADINPUT */

static void cpad_input_add(struct cpad_context *cpad)
{
	INIT_WORK (&cpad->touchpad.submit_urb, cpad_submit_int_urb, cpad);
	schedule_delayed_work (&cpad->touchpad.submit_urb, HZ/4);
}

static void cpad_input_remove(struct cpad_context *cpad) { }

/* data must always be fechted, otherwise cpad reconnects */
static void cpad_input_callback(struct urb *urb, struct pt_regs *regs)
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

/* data must always be fetched from the int endpoint, otherwise the cpad would
 * reconnect to force driver reload, so this is always scheduled by probe
 */
static void cpad_submit_int_urb(void *arg)
{
	struct cpad_context *cpad = (struct cpad_context *) arg;
	int res;

	down (&cpad->sem);
	res = usb_submit_urb (cpad->touchpad.in.urb, GFP_KERNEL);
	if (res)
		err ("usb_submit_urb int in failed with result %d", res);

	up (&cpad->sem);
}


/*****************************************************************************
 *	cpad character device routines, based on:			     *
 *		drivers/usb/usb-skeleton.c				     *
 *****************************************************************************/

#ifdef CONFIG_USB_CPADDEV

static int disable_cdev = 0;
MODULE_PARM(disable_cdev, "i");
MODULE_PARM_DESC(disable_cdev, "disable cPad character device");

#define USB_CPAD_MINOR_BASE	66
#define CPAD_DRIVER_NUM 	7

/* character device prototypes */
static int cpad_dev_open (struct inode *inode, struct file *file);
static int cpad_dev_release (struct inode *inode, struct file *file);
static ssize_t cpad_dev_read (struct file *file, char *buffer, size_t count,
					loff_t *ppos);
static ssize_t cpad_dev_write (struct file *file, const char *ubuffer,
					size_t count, loff_t *ppos);
static int cpad_dev_ioctl (struct inode *inode, struct file *file,
					unsigned int cmd, unsigned long arg);

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

static void cpad_dev_add(struct cpad_context *cpad)
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
	info ("USB Synaptics device now attached to USBcpad-%d",
							cpad->char_dev.minor);
}

static void cpad_dev_remove(struct cpad_context *cpad)
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
	int minor, retval;

	minor = iminor(inode);

	/* prevent disconnects */
	retval = cpad_down (&disconnect_sem, file);
	if (retval)
		return retval;

	interface = usb_find_interface (&cpad_driver, minor);
	if (!interface) {
		err ("error, can't find device for minor %d", minor);
		retval = -ENODEV;
		goto error;
	}

	cpad = usb_get_intfdata (interface);
	if (!cpad) {
		retval = -ENODEV;
		goto error;
	}

	down (&cpad->open_count_sem);
	++cpad->open_count;
	file->private_data = cpad;
	up (&cpad->open_count_sem);
error:
	up (&disconnect_sem);
	return retval;
}

static int cpad_dev_release (struct inode *inode, struct file *file)
{
	struct cpad_context *cpad = (struct cpad_context *)file->private_data;
	int retval = 0;

	if (cpad == NULL)
		return -ENODEV;

	down (&cpad->open_count_sem);

	if (cpad->open_count <= 0) {
		retval = -ENODEV;
		goto error;
	}

	if (!cpad->present && (cpad->open_count == 1)) {
		up (&cpad->open_count_sem);
		cpad_free_context (cpad);
		return 0;
	}

	--cpad->open_count;
error:
	up (&cpad->open_count_sem);
	return retval;
}

static ssize_t cpad_dev_read (struct file *file, char *buffer, size_t count,
					loff_t *ppos)
{
	struct cpad_context *cpad = (struct cpad_context *)file->private_data;
	ssize_t bytes_read;
	int retval;

	retval = cpad_down (&cpad->sem, file);
	if (retval)
		return retval;

	if (!cpad->present) {
		retval = -ENODEV;
		goto error;
	}

	retval = cpad_wait (cpad, file);
	if (retval)
		goto error;

	retval = cpad_check_display_urb_errors(cpad);
	if (retval)
		goto error;

	bytes_read = min ((int)count, cpad->display.in.urb->actual_length);
	if (copy_to_user (buffer, cpad->display.in.buffer, bytes_read)) {
		retval = -EFAULT;
	} else {
		retval = bytes_read;
	}
error:
	up (&cpad->sem);
	return retval;
}

static inline int cpad_dev_write_fillbuffer(struct cpad_context *cpad,
				const unsigned char *ubuffer, size_t count)
{
	unsigned char *out_buf = cpad->display.out.buffer;
	size_t out_buffer_size = cpad->display.out.buffer_size;
	unsigned char *param = cpad->display.tmpbuf;
	size_t max_param_size = cpad->display.tmpbuf_size;
	size_t param_size, actual_length;
	unsigned char cmd;
	const unsigned char *ubuffer_orig = ubuffer;

	/* get 1335 command first */
	get_user(cmd, ubuffer);
	ubuffer++;
	if (count == 1) {
		/* 1335 command without params */
		*(out_buf++) = SEL_1335;
		*(out_buf++) = cmd;
		actual_length = 2;
	}
	else {
		actual_length = 0;
		count--;
		while (count > 0) {
			param_size = min (max_param_size, count);
			if (actual_length+param_size+2 > out_buffer_size)
				break;

			if (copy_from_user(param, ubuffer, param_size)) {
				return -EFAULT;
			}
			ubuffer += param_size;
			count -= param_size;

			out_buf = cpad_1335_fillpacket(cmd, param, param_size,
								out_buf);
			actual_length += param_size + 2;
		}
	}

	cpad->display.out.urb->transfer_buffer_length = actual_length;
	return ubuffer-ubuffer_orig;
}

static ssize_t cpad_dev_write (struct file *file, const char *ubuffer,
					size_t count, loff_t *ppos)
{
	struct cpad_context *cpad = (struct cpad_context *)file->private_data;
	int retval, length;

	if (count == 0)
		return 0;

	retval = cpad_down (&cpad->sem, file);
	if (retval)
		return retval;

	if (!cpad->present) {
		retval = -ENODEV;
		goto error;
	}

	retval = cpad_wait (cpad, file);
	if (retval)
		goto error;

	cpad_fb_deactivate (cpad);

	length = cpad_dev_write_fillbuffer (cpad, ubuffer, count);
	if (length < 0) {
		retval = length;
		goto error;
	}

	retval = cpad_submit_display_urb(cpad);
	if (retval) {
		err ("failed submitting write urb, error %d", retval);
	} else {
		retval = length;
	}
error:
	up (&cpad->sem);
	return retval;
}

int cpad_driver_num = CPAD_DRIVER_NUM;

static int cpad_dev_ioctl (struct inode *inode, struct file *file,
					unsigned int cmd, unsigned long arg)
{
	struct cpad_context *cpad = (struct cpad_context *)file->private_data;
	unsigned char cval = 0;
	int ival = 0;
	void *rval = NULL;
	int res = 0;

	res = cpad_down (&cpad->sem, file);
	if (res)
		return res;

	if (!cpad->present) {
		res = -ENODEV;
		goto error;
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
			res = -ENOIOCTLCMD;
			goto error;
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

#else /* CONFIG_USB_CPADDEV */

static void cpad_dev_add(struct cpad_context *cpad) { }
static void cpad_dev_remove(struct cpad_context *cpad) { }

#endif /* CONFIG_USB_CPADDEV */


/*****************************************************************************
 *	procfs routines							     *
 *****************************************************************************/

#ifdef CONFIG_USB_CPADPROCFS

static int disable_procfs = 0;
MODULE_PARM(disable_procfs, "i");
MODULE_PARM_DESC(disable_procfs, "disable cPad procfs interface");

static struct proc_dir_entry *cpad_procfs_root;

/* procfs prototypes */
static int cpad_procfs_open(struct inode *inode, struct file *file);
static int cpad_procfs_release(struct inode *inode, struct file *file);
static int cpad_procfs_read(struct file *file, char __user *buf,
				size_t nbytes, loff_t *ppos);
static int cpad_procfs_write(struct file *file, const char __user *buffer,
				size_t count, loff_t *ppos);

struct file_operations cpad_proc_fops = {
	.open = cpad_procfs_open,
	.release = cpad_procfs_release,
	.read = cpad_procfs_read,
	.write = cpad_procfs_write,
};

static void cpad_procfs_init(void)
{
	if (disable_procfs)
		return;

	cpad_procfs_root = proc_mkdir("driver/cpad", NULL);
	if (! cpad_procfs_root)
	{
		disable_procfs = 1;
		err("can not create /proc/driver/cpad");
		return;
	}
	cpad_procfs_root->owner = THIS_MODULE;
}

static void cpad_procfs_exit(void)
{
	if (disable_procfs)
		return;
	remove_proc_entry("driver/cpad", NULL);
}

static void cpad_procfs_add(struct cpad_context *cpad)
{
	struct cpad_procfs *procfs = &cpad->procfs;

	if (disable_procfs)
		return;

	if (cpad->table_index < 0) {
		err("cpad_table is full, proc file disabled");
		cpad->procfs.disable = 1;
		return;
	}

	sprintf(procfs->name, "%i", cpad->table_index);
	procfs->entry = create_proc_entry(procfs->name,
					S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP,
					cpad_procfs_root);
	if (! procfs->entry)
	{
		err("can not create /proc/driver/cpad/%s", procfs->name);
		cpad->procfs.disable = 1;
		return;
	}
	procfs->entry->owner = THIS_MODULE;
	procfs->entry->data = (void *) cpad->table_index;
	procfs->entry->proc_fops = &cpad_proc_fops;
}

static void cpad_procfs_remove(struct cpad_context *cpad)
{
	if (disable_procfs || cpad->procfs.disable)
		return;
	remove_proc_entry(cpad->procfs.name, cpad_procfs_root);
	cpad->procfs.disable = 1;
}

static int cpad_procfs_open(struct inode *inode, struct file *file)
{
	struct cpad_context *cpad;
	static struct proc_dir_entry *entry;
	int retval = 0;
	int index = 0;

	retval = cpad_down (&disconnect_sem, file);
	if (retval)
		return retval;

	entry = PDE(inode);
	index = (int) entry->data;
	if ((index < 0) || (index >= MAX_DEVICES)) {
		retval = -ENODEV;
		goto error;
	}

	cpad = cpad_table[index];
	if ((!cpad) || (cpad->procfs.entry != entry)) {
		retval = -ENODEV;
		goto error;
	}

	down (&cpad->open_count_sem);
	++cpad->open_count;
	++cpad->procfs.open;
	file->private_data = cpad;
	up (&cpad->open_count_sem);
error:
	up (&disconnect_sem);
	return retval;
}

static int cpad_procfs_release(struct inode *inode, struct file *file)
{
	struct cpad_context *cpad = (struct cpad_context *)file->private_data;
	int retval = 0;

	if (cpad == NULL)
		return -ENODEV;

	down (&cpad->open_count_sem);

	if (cpad->open_count <= 0) {
		retval = -ENODEV;
		goto error;
	}

	if (!cpad->present) {
		if (cpad->procfs.open == 1) {
			cpad_procfs_remove (cpad);
		}
		if (cpad->open_count == 1) {
			up (&cpad->open_count_sem);
			cpad_free_context (cpad);
			return 0;
		}
	}

	--cpad->open_count;
	--cpad->procfs.open;
error:
	up (&cpad->open_count_sem);
	return retval;
}

static char *cpad_procfs_read_generic(struct cpad_context *cpad, char *pos)
{
	int i;

	pos += sprintf(pos, "eeprom:     ");
	cpad_nlcd_function(cpad, CPAD_R_ROM, 0);
	for (i=2; i < cpad->display.in.urb->actual_length; i++)
		pos += sprintf(pos, " %02x",
				cpad->display.in.buffer[i]);
	pos += sprintf(pos, "\n");

	pos += sprintf(pos, "lcd:         %i\n",
				cpad_nlcd_function(cpad, CPAD_R_LCD, 0));
	pos += sprintf(pos, "backlight:   %i\n",
				cpad_nlcd_function(cpad, CPAD_R_LIGHT, 0));
	return pos;
}

static char *cpad_procfs_read_input(struct cpad_context *cpad, char *pos)
{
#ifdef CONFIG_USB_CPADINPUT
	struct input_handle *handle;

	if (disable_input) {
		pos += sprintf(pos, "input dev:   disabled\n");
	} else {
		pos += sprintf(pos, "input dev:  ");
		list_for_each_entry(handle, &cpad->touchpad.idev.h_list, d_node)
			pos += sprintf(pos, " %s", handle->name);
		pos += sprintf(pos, "\n");
	}
#else
	pos += sprintf(pos, "input dev:   not compiled in\n");
#endif
	return pos;
}

static char *cpad_procfs_read_cdev(struct cpad_context *cpad, char *pos)
{
#ifdef CONFIG_USB_CPADDEV
	if (disable_cdev || cpad->char_dev.disable) {
		pos += sprintf(pos, "cdev:        disabled\n");
	} else {
		pos += sprintf(pos, "cdev minor:  %i\n",
					(int) cpad->char_dev.minor);
	}
#else
	pos += sprintf(pos, "cdev:        not comliped in\n");
#endif
	return pos;
}

static char *cpad_procfs_read_fb(struct cpad_context *cpad, char *pos)
{
#ifdef CONFIG_USB_CPADFB
	if (disable_fb || cpad->fb.disable) {
		pos += sprintf(pos, "framebuffer: disabled\n");
	} else {
		pos += sprintf(pos, "fb nr.:      %i\n", cpad->fb.node);
		pos += sprintf(pos, "framerate:   %i\n", cpad->fb.rate);
		pos += sprintf(pos, "dithering:   %i\n", cpad->fb.dither);
		pos += sprintf(pos, "brightness:  %i\n", cpad->fb.brightness);
		pos += sprintf(pos, "invert:      %i\n", cpad->fb.invert);
		pos += sprintf(pos, "onlychanged: %i\n", cpad->fb.onlychanged);
		pos += sprintf(pos, "onlyvisible: %i\n", cpad->fb.onlyvisible);
	}
#else
	pos += sprintf(pos, "framebuffer: not comliped in\n");
#endif
	return pos;
}

static int cpad_procfs_read(struct file *file, char __user *buf,
				size_t nbytes, loff_t *ppos)
{
	struct cpad_context *cpad = (struct cpad_context *)file->private_data;
	char *page = (char*) __get_free_page(GFP_KERNEL);
	char *page_pos = page;
	int retval;

	if (!page)
		return -ENOMEM;

	retval = cpad_down (&cpad->sem, file);
	if (retval)
		goto error;

	if (!cpad->present) {
		retval = -ENODEV;
		goto error;
	}

	page_pos = cpad_procfs_read_generic (cpad, page_pos);
	page_pos = cpad_procfs_read_input (cpad, page_pos);
	page_pos = cpad_procfs_read_cdev (cpad, page_pos);
	page_pos = cpad_procfs_read_fb (cpad, page_pos);

	up(&cpad->sem);

	retval = min(max((int)(page_pos - page - *ppos), 0), (int)nbytes);
	if (retval == 0)
		goto error;
	if (copy_to_user(buf, page + *ppos, retval)) {
		retval = -EFAULT;
		goto error;
	}
	*ppos += retval;
error:
	free_page((unsigned long) page);
	return retval;
}

static inline void cpad_procfs_write_command(struct cpad_context *cpad,
							char *cmd)
{
	int value;
	char str[16];

	if (sscanf(cmd, " flash : %i", &value) == 1)
		cpad_flash(cpad, value);
	else if (sscanf(cmd, " framerate : %i", &value) == 1) {
		if (value <= 0)
			cpad_fb_deactivate (cpad);
		else
			cpad_fb_activate (cpad, value);
	} else if (sscanf(cmd, " dithering : %i", &value) == 1) {
		cpad->fb.dither = min(max(value, 0), 3);
	} else if (sscanf(cmd, " brightness : %i", &value) == 1) {
		cpad->fb.brightness = min(max(value, 0), 10000);
	} else if (sscanf(cmd, " invert : %i", &value) == 1) {
		cpad->fb.invert = (value <= 0) ? 0 : 1;
	} else if (sscanf(cmd, " onlychanged : %i", &value) == 1) {
		cpad->fb.onlychanged = (value <= 0) ? 0 : 1;
	} else if (sscanf(cmd, " onlyvisible : %i", &value) == 1) {
		cpad->fb.onlyvisible = (value <= 0) ? 0 : 1;
	} else if (sscanf(cmd, " %16s", str) == 1) {
		if (!strcmp("oneframe", str))
			cpad_fb_oneframe(cpad);
	}
}

static int cpad_procfs_write(struct file *file, const char __user *buffer,
				size_t count, loff_t *ppos)
{
	struct cpad_context *cpad = (struct cpad_context *)file->private_data;
	char *page = (char*) __get_free_page(GFP_KERNEL);
	char *page_pos = page;
	int retval;

	if (!page)
		return -ENOMEM;

	count = min(count, (size_t)(PAGE_SIZE-1));
	if (copy_from_user(page, buffer, count)) {
		retval = -EFAULT;
		goto error2;
	}
	page[count] = '\0';

	retval = cpad_down (&cpad->sem, file);
	if (retval)
		goto error2;

	if (!cpad->present) {
		retval = -ENODEV;
		goto error1;
	}

	retval = count;
	while (count) {
		cpad_procfs_write_command (cpad, page_pos);

		do {
			++page_pos;
			--count;
		} while (count && *(page_pos-1) != '\n' &&
						*(page_pos-1) != ';');
	}
error1:
	up(&cpad->sem);
error2:
	free_page((unsigned long) page);
	return retval;
}

#else /* CONFIG_USB_CPADPROCFS */

static void cpad_procfs_init(void) { }
static void cpad_procfs_exit(void) { }
static void cpad_procfs_add(struct cpad_context *cpad) { }
static void cpad_procfs_remove(struct cpad_context *cpad) { }

#endif /* CONFIG_USB_CPADPROCFS */


/*****************************************************************************
 *	frame buffer routines, based on:				     *
 *		drivers/video/vfb.c					     *
 *		drivers/usb/media/vicam.c	(mmap functions)	     *
 *****************************************************************************/

#ifdef CONFIG_USB_CPADFB

static int disable_fb = 0;
MODULE_PARM(disable_fb, "i");
MODULE_PARM_DESC(disable_fb, "disable cPad frame buffer device");

static int max_bpp = 24;
MODULE_PARM(max_bpp, "i");
MODULE_PARM_DESC(max_bpp, "maximum bpp, set to 1 to reduce mem usage");

static struct fb_var_screeninfo cpad_fb_default_var = {
	.xres =		240,
	.yres =		160,
	.xres_virtual =	240,
	.yres_virtual =	160,
	.bits_per_pixel = 1,
	.red =		{ 0, 1, 0 },
      	.green =	{ 0, 1, 0 },
      	.blue =		{ 0, 1, 0 },
      	.activate =	FB_ACTIVATE_NOW,
      	.height =	38,
      	.width =	46,
      	.vmode =	FB_VMODE_NONINTERLACED,
};

static struct fb_fix_screeninfo cpad_fb_default_fix = {
	.id =		"cPad FB",
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	FB_VISUAL_MONO01,
	.accel =	FB_ACCEL_NONE,
	.line_length =	240/8,
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

/* frame buffer device prototypes */
static int cpad_fb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
					u_int transp, struct fb_info *info);
static int cpad_fb_open(struct fb_info *info, int user);
static int cpad_fb_release(struct fb_info *info, int user);
static int cpad_fb_mmap(struct fb_info *info, struct file *file,
			struct vm_area_struct *vma);
static void cpad_fb_sendimage(void *arg);
int cpad_fb_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
				unsigned long arg, struct fb_info *info);
int cpad_fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info);
int cpad_fb_set_par(struct fb_info *info);

static struct fb_ops cpad_fb_ops = {
	.owner		= THIS_MODULE,
	.fb_setcolreg	= cpad_fb_setcolreg,
	.fb_check_var	= cpad_fb_check_var,
	.fb_set_par	= cpad_fb_set_par,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_cursor	= soft_cursor,
	.fb_open	= cpad_fb_open,
	.fb_release	= cpad_fb_release,
	.fb_mmap	= cpad_fb_mmap,
	.fb_ioctl	= cpad_fb_ioctl,
};

static void cpad_fb_add(struct cpad_context *cpad)
{
	struct fb_info *info = &cpad->fb.info;
	u_long vsize, size;
	void *adr;

	if (disable_fb)
		return;

	if (cpad->table_index < 0) {
		err("cpad_table is full, framebuffer disabled");
		cpad->fb.disable = 1;
		return;
	}

	if (max_bpp == 1) {
		vsize = 8192;
	} else {
		vsize = 118784;
	}

	cpad_fb_default_fix.smem_len = vsize;
	if (!(cpad->fb.videomemory = vmalloc(vsize))) {
		cpad->fb.disable = 1;
		err("not enough memory for frame buffer");
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
		cpad_fb_free (cpad);
		cpad->fb.disable = 1;
		err("could not register frame buffer");
		return;
	}

	cpad->fb.node = info->node;
	cpad->fb.brightness = 200;
	cpad->fb.dither = 3;
	cpad->fb.onlyvisible = 1;
}

static void cpad_fb_remove(struct cpad_context *cpad)
{
	if (disable_fb || cpad->fb.disable)
		return;

	cpad_fb_free (cpad);
	unregister_framebuffer (&cpad->fb.info);
}

static void cpad_fb_free (struct cpad_context *cpad)
{
	unsigned long adr, size;

	if (!cpad->fb.videomemory)
		return;

	adr = (unsigned long) cpad->fb.videomemory;
	size = cpad->fb.info.fix.smem_len;
	while ((long) size > 0) {
		ClearPageReserved(vmalloc_to_page((void *)adr));
		adr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
	vfree(cpad->fb.videomemory);
	cpad->fb.videomemory = 0;
}

static void cpad_fb_activate(struct cpad_context *cpad, int rate)
{
	if (disable_fb || cpad->fb.disable)
		return;

	cpad->fb.rate = min(rate, 60);

	if (cpad->fb.active)
		return;

	cpad->fb.active = 1;
	init_completion (&cpad->fb.finished);
	INIT_WORK (&cpad->fb.sendimage, cpad_fb_sendimage, cpad);
	schedule_work (&cpad->fb.sendimage);
}

static void cpad_fb_deactivate(struct cpad_context *cpad)
{
	if (cpad->fb.active) {
		cpad->fb.active = 0;
		if (!cancel_delayed_work(&cpad->fb.sendimage)) 
			wait_for_completion (&cpad->fb.finished);
		cpad->fb.rate = 0;
	}
}

static int cpad_fb_open(struct fb_info *info, int user)
{
	struct cpad_context *cpad = (struct cpad_context *)info->par;
	int retval = 0;
	int index;

	if (cpad == NULL)
		return -ENODEV;

	if (down_interruptible (&disconnect_sem))
		return -ERESTARTSYS;

	index = cpad->table_index;
	if ((index < 0) || (index >= MAX_DEVICES)) {
		retval = -ENODEV;
		goto error;
	}

	if ((cpad != cpad_table[index]) || (&cpad->fb.info != info)) {
		retval = -ENODEV;
		goto error;
	}

	down (&cpad->open_count_sem);
	++cpad->open_count;
	++cpad->fb.open;
	up (&cpad->open_count_sem);
error:
	up (&disconnect_sem);
	return retval;
}

static int cpad_fb_release(struct fb_info *info, int user)
{
	struct cpad_context *cpad = (struct cpad_context *)info->par;
	int retval = 0;

	if (cpad == NULL)
		return -ENODEV;

	down (&cpad->open_count_sem);

	if (cpad->fb.open <= 0) {
		retval = -ENODEV;
		goto error;
	}

	if (!cpad->present) {
		if (cpad->fb.open == 1) {
			cpad_fb_remove (cpad);
		}
		if (cpad->open_count == 1) {
			up (&cpad->open_count_sem);
			cpad_free_context (cpad);
			return 0;
		}
	}

	--cpad->open_count;
	--cpad->fb.open;
error:
	up (&cpad->open_count_sem);
	return retval;
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
		kva = (unsigned long)page_address(vmalloc_to_page((void *)pos));
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

static int cpad_fb_sendurb(struct cpad_context *cpad, int length)
{
	int res;

	cpad->display.out.urb->transfer_buffer_length = length;
	res = cpad_submit_display_urb(cpad);
	cpad->display.submit_res = res;
	if (res)
		goto error;

	wait_for_completion (&cpad->display.finished);
	res = cpad_check_display_urb_errors(cpad);
error:
	if (res) {
		err("sending urb failed, error %d. deactivating "
						"frame buffer", res);
		return res;
	}
	return 0;
}

static int cpad_fb_setcursor(struct cpad_context *cpad, int cursor)
{
	unsigned char *out_buf = cpad->display.out.buffer;

	*(out_buf++) = SEL_1335;
	*(out_buf++) = CSRW_1335;
	*(out_buf++) = cursor >> 8;
	*(out_buf++) = cursor & 0xff;
	return cpad_fb_sendurb(cpad, 4);
}

static inline int cpad_fb_dither(int x, int y, int grey, int maxgrey,
							int dither)
{
	switch (dither) {
	case 0:	return (2*grey > maxgrey) ? 0 : 1;
	case 1:	return (5*grey > cpad_fb_dither2[x%2][y%2]*maxgrey) ? 0 : 1;
	case 2:	return (17*grey > cpad_fb_dither4[x%4][y%4]*maxgrey) ? 0 : 1;
	case 3:	return (65*grey > cpad_fb_dither8[x%8][y%8]*maxgrey) ? 0 : 1;
	}
}

static inline unsigned char *cpad_fb_convert_line(struct cpad_context *cpad,
			int line, unsigned char *vmem, unsigned char *param)
{
	int byte, bit, grey, black;
	int maxgrey = 255*10;
	int x = 0;

	for (byte=0; byte<30; byte++)
		for (bit=7; bit>=0; bit--) {
			grey = 3*vmem[0] + 6*vmem[1] + vmem[2];
			grey = min((grey*cpad->fb.brightness)/100, maxgrey);

			black = cpad_fb_dither(x, line, grey, maxgrey,
							cpad->fb.dither);

			if (cpad->fb.invert)
				black = !black;

			if (black) {
				set_bit (bit, (unsigned long *) &param[byte]);
			}
			else {
				clear_bit (bit, (unsigned long *) &param[byte]);
			}
			vmem += 3;
			x++;
		}

	return vmem;
}

static inline unsigned char *cpad_fb_fillpacket(unsigned char *param,
			size_t param_size, unsigned char *out_buf, int *changed)
{
	unsigned char *point;

	*changed = param_size;
	*(out_buf++) = SEL_1335;
	*(out_buf++) = MWRITE_1335;
	for (point=param+param_size-1; point>=param; point--) {
		if (*out_buf == *point) {
			(*changed)--;
		} else {
			*out_buf = *point;
		}
		out_buf++;
	}
	return out_buf;
}

static inline void cpad_fb_oneframe_1bpp(struct cpad_context *cpad,
							int *urb_size)
{
	unsigned char *param = cpad->fb.videomemory;
	unsigned char *out_buf = cpad->display.out.buffer;
	int line, maxline;

	if (cpad->fb.onlyvisible) {
		maxline = 160;
	} else {
		maxline = 273;
	}

	for (line=0; line<maxline; line++) {
		out_buf = cpad_1335_fillpacket(MWRITE_1335, param, 30, out_buf);
		param += 30;
	}

	if (cpad->fb.onlyvisible) {
		*urb_size = 160*32;
	} else {
		cpad_1335_fillpacket (MWRITE_1335, param, 2, out_buf);
		*urb_size = 273*32 + 4;
	}
}

static inline void cpad_fb_oneframe_24bpp(struct cpad_context *cpad,
							int *urb_size)
{
	unsigned char *param = cpad->display.tmpbuf;
	unsigned char *out_buf = cpad->display.out.buffer;
	unsigned char *vmem = cpad->fb.videomemory;
	int line;

	for (line=0; line<160; line++) {
		vmem = cpad_fb_convert_line (cpad, line, vmem, param);
		out_buf = cpad_1335_fillpacket(MWRITE_1335, param, 30, out_buf);
	}
	*urb_size = 160*32;
}

static inline void cpad_fb_oneframe_1bpp_onlychanged(struct cpad_context *cpad,
						int *firstline, int *urb_size)
{
	unsigned char *param = cpad->fb.videomemory;
	unsigned char *out_buf = cpad->fb.buffer;
	int line, changed, maxline;
	int lastline = 0;
	int gotfirstline = 0;

	if (cpad->fb.onlyvisible) {
		maxline = 160;
	} else {
		maxline = 274;
	}

	for (line=0; line<maxline; line++) {
		out_buf = cpad_fb_fillpacket (param, (line==273) ? 2 : 30,
							out_buf, &changed);
		param += 30;
		if (changed) {
			if (gotfirstline) {
				lastline = line;
			} else {
				*firstline = line;
				lastline = line;
				gotfirstline = 1;
			}
		}
	}

	if (gotfirstline) {
		*urb_size = (lastline - *firstline + 1) * 32;
		if (lastline == 273)
			*urb_size -= 28;
	} else {
		*urb_size = 0;
	}
}

static inline void cpad_fb_oneframe_24bpp_onlychanged(struct cpad_context *cpad,
						int *firstline, int *urb_size)
{
	unsigned char *param = cpad->display.tmpbuf;
	unsigned char *out_buf = cpad->fb.buffer;
	unsigned char *vmem = cpad->fb.videomemory;
	int line, changed;
	int lastline = 0;
	int gotfirstline = 0;

	for (line=0; line<160; line++) {
		vmem = cpad_fb_convert_line (cpad, line, vmem, param);
		out_buf = cpad_fb_fillpacket (param, 30, out_buf, &changed);
		if (changed) {
			if (gotfirstline) {
				lastline = line;
			} else {
				*firstline = line;
				lastline = line;
				gotfirstline = 1;
			}
		}
	}

	if (gotfirstline) {
		*urb_size = (lastline - *firstline + 1) * 32;
	} else {
		*urb_size = 0;
	}
}

static int cpad_fb_oneframe(struct cpad_context *cpad)
{
	int res;
	int urb_size;
	int firstline;
	int bpp = cpad->fb.info.var.bits_per_pixel;

	if (cpad->fb.onlychanged) {
		if (bpp == 1) {
			cpad_fb_oneframe_1bpp_onlychanged(cpad, &firstline,
								&urb_size);
		} else {
			cpad_fb_oneframe_24bpp_onlychanged(cpad, &firstline,
								&urb_size);
		}

		res = cpad_fb_setcursor(cpad, firstline*30);
		if (res)
			return res;

		memcpy(cpad->display.out.buffer, cpad->fb.buffer + firstline*32,
								urb_size);
	} else {
		res = cpad_fb_setcursor(cpad, 0);
		if (res)
			return res;

		if (bpp == 1) {
			cpad_fb_oneframe_1bpp(cpad, &urb_size);
		} else {
			cpad_fb_oneframe_24bpp(cpad, &urb_size);
		}
	}

	if (urb_size) {
		return cpad_fb_sendurb(cpad, urb_size);
	} else {
		return 0;
	}
}

static void cpad_fb_sendimage(void *arg)
{
	struct cpad_context *cpad = (struct cpad_context *) arg;

	if (!cpad->fb.active) {
		complete(&cpad->fb.finished);
		return;
	}

	if (down_trylock (&cpad->sem)) {
		INIT_WORK (&cpad->fb.sendimage, cpad_fb_sendimage, cpad);
		schedule_delayed_work (&cpad->fb.sendimage, HZ/60);
		return;
	}

	if (atomic_read (&cpad->display.busy))
		wait_for_completion (&cpad->display.finished);

	if (cpad_fb_oneframe(cpad)) {
		cpad->fb.active = 0;
		cpad->fb.rate = 0;
	} else {
		INIT_WORK (&cpad->fb.sendimage, cpad_fb_sendimage, cpad);
		schedule_delayed_work (&cpad->fb.sendimage, HZ/cpad->fb.rate);
	}

	up (&cpad->sem);
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

int cpad_fb_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
				unsigned long arg, struct fb_info *info)
{
	struct cpad_context *cpad = (struct cpad_context *)info->par;
	unsigned char cval = 0;
	int ival = 0;
	int res = 0;

	res = cpad_down (&cpad->sem, file);
	if (res)
		return res;

	if (!cpad->present) {
		res = -ENODEV;
		goto error;
	}

	if (cmd & IOC_IN) {
		switch (_IOC_SIZE(cmd)) {
		case sizeof(char):
			get_user(cval, (char *) arg);
			break;
		case sizeof(int):
			get_user(ival, (int *) arg);
			break;
		default:
			res = -ENOIOCTLCMD;
			goto error;
		}
	}

	switch (cmd) {
	case CPAD_FRAMERATE:
		if (cval <= 0)
			cpad_fb_deactivate (cpad);
		else
			cpad_fb_activate (cpad, cval);
		break;

	case CPAD_DITHER:
		cpad->fb.dither = min(max((int) cval, 0), 3);
		break;

	case CPAD_BRIGHTNESS:
		cpad->fb.brightness = min(max(ival, 0), 10000);
		break;

	case CPAD_INVERT:
		cpad->fb.invert = (cval <= 0) ? 0 : 1;
		break;

	case CPAD_ONLYCHANGED:
		cpad->fb.onlychanged = (cval <= 0) ? 0 : 1;
		break;

	case CPAD_ONLYVISIBLE:
		cpad->fb.onlyvisible = (cval <= 0) ? 0 : 1;
		break;

	case CPAD_ONEFRAME:
		res = cpad_fb_oneframe(cpad);
		break;

	default:
		res = -ENOIOCTLCMD;
	}
error:
	up (&cpad->sem);
	if (res < 0)
		return res;

	return 0;
}

int cpad_fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	var->xres = 240;
	var->yres = 160;
	var->xres_virtual = 240;
	var->yres_virtual = 160;
      	var->activate = FB_ACTIVATE_NOW;
      	var->height = 38;
      	var->width = 46;
      	var->vmode = FB_VMODE_NONINTERLACED;

	if ((var->bits_per_pixel == 1) || (max_bpp == 1)) {
		var->bits_per_pixel = 1;
		var->red.offset = 0;
		var->red.length = 1;
		var->green.offset = 0;
		var->green.length = 1;
		var->blue.offset = 0;
		var->blue.length = 1;
	} else {
		var->bits_per_pixel = 24;
		var->red.offset = 0;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 16;
		var->blue.length = 8;
	}
	var->red.msb_right = 0;
	var->green.msb_right = 0;
	var->blue.msb_right = 0;
	var->transp.offset = 0;
	var->transp.length = 0;
	var->transp.msb_right = 0;

	return 0;
}

int cpad_fb_set_par(struct fb_info *info)
{
	if (info->var.bits_per_pixel == 1) {
		info->fix.visual = FB_VISUAL_MONO01;
		info->fix.line_length = 240/8;
	} else {
		info->fix.visual = FB_VISUAL_TRUECOLOR;
		info->fix.line_length = 240*3;
	}
	return 0;
}

#else /* CONFIG_USB_CPADFB */

static int disable_fb = 1;
static void cpad_fb_add(struct cpad_context *cpad) { }
static void cpad_fb_remove(struct cpad_context *cpad) { }
static void cpad_fb_free (struct cpad_context *cpad) { }
static void cpad_fb_activate(struct cpad_context *cpad, int rate) { }
static void cpad_fb_deactivate(struct cpad_context *cpad) { }
static int cpad_fb_oneframe(struct cpad_context *cpad) { return 0; }

#endif /* CONFIG_USB_CPADFB */
