#ifndef _LINUX_CPAD_H
#define _LINUX_CPAD_H

#include <linux/input.h>


/*****************************************************************************
 * /dev/usb/cpad* character device					     *
 *	supported operations: read, write, ioctl			     *
 *	it is compatible to version 0.7 of Rob Miller's and Ron Lee's cPad   *
 *	driver for kernel 2.4, available at:				     *
 *		http://www.janerob.com/rob/ts5100/cPad/index.shtml    	     *
 *	example programs for the character device are usr_cpad (in the	     *
 *	kernel 2.4 driver tarball) and cpadconsole from:		     *
 *		http://www.uni-jena.de/~p1stja/linux/cpadconsole.html	     *
 *****************************************************************************/

#define CPAD_IOCTL_BASE		0x71

/* supported ioctls
 * because the backlight has a finite lifespan (ca. 1000 - 3000 hours), the
 * CPAD_FLASH ioctl should be used instead of CPAD_WLIGHT.
 * to get the result of CPAD_RLIGHT and CPAD_RLCD, read some bytes from
 * the character device. the third byte is the result.
 */
#define CPAD_VERSION		_IOR('U', CPAD_IOCTL_BASE, int)
						/* get driver version */
#define CPAD_CGID		_IOR('U', CPAD_IOCTL_BASE+1, struct input_id)
						/* get device ID */
#define CPAD_RESET		_IO('U', CPAD_IOCTL_BASE+2)
						/* reset usb device cpad */
#define CPAD_WLIGHT		_IOW('U', CPAD_IOCTL_BASE+3, unsigned char)
						/* set backlight state */
#define CPAD_WLCD		_IOW('U', CPAD_IOCTL_BASE+5, unsigned char)
						/* set LCD state */
#define CPAD_RLIGHT		_IO('U', CPAD_IOCTL_BASE+6)
						/* read backlight state */
#define CPAD_RLCD		_IO('U', CPAD_IOCTL_BASE+7)
						/* read LCD state */
#define CPAD_FLASH		_IOW('U', CPAD_IOCTL_BASE+8, int)
						/* flash backlight */

/* ioctls not yet supported */
#define CPAD_WIMAGEL		_IOW('U', CPAD_IOCTL_BASE+4, char[30])
						/* write an image line */

/* ioctls not supported by this driver, only added for compatibility here
   these things are handled by Synaptics TouchPad driver for XFree86 now */
#define CPAD_SET_STROKE		_IOW('U', CPAD_IOCTL_BASE+9, int)
						/* set mouse motion sensit. */
#define CPAD_SET_SENS		_IOW('U', CPAD_IOCTL_BASE+15, int)
						/* set tap sensitivity */
#define CPAD_SET_ABS		_IOW('U', CPAD_IOCTL_BASE+16, char)
						/* set abs/rel output mode */

/*
 * writing to /dev/usb/cpad*
 *	the cPad display is controlled by a Seiko/Epson 1335 LCD Controller IC
 *	a write to the device consists of a command followed by data:
 *		<1335 command> [<data> ...]
 *	for MRWITE_1335 command, data may be up to 160*30 bytes long
 *	possible commands as reported in the sed1335 data sheet are listed
 *	below. look at the cpadconsole program (specially sed1335.h) to see how
 *	to use these commands.
 */
/* System control */
#define SYSSET_1335		0x40
#define SLEEP_1335		0x53

/* Display control */
#define DISPOFF_1335		0x58
#define DISP_1335		0x59
#define SCROLL_1335		0x44
#define CSRF_1335		0x5d
#define CGRAMADR_1335		0x5c
#define CSRDIR_RIGHT_1335	0x4c
#define CSRDIR_LEFT_1335	0x4d
#define CSRDIR_UP_1335		0x4e
#define CSRDIR_DOWN_1335	0x4f
#define HDOTSCR_1335		0x5a
#define OVLAY_1335		0x5b

/* Drawing control */
#define CSRW_1335		0x46
#define CSRR_1335		0x47

/* Memory control */
#define MWRITE_1335		0x42
#define MREAD_1335		0x43

/*
 * reading /dev/usb/cpad*
 *	reads answer of the sed1335 to a command, can be 0-32 bytes long.
 *	the information is only accessible until the next write access.
 *	because writes are done asyncronously, some write errors can occure
 *	after write returned successful. these write errors will lead to an
 *	error in a following read instead.
 */


/*****************************************************************************
 * frame buffer device							     *
 *	additional ioctl						     *
 *****************************************************************************/

#define CPAD_FRAMERATE		_IOW('U', CPAD_IOCTL_BASE, unsigned char)
						/* set framerate */
#define CPAD_DITHER		_IOW('U', CPAD_IOCTL_BASE+1, unsigned char)
						/* set dithering mode */
#define CPAD_INVERT		_IOW('U', CPAD_IOCTL_BASE+2, unsigned char)
						/* invert screen */
#define CPAD_BRIGHTNESS		_IOW('U', CPAD_IOCTL_BASE+3, unsigned int)
						/* set brightness */
#define CPAD_ONLYCHANGED	_IOW('U', CPAD_IOCTL_BASE+4, unsigned char)
						/* draw only changed pixels */
#define CPAD_ONLYVISIBLE	_IOW('U', CPAD_IOCTL_BASE+6, unsigned char)
						/* send only visible pixels */
#define CPAD_ONEFRAME		_IO('U', CPAD_IOCTL_BASE+5)
						/* draw only one frame */

#endif /* _LINUX_CPAD_H */
