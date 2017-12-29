#ifndef _LINUX_CPAD_H
#define _LINUX_CPAD_H

#include <linux/input.h>


/*****************************************************************************
 * /dev/usb/cpad* device						     *
 *	supported operations: read, write, ioctl			     *
 *	it is compatible to version 0.7 of Rob Miller's and Ron Lee's cPad   *
 *	driver for kernel 2.4, available at:				     *
 *		http://www.janerob.com/rob/ts5100/cPad/index.shtml    	     *
 *****************************************************************************/

/* some constants */
#define USB_CPAD_MINOR_BASE	66
#define CPAD_IOCTL_BASE		0x71
#define CPAD_MAX_FLASH		1000		/* max. time for CPAD_FLASH ioctl in ms */
#define CPAD_DRIVER_NUM 	7		/* version reported by CPAD_VERSION ioctl */

/* supported ioctls
 * because the backlight has a finite lifespan (ca. 1000 - 3000 hours), the
 * CPAD_FLASH ioctl should be used instead of CPAD_WLIGHT.
 * to get the result of CPAD_RLIGHT and CPAD_RLCD, read one byte from the
 * character device.
 */
#define CPAD_VERSION		_IOR('U', CPAD_IOCTL_BASE, int)			/* get driver version */
#define CPAD_CGID		_IOR('U', CPAD_IOCTL_BASE+1, struct input_id)	/* get device ID */
#define CPAD_RESET		_IO('U', CPAD_IOCTL_BASE+2)			/* reset usb device cpad */
#define CPAD_WLIGHT		_IOW('U', CPAD_IOCTL_BASE+3, unsigned char)	/* set backlight state */
#define CPAD_WLCD		_IOW('U', CPAD_IOCTL_BASE+5, unsigned char)	/* set LCD state */
#define CPAD_RLIGHT		_IO('U', CPAD_IOCTL_BASE+6)  			/* read backlight state */
#define CPAD_RLCD		_IO('U', CPAD_IOCTL_BASE+7)  			/* read LCD state */
#define CPAD_FLASH		_IOW('U', CPAD_IOCTL_BASE+8, int)		/* flash backlight */

/* ioctls not yet supported */
#define CPAD_WIMAGEL		_IOW('U', CPAD_IOCTL_BASE+4, char[30])	/* write an image line */

/* ioctls not supported by this driver, only added for compatibility here
   these things are handled by Synaptics TouchPad driver for XFree86 now */
#define CPAD_SET_STROKE		_IOW('U', CPAD_IOCTL_BASE+9, int)	/* change mouse motion sensitivity */
#define CPAD_SET_SENS		_IOW('U', CPAD_IOCTL_BASE+15, int)	/* change tap sensitivity */
#define CPAD_SET_ABS		_IOW('U', CPAD_IOCTL_BASE+16, char)	/* change abs/rel output mode */

/*
 * writing to /dev/usb/cpad*
 *	the cPad display is controlled by a Seiko/Epson 1335 LCD Controller IC
 *	a write to the device consists of a command followed by data:
 *		<1335 command> [<data> ...]
 *	for MRWITE_1335 command, data may be up to 160*30 bytes long
 *	possible commands as reported in the sed1335 data sheet are listed below
 *
 *	<command>	<hex>	   <command description>		<No. of Bytes>	*/

/* System control */
#define SYSSET_1335	0x40	/* Initialize device and display 	8		*/
#define SLEEP_1335	0x53	/* Enter standby mode 			0		*/

/* Display control */
#define DISPOFF_1335	0x58	/* Enable and disable display and			*/
#define DISP_1335	0x59	/* display flashing			1		*/
#define SCROLL_1335	0x44	/* Set display start address and			*/
				/* display regions			10		*/
#define CSRF_1335	0x5d	/* Set cursor type			2		*/
#define CGRAMADR_1335	0x5c	/* Set start address of character			*/
				/* generator RAM			2		*/
#define CSRDIR_RIGHT_1355 0x4c	/* Set direction of cursor movement	0		*/
#define CSRDIR_LEFT_1355  0x4d	/* "					0		*/
#define CSRDIR_UP_1355	  0x4e	/* "					0		*/
#define CSRDIR_DOWN_1355  0x4f	/* "					0		*/
#define HDOTSCR_1335	0x5a	/* Set horizontal scroll position	1		*/
#define OVLAY_1335	0x5b	/* Set display overlay format		1		*/

/* Drawing control */
#define CSRW_1335	0x46	/* Set cursor address			2		*/
#define CSRR_1335	0x47	/* Read cursor address			2		*/

/* Memory control */
#define MWRITE_1335	0x42	/* Write to display memory		0-160*30	*/
#define MREAD_1335	0x43	/* Read from display memory		--		*/

/*
 * reading /dev/usb/cpad*
 *	reads answer of the sed1335 to a command, can be 0-32 bytes long.
 *	the information is only accessible until the next write access.
 *	because writes are done asyncronously, some write errors can occure
 *	after write returned successful. these write errors will lead to an
 *	error in a following read instead.
 */


/*****************************************************************************
 *	internals							     *
 *****************************************************************************/

/*
 * How the bulk endpoints work:
 *
 * in order to send commands to the sed1335, each packet in the urb must look
 * like this:
 *	02 <1335 command> [<data in reversed order> ...]
 * the urb must be send to the bulk out endpoint. because the packet size of
 * this endoint is 32 bytes, "02 <1335 command>" must be repeated after 30
 * bytes of data. the data must be in reversed order in each of this 30 bytes
 * block. all this is done automatically in cpad_dev_write.
 *
 * functions that are not controlled by the sed1335, like the backlight, can be
 * accessed by
 *	01 <function> <state>
 * the packet must be send to the bulk endpoint. these functions can only be
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

#endif /* _LINUX_CPAD_H */
