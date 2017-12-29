#ifndef _CPAD_H
#define _CPAD_H

#include "kconfig.h"

#ifdef CONFIG_USB_CPADDEV

#include <linux/usb.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/workqueue.h>

#include "synapticsusb.h"

#define USB_CPAD_MINOR_BASE	192

/* packet size of in and out bulk endpoints */
#define CPAD_PACKET_SIZE	32
/* a packet consists of a command and optional parameters */
#define CPAD_COMMAND_SIZE	2
#define CPAD_MAX_PARAM_SIZE	(CPAD_PACKET_SIZE-CPAD_COMMAND_SIZE)
/* One packet (32 bytes) is needed to write 30 bytes to the display RAM.
 * The cPad has 8k (=30*273+2) bytes of RAM, so the maximum useful number
 * of packets in an URB is 274. Notice that the transfer buffer is re-used. */
#define CPAD_MAX_TRANSFER	(274*CPAD_PACKET_SIZE)

/* possible values for the first byte of a packet */
#define SEL_CPAD	0x01	/* select non-lcd-controller function */
#define SEL_1335	0x02	/* run sed1335 command, see linux/cpad.h */

/* observed non-lcd-controller functions */
#define CPAD_W_ROM	0x01	/* write EEPROM, not supported by this driver */
#define CPAD_R_ROM	0x02	/* read EEPROM */
#define CPAD_W_LIGHT	0x03	/* write backlight state (on/off) */
#define CPAD_R_LIGHT	0x04	/* read backlight state (on/off) */
#define CPAD_W_LCD	0x05	/* write lcd state (on/off) */
#define CPAD_R_LCD	0x06	/* read lcd state (on/off) */
#define CPAD_RSRVD	0x07

/* struct syndisplay uses its parents kref */
struct syndisplay {
	struct synusb *		parent;

	struct mutex		open_mutex;	/* synchronize open/close against disconnect*/
	int			open_count;	/* locked by open_close_mutex */

	struct mutex		io_mutex;	/* synchronize io, locks all except open_count */
	struct completion	done;		/* wait for callbacks */
	int			error;		/* report error in callback */
	struct cpad_urb	*	user_urb;	/* urb initiated by user, is re-used */
	struct cpad_urb	*	nlcd_urb;	/* used by the driver, e.g., to switch off the backlight */
	struct delayed_work	flash;		/* flash backlight */
	__u8			in_endpointAddr;
	__u8			out_endpointAddr;

	/* status of the device */
	unsigned int		gone		:1;
	unsigned int		has_indata	:1;
	unsigned int		reset_notify	:1;
	unsigned int		reset_in_progress :1;
	unsigned int		suspended	:1;
	unsigned int		backlight_on	:1;
	unsigned int				:0;
};

#define CPAD_USER_URB	0	/* use syndisplay.user_urb */
#define CPAD_NLCD_URB	1	/* use syndisplay.nlcd_urb */

struct cpad_urb {
	struct syndisplay *	display;
	struct urb		*in, *out;
};

int cpad_alloc(struct synusb *);
void cpad_free(struct synusb *);
int cpad_setup_in(struct synusb *, struct usb_endpoint_descriptor *);
int cpad_setup_out(struct synusb *, struct usb_endpoint_descriptor *);
int cpad_check_setup(struct synusb *);
int cpad_init(struct synusb *);
void cpad_disconnect(struct synusb *);

int cpad_suspend(struct synusb *);
int cpad_resume(struct synusb *);
int cpad_reset_resume(struct synusb *);
int cpad_pre_reset(struct synusb *);
int cpad_post_reset(struct synusb *);

#else /* CONFIG_USB_CPADDEV */

#include "synapticsusb.h"

struct syndisplay { };

static inline int cpad_alloc(struct synusb *synusb) { return 0; }
void cpad_free(struct synusb *synusb) { }
static inline int cpad_setup_in(struct synusb *synusb,
	struct usb_endpoint_descriptor *endpoint) { return 0; }
static inline int cpad_setup_out(struct synusb *synusb,
	struct usb_endpoint_descriptor *endpoint) { return 0; }
static inline int cpad_check_setup(struct synusb *synusb) { return 0; }
static inline int cpad_init(struct synusb *synusb) { return 0; }
static inline void cpad_disconnect(struct synusb *synusb) { }

int cpad_suspend(struct synusb *synusb) { return 0; }
int cpad_resume(struct synusb *synusb) { return 0; }
int cpad_reset_resume(struct synusb *synusb) { return 0; }
int cpad_pre_reset(struct synusb *synusb) { return 0; }
int cpad_post_reset(struct synusb *synusb) { return 0; }

#endif /* CONFIG_USB_CPADDEV */

#endif /* _CPAD_H */
