/*
 * Check kernel config
 * this is will be handled by a Kconfig file later
 */


#if !defined(CONFIG_USB) && !defined(CONFIG_USB_MODULE)
#error : kernel has no USB support. Compile kernel with CONFIG_USB.
#endif
#if !defined(CONFIG_INPUT) && !defined(CONFIG_INPUT_MODULE)
#error : No input support. Compile kernel with CONFIG_INPUT to enable it.
#endif

#if !defined(CONFIG_INPUT_EVDEV) && !defined(CONFIG_INPUT_EVDEV_MODULE)
#warning : Synaptics TouchPad driver for XFree86 needs CONFIG_INPUT_EVDEV.
#endif

/* support for /dev/usb/cpad* device */
#define CONFIG_USB_CPADDEV
/* uncomment the following to disable it */
//#undef CONFIG_USB_CPADDEV

/* enable debugging messages */
//#define DEBUG 1

#undef dbg
#ifdef DEBUG
#define dbg(format, arg...) printk(KERN_DEBUG "%s: " format "\n" , \
	__FILE__ , ## arg)
#else
#define dbg(format, arg...) do {} while (0)
#endif
