/*
 * Check kernel config
 * this is will be handled by a Kconfig file later
 */

#ifndef _SYNUSB_KCONFIG_H
#define _SYNUSB_KCONFIG_H

#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)
#include <linux/config.h>
#endif

/* support for /dev/usb/cpad* device */
/* comment the following line to disable it */
#define CONFIG_MOUSE_SYNAPTICS_CPADDEV

/* enable debugging messages */
//#define DEBUG

#if !defined(CONFIG_USB) && !defined(CONFIG_USB_MODULE)
#error : kernel has no USB support. Compile kernel with CONFIG_USB.
#endif
#if !defined(CONFIG_INPUT) && !defined(CONFIG_INPUT_MODULE)
#error : No input support. Compile kernel with CONFIG_INPUT to enable it.
#endif

#if !defined(CONFIG_INPUT_EVDEV) && !defined(CONFIG_INPUT_EVDEV_MODULE)
#warning : Synaptics TouchPad driver for XFree86 needs CONFIG_INPUT_EVDEV.
#endif

#endif /* _SYNUSB_KCONFIG_H */
