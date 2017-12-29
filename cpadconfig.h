/*
 * Check kernel config
 * this is will be handled by a Kconfig file later
 */

#if !defined(CONFIG_USB) && !defined(CONFIG_USB_MODULE)
# error : kernel has no USB support. Compile kernel with CONFIG_USB.
#endif

#define CONFIG_USB_CPAD_MODULE 1

/* support for cPad input device */
#if defined(CONFIG_INPUT) || defined(CONFIG_INPUT_MODULE)
# define CONFIG_USB_CPADINPUT 1
#else
# undef CONFIG_USB_CPADINPUT
# warning : cPad input device disabled. Compile kernel with CONFIG_INPUT to enable it.
#endif
#if !defined(CONFIG_INPUT_EVDEV) && !defined(CONFIG_INPUT_EVDEV_MODULE)
# warning : Synaptics TouchPad driver for XFree86 needs CONFIG_INPUT_EVDEV.
#endif

/* support for /dev/usb/cpad* device */
#define CONFIG_USB_CPADDEV 1

/* support for /proc/driver/cpad/cpad* */
#ifdef CONFIG_PROC_FS
# define CONFIG_USB_CPADPROCFS 1
#else
# undef CONFIG_USB_CPADPROCFS
# warning : Procfs interface disabled. Compile kernel with CONFIG_PROC_FS to enable it.
#endif

/* support for /dev/fb* */
/* vesafb is needed because of the cfb_imageblit, cfb_copyarea and cfb_fillrect functions*/
#if defined(CONFIG_FB) && defined(CONFIG_FB_VESA)
# define CONFIG_USB_CPADFB 1
#else
# undef CONFIG_USB_CPADFB
# warning : Framebuffer disabled. Compile kernel with CONFIG_FB and CONFIG_FB_VESA to enable it.
#endif

/* uncomment to disable some features */
/*#undef CONFIG_USB_CPADINPUT*/
/*#undef CONFIG_USB_CPADDEV*/
/*#undef CONFIG_USB_CPADPROCFS*/
/*#undef CONFIG_USB_CPADFB*/
