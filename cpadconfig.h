/*
 * Check kernel config
 */

#if !defined(CONFIG_USB) && !defined(CONFIG_USB_MODULE)
# error : kernel has no USB support
#endif

#define CONFIG_USB_CPAD_MODULE 1

/* support for cPad input device */
#if defined(CONFIG_INPUT) || defined(CONFIG_INPUT_MODULE)
# define CONFIG_USB_CPADINPUT 1
#else
# undef CONFIG_USB_CPADINPUT
#endif

/* support for /dev/usb/cpad* device */
#define CONFIG_USB_CPADDEV 1

/* support for /proc/driver/cpad/cpad* */
#ifdef CONFIG_PROC_FS
# define CONFIG_USB_CPADPROCFS 1
#else
# undef CONFIG_USB_CPADPROCFS
#endif

/* support for /dev/fb* */
#ifdef CONFIG_FB
# define CONFIG_USB_CPADFB 1
#else
# undef CONFIG_USB_CPADFB
#endif

/* uncomment to disable some features */
/*#undef CONFIG_USB_CPADINPUT*/
/*#undef CONFIG_USB_CPADDEV*/
/*#undef CONFIG_USB_CPADPROCFS*/
/*#undef CONFIG_USB_CPADFB*/
