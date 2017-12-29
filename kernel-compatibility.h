#include <linux/version.h>

#include "kconfig.h"

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)

#define usb_lock_device(dev) down(&dev->serialize)
#define usb_unlock_device(dev) up(&dev->serialize)

static int driver_probe_device(struct device_driver * drv, struct device * dev)
{
	if (drv->bus->match && !drv->bus->match(dev, drv))
		return -ENODEV;

	dev->driver = drv;
	if (drv->probe) {
		int error = drv->probe(dev);
		if (error) {
			dev->driver = NULL;
			return error;
		}
	}

	device_bind_driver(dev);
	return 0;
}

#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10) */


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)

#define usb_kill_urb(urb) usb_unlink_urb(urb)

#endif /*LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8) */


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,5)

#include <linux/types.h>
#include <asm/atomic.h>

struct kref {
	atomic_t refcount;
};

static void kref_init(struct kref *kref)
{
	atomic_set(&kref->refcount,1);
}

static void kref_get(struct kref *kref)
{
	WARN_ON(!atomic_read(&kref->refcount));
	atomic_inc(&kref->refcount);
}

static void kref_put(struct kref *kref, void (*release) (struct kref *kref))
{
	WARN_ON(release == NULL);
	WARN_ON(release == (void (*)(struct kref *))kfree);

	if (atomic_dec_and_test(&kref->refcount))
		release(kref);
}

#define cur_altsetting altsetting

#else /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,5) */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9)

#define kref_init(kref) kref_init(kref, cpad_delete)
#define kref_put(kref, release) kref_put(kref)

#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,5) */
