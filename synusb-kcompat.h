#include <linux/version.h>
#include <linux/usb.h>
#include <linux/input.h>
#include <asm/byteorder.h>
#include <linux/types.h>
#include <asm/atomic.h>


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14)

static inline void *kzalloc(size_t size, unsigned int gfp)
{
	void *retval;

	retval = kmalloc(size, gfp);
	if (retval)
		memset(retval, 0x00, size);

	return retval;
}

#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14) */


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)

static inline struct input_dev *input_allocate_device(void)
{
	return kzalloc(sizeof(struct input_dev), GFP_KERNEL);
}

static inline void input_free_device(struct input_dev *dev)
{
	kfree(dev);
}

#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15) */


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)

#define usb_lock_device(dev) down(&dev->serialize)
#define usb_unlock_device(dev) up(&dev->serialize)

/*static int driver_probe_device(struct device_driver * drv, struct device * dev)
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
}*/

#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10) */

static struct usb_driver synusb_driver;
static struct usb_device_id synusb_idtable [];

static void repossess_devices( struct usb_driver *driver, struct usb_device_id *id )
{
	while( id->idVendor )
	{
		// XXX Not enough, we need to find ALL devices, not just the
		//     first of any particular model.
		struct usb_device *udev = usb_find_device(id->idVendor,id->idProduct);

		if (udev)
		{
			struct usb_interface    *interface;

			down_write( &udev->dev.bus->subsys.rwsem );
			usb_lock_device( udev );

			interface = usb_ifnum_to_if(udev, 0);

			if ( interface->dev.driver != &driver->driver )
			{
			    if (usb_interface_claimed(interface))
			    {
				    info("releasing '%s' from generic driver '%s'.",
					 interface->dev.kobj.k_name,
					 interface->dev.driver->name);
				    device_release_driver(&interface->dev);
			    }

			    driver_attach(&driver->driver);
			}
			else
			    info( "already attached" );

			usb_unlock_device( udev );
			up_write( &udev->dev.bus->subsys.rwsem );

			usb_put_dev(udev);
		}
		++id;
	}
}

static int rebind;

static int synusb_set_rebind(const char *val, struct kernel_param *kp)
{
	int retval;

	retval = param_set_int(val, kp);
	repossess_devices( &synusb_driver, synusb_idtable );
	return retval;
}

module_param_call(rebind, synusb_set_rebind, param_get_int, &rebind, 0664);

static inline void
usb_to_input_id(const struct usb_device *dev, struct input_id *id)
{
	id->bustype = BUS_USB;
	id->vendor = le16_to_cpu(dev->descriptor.idVendor);
	id->product = le16_to_cpu(dev->descriptor.idProduct);
	id->version = le16_to_cpu(dev->descriptor.bcdDevice);
}

#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13) */


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)

#define usb_kill_urb(urb) usb_unlink_urb(urb)

#endif /*LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8) */


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,7)

static inline void input_set_abs_params(struct input_dev *dev, int axis, int min, int max, int fuzz, int flat)
{
	dev->absmin[axis] = min;
	dev->absmax[axis] = max;
	dev->absfuzz[axis] = fuzz;
	dev->absflat[axis] = flat;
	dev->absbit[LONG(axis)] |= BIT(axis);
}

#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,7) */


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,5)

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

#define kref_init(kref) kref_init(kref, synusb_delete)
#define kref_put(kref, release) kref_put(kref)

#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9) */
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,5) */
