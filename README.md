# Linux kernel driver for some Synaptics USB devices

This is a driver for some Synaptics USB devices for kernel 2.6 and higher.
A version of this driver
[propagated into the main linux kernel tree](http://git.kernel.org/?p=linux/kernel/git/torvalds/linux.git;a=blob;f=drivers/input/mouse/synaptics_usb.c)
and is available since kernel version 3.4-rc1; cPad display support is not yet
included there. In order to make the cPad display work, you need this driver. Follow the
[installation instructions](http://jan-steinhoff.de/linux/synaptics-usb.html).

For newer touchpads with multitouch support, have a look at
[hid-multitouch](https://github.com/torvalds/linux/blob/master/drivers/hid/hid-multitouch.c).

In order to make use of the cPad display device, you may want to have a look at:

* usr_cpad from [Rob Miller's page](http://www.janerob.com/rob/ts5100/cPad/index.shtml)
* [cpadfb](https://github.com/jsteinhoff/cpadfb)
* [cpadconsole](https://github.com/jsteinhoff/cpadconsole)
