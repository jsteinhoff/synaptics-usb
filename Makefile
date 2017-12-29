KVERSION	:= `uname -r`
DVERSION	:= 1.5
KSRC		:= /lib/modules/$(KVERSION)/build
MODINSTDIR	:= /lib/modules/$(KVERSION)/kernel/drivers/input/mouse
INSTDIR		:= /usr/local/sbin

obj-m		:= synaptics_usb.o
synaptics_usb-objs	:= synapticsusb.o cpad.o

all:
	$(MAKE) -C $(KSRC) M=`pwd` CPATH=`pwd` modules

.PHONY: all clean patchfile patch-kernel mrproper install uninstall

patchfile:
	$(MAKE) KSRC=$(KSRC) KVERSION=$(KVERSION) DVERSION=$(DVERSION) -C kpatch-helper

patch-kernel:	patchfile
	kpatch-helper/patch-kernel $(KSRC) $(KVERSION) $(DVERSION)

clean:
	$(MAKE) -C $(KSRC) M=`pwd` clean
	$(MAKE) clean -C kpatch-helper

distclean:	clean
	$(RM) -fr *.o *.ko *.mod.c .*.cmd .tmp_versions Module.symvers Module.markers modules.order
	$(RM) -f *~ linux/*~
	$(MAKE) distclean -C kpatch-helper
	$(RM) -f kernel-patches/synaptics-usb-*.patch

install:	all
	install -D -m 644 synaptics_usb.ko $(MODINSTDIR)/synaptics_usb.ko
	install -D synaptics-usb $(INSTDIR)/synaptics-usb
	depmod -a

uninstall:
	rm -f $(MODINSTDIR)/synaptics_usb.ko $(INSTDIR)/synaptics-usb
	depmod -a
