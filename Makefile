KVERSION	:= `uname -r`
DVERSION	:= 1.5
KSRC		:= /lib/modules/$(KVERSION)/build
MODINSTDIR	:= /lib/modules/$(KVERSION)/kernel/drivers/input/mouse
INSTDIR		:= /usr/local/sbin

obj-m		:= synaptics-usb.o

all:
	$(MAKE) -C $(KSRC) M=`pwd` CPATH=`pwd` modules
#	$(MAKE) -C $(KSRC) SUBDIRS=`pwd` CPATH=`pwd` modules

.PHONY: all clean patchfile patch-kernel mrproper install uninstall

patchfile:
	$(MAKE) KSRC=$(KSRC) KVERSION=$(KVERSION) DVERSION=$(DVERSION) -C kpatch-helper

patch-kernel:	patchfile
	kpatch-helper/patch-kernel $(KSRC) $(KVERSION) $(DVERSION)

clean:
	$(MAKE) -C $(KSRC) M=`pwd` clean
#	$(RM) -fr *.o *.ko *.mod.c .*.cmd .tmp_versions Module.symvers Module.markers modules.order
	$(MAKE) clean -C kpatch-helper

mrproper:	clean
	$(RM) -f *~ linux/*~
	$(MAKE) mrproper -C kpatch-helper
	$(RM) -f kernel-patches/synaptics-usb-*.patch

install:	all
	install -D -m 644 synaptics-usb.ko $(MODINSTDIR)/synaptics-usb.ko
	install -D synaptics-usb $(INSTDIR)/synaptics-usb
	install -D synaptics-usb-rebind $(INSTDIR)/synaptics-usb-rebind
	depmod -a

uninstall:
	rm -f $(MODINSTDIR)/synaptics-usb.ko $(INSTDIR)/synaptics-usb-rebind $(INSTDIR)/synaptics-usb
	depmod -a
