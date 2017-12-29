KVERSION	:= `uname -r`
DVERSION	:= 1.4rc7
KSRC		:= /lib/modules/$(KVERSION)/build
MODINSTDIR	:= /lib/modules/$(KVERSION)/kernel/drivers/usb/input
INSTDIR		:= /usr/local/sbin

obj-m		:= synaptics-usb.o

all:
	$(MAKE) modules -C $(KSRC) SUBDIRS=`pwd` CPATH=`pwd`

.PHONY: clean patchfile patch-kernel mrproper install

patchfile:
	$(MAKE) KSRC=$(KSRC) KVERSION=$(KVERSION) DVERSION=$(DVERSION) -C kpatch-helper

patch-kernel:	patchfile
	kpatch-helper/patch-kernel $(KSRC) $(KVERSION) $(DVERSION)

clean:
	$(RM) -fr *.o *.ko *.mod.c .*.cmd .tmp_versions Modules.symvers Module.symvers *~ linux/*~
	$(MAKE) clean -C kpatch-helper

mrproper:	clean
	$(MAKE) mrproper -C kpatch-helper
	$(RM) -f kernel-patches/synaptics-usb-*.patch

install:	all
	install -m 644 synaptics-usb.ko $(MODINSTDIR)
	install synaptics-usb-rebind $(INSTDIR)
	depmod -a

uninstall:
	rm -f $(MODINSTDIR)/synaptics-usb.ko $(INSTDIR)/synaptics-usb-rebind
	depmod -a
