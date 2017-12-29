KVERSION	:= `uname -r`
DVERSION	:= 1.4rc2
KSRC		:= /lib/modules/$(KVERSION)/build
INSTDIR		:= /lib/modules/$(KVERSION)/kernel/drivers/usb/input

obj-m		:= synaptics-usb.o

all:
	$(MAKE) modules -C $(KSRC) SUBDIRS=`pwd` CPATH=`pwd`

.PHONY: clean patchfile

patchfile:
	$(MAKE) KSRC=$(KSRC) KVERSION=$(KVERSION) DVERSION=$(DVERSION) -C kpatch

patch-kernel:	patchfile
	kpatch/patch-kernel $(KSRC) $(KVERSION) $(DVERSION)

clean:
	$(RM) -fr *.o *.ko *.mod.c .*.cmd .tmp_versions *~ linux/*~
	$(MAKE) clean -C kpatch

mrproper:	clean
	$(MAKE) mrproper -C kpatch

install:
	install -m 644 synaptics-usb.ko $(INSTDIR)
	depmod -a
