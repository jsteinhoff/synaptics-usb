KVERSION 	:= `uname -r`
INSTDIR		:= /lib/modules/$(KVERSION)/kernel/drivers/usb/input

obj-m 		:= cpad.o

all:
	$(MAKE) modules -C /lib/modules/$(KVERSION)/build SUBDIRS=`pwd`

.PHONY: clean

clean:
	$(RM) -fr *.o *.ko *.mod.c .*.cmd .tmp_versions *~

install:
	install -m 644 cpad.ko $(INSTDIR)
