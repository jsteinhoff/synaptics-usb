KVERSION 	:= `uname -r`
INSTDIR		:= /lib/modules/$(KVERSION)/kernel/drivers/usb/input

obj-m 		:= cpad.o

all:
	$(MAKE) modules -C /lib/modules/$(KVERSION)/build SUBDIRS=`pwd`

.PHONY: clean

clean:
	$(RM) -f *.o *.ko *.mod.c .*.cmd

install:
	install -m 644 -c cpad.ko $(INSTDIR)
