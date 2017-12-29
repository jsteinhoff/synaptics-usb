all:	version_code
	./make_patchfile $(KSRC) $(KVERSION) $(DVERSION)

.PHONY: clean version.h

version.h:
	ln -sf $(KSRC)/include/linux/version.h version.h

version_code:	version.h

clean:
	$(RM) -fr version.h version_code tmp

mrproper:	clean
	$(RM) -f *~

