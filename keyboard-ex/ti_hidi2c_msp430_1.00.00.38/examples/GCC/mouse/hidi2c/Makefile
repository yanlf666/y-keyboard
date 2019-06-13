SUBDIRS = hid i2c

all:
	@for i in $(SUBDIRS); do \
	(cd $$i; make $(TIMAKEFLAGS)); done
