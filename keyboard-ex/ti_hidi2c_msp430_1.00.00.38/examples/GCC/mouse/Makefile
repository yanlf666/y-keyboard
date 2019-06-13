include Makefile.inc
SRCS = main.c
OBJS=$(SRCS:.c=.o)

BUILDDIR=$(shell pwd)
OUTPUTDIR=$(BUILDDIR)/Output
CFLAGS=-I$(BUILDDIR) -I$(BUILDDIR)/hidi2c -I$(BUILDDIR)/hidi2c/hid/ -I$(BUILDDIR)/hidi2c/i2c/ -I$(BUILDDIR)/hidi2c/i2c/usci/

SRCS = main.c
TIMAKEFLAGS = 'BUILDDIR=$(BUILDDIR)' 'CFLAGS=$(CFLAGS)' 'CC=$(CC)' 'DEVICE=$(DEVICE)'

all: init $(OBJS) hidi2c 

init:
	mkdir -p $(OUTPUTDIR)
	
hidi2c:
	cd hidi2c; make $(TIMAKEFLAGS);
	$(CC) $(CFLAGS) -O2 -mmcu=$(DEVICE) *.o -o $(OUTPUTDIR)/mouse_example.elf
	
%.o : %.c
	$(CC) $(CFLAGS) -c -O2 -mmcu=$(DEVICE) $< -o $(BUILDDIR)/$@	
	
clean:
	rm -rf $(OUTPUTDIR)
	rm -rf *.o
	
.PHONY: clean hidi2c
