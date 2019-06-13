SRCS = i2c_engine_usci.c i2c_engine_usci_init.c
OBJS=$(SRCS:.c=.o)

all: $(OBJS)
	
%.o : %.c
	$(CC) -c -O2 $(CFLAGS) -mmcu=$(DEVICE) $< -o $(BUILDDIR)/$@	