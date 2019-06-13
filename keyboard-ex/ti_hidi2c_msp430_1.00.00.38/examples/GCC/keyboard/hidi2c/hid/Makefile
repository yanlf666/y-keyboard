SRCS = hid_reportparser.c hid_engine.c
OBJS=$(SRCS:.c=.o)

all: $(OBJS)
	
%.o : %.c
	$(CC) -c -O2 $(CFLAGS) -mmcu=$(DEVICE) $< -o $(BUILDDIR)/$@	

