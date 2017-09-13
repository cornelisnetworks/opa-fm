CC=gcc
CFLAGS=-D__X86_64__ -DCPU_LE -DLINUX -DIB_STACK_OPENIB -I/usr/include/opamgt

LIBS=-lopamgt \
	 -libverbs \
	 -libumad \
	 -lssl

SOURCES := $(shell ls *.c)

APPS := $(SOURCES:%.c=%)

.PHONY: all
all: $(APPS)

#default rule for all apps
% : %.c
	$(CC) $(CFLAGS) $^ -o $@ $(LIBS)

.PHONY: clean
clean:
	rm -rf $(APPS)
