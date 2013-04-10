#
# Makefile for mcp2210
#

CONFIG += -DCONFIG_MCP2210_MODULE=1 -DCONFIG_MCP2210_SPI=1 \
	  -DCONFIG_MCP2210_GPIO=1 -DCONFIG_MCP2210_EEPROM=1 \
	  -DCONFIG_MCP2210_DEBUG=1 -DCONFIG_MCP2210_DEBUG_VERBOSE=1 \
	  -DCONFIG_MCP2210_CREEK=1 \
	  -DCONFIG_MCP2210_DEBUG_INITIAL=9

EXTRA_CFLAGS += -Werror -Wall -g -Wunused-macros $(CONFIG)

# To build modules outside of the kernel tree, we run "make"
# in the kernel source tree; the Makefile these then includes this
# Makefile once again.
# This conditional selects whether we are being included from the
# kernel Makefile or not.
ifeq ($(KERNELRELEASE),)

    # Assume the source tree is where the running kernel was built
    # You should set KERNELDIR in the environment if it's elsewhere
    KERNELDIR ?= /lib/modules/$(shell uname -r)/build
    # The current directory is passed to sub-makes as argument
    PWD := $(shell pwd)

all: modules user

user:
	$(MAKE) -C user

modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c mcp2210.s .tmp_versions \
	       modules.order Module.symvers
	$(MAKE) -C user clean

mcp2210.s: modules
	$(CROSS_COMPILE)objdump -dSr --prefix-addresses --line-numbers \
							mcp2210.ko > mcp2210.s

.PHONY: modules modules_install user clean

else
    # called from kernel build system: just declare what our modules are
    CONFIG_MCP2210 ?= m
    mcp2210-objs := mcp2210-core.o mcp2210-ioctl.o mcp2210-ctl.o \
		    mcp2210-spi.o mcp2210-eeprom.o mcp2210-lib.o

    obj-$(CONFIG_MCP2210)		+= mcp2210.o
endif

