obj-m += ccu_spi.o

PWD := $(shell pwd)

DESTDIR ?= $(INSTALL_MOD_PATH)

modules all:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules

modules_install install: all
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules_install INSTALL_MOD_PATH=$(DESTDIR)
	@echo Copying ccu-spi headers to toolchain
	mkdir -p $(DESTDIR)/usr/include/linux
	cp -f ccu_spi.h $(DESTDIR)/usr/include/linux/

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean

