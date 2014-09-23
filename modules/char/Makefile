KERNELDIR ?= /home/jf/work/git/exynos4412/kernel/linux-3.14
NAME := char
PWD := $(shell pwd)
obj-m += $(NAME).o
modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
.PHONY: clean
clean:
	@rm -rf *.o *.ko *mod* Module.symvers .tmp* .*.cmd 
	
