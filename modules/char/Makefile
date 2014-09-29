KERNELDIR ?= /home/jf/work/git/exynos4412/kernel/linux-3.14
#KERNELDIR ?= /home/jf/work/git/mtk/mt8127_jb_0516/out/target/product/em_t8611_nand/obj/KERNEL_OBJ
NAME := char
PWD := $(shell pwd)
#CROSS_TOOLS = /home/jf/work/git/mtk/mt8127_jb_0516/prebuilts/gcc/linux-x86/arm/arm-linux-androideabi-4.7/bin/arm-linux-androideabi-
OBJDUMPFLAGS = -D -S
obj-m += $(NAME).o
modules:
	@#$(MAKE) -C $(KERNELDIR) M=$(PWD) modules TARGET_PRODUCT=em_t8611_nand CROSS_COMPILE=$(CROSS_TOOLS)
	@#$(CROSS_TOOLS)objdump $(OBJDUMPFLAGS)  $(NAME).ko > $(NAME).dis
	@$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
.PHONY: clean
clean:
	@-rm -rf *.o *.ko *mod* Module.symvers .tmp* .*.cmd *.dis 
	
