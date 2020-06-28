#
# ============================================================================
# Copyright (c) 2015 Marvell International, Ltd. All Rights Reserved
#
# Marvell Confidential
# ============================================================================
#

obj-m := i2c-test.o

SRC := $(shell pwd)

all:
	echo "i2c-test build"
	echo $(KERNEL_SRC)
	#$(MAKE) -C $(KERNEL_SRC) M=$(SRC)
	make -C $(KERNEL_SRC) M=$(PWD) modules
    
modules_install:
	echo "install i2c-test"
	echo $(KERNEL_SRC) 
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers


