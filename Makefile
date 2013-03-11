# Cross compiler makefile for FIFO DMA example
KERN_SRC=~/PROJECTS/ACQ400/linux-xlnx
obj-m := acq420FMC_drv.o

all:
	make -C $(KERN_SRC) ARCH=arm M=`pwd` modules

clean:
	@rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions
