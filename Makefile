# Cross compiler makefile for FIFO DMA example
KERN_SRC=~/PROJECTS/ACQ400/linux-xlnx
obj-m := acq420FMC.o


acq420FMC-objs := acq420FMC_drv.o acq420_sysfs.o acq420_proc.o hbm.o

all: modules apps
	

apps: mmap acq400_stream

modules:
	make -C $(KERN_SRC) ARCH=arm M=`pwd` modules
	
clean:
	@rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions mmap acq400_stream
	
mmap: mmap.o
	$(CC) -o mmap mmap.o -L../lib -lpopt
	
acq400_stream: acq400_stream.o
	$(CXX) -o acq400_stream acq400_stream.o -L../lib -lpopt
	
zynq:
	./make.zynq
		
