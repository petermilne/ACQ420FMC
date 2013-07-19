# Cross compiler makefile for FIFO DMA example
KERN_SRC=~/PROJECTS/ACQ400/linux-xlnx
obj-m += acq420FMC.o
obj-m += dmatest_pgm.o
obj-m += debugfs2.o
obj-m += pl330_fs.o

DC=$(shell date +%y%m%d%H%M%S)
SEQ=10

acq420FMC-objs := acq420FMC_drv.o acq420_sysfs.o acq420_proc.o hbm.o zynq-timer.o

dmatest_pgm-objs := dmatest.o zynq-timer.o


all: modules apps
	
date:
	echo $(DC)

package: all
	cp mmap acq400_stream opkg/usr/local/bin
	cp *.ko opkg/usr/local/lib/modules
	tar cvzf release/$(SEQ)-acq420-$(DC).tgz -C opkg .

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
		
