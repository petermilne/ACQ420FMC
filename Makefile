# Cross compiler makefile for FIFO DMA example
KERN_SRC=~/PROJECTS/ACQ400/linux-xlnx
obj-m += acq420fmc.o
obj-m += dmatest_pgm.o
obj-m += debugfs2.o
obj-m += pl330_fs.o
obj-m += pl330.o

DC=$(shell date +%y%m%d%H%M%S)
SEQ=10

acq420fmc-objs := acq400_drv.o acq400_sysfs.o acq400_proc.o hbm.o zynq-timer.o

dmatest_pgm-objs := dmatest.o zynq-timer.o


all: modules apps
	
date:
	echo $(DC)

package: all
	cp mmap acq400_stream monitorregs bin/* opkg/usr/local/bin
	cp *.ko opkg/usr/local/lib/modules
	tar czf release/$(SEQ)-acq420-$(DC).tgz -C opkg .
	@echo created package release/$(SEQ)-acq420-$(DC).tgz
	rm -f ../PACKAGES/$(SEQ)-acq420*
	cp release/$(SEQ)-acq420-$(DC).tgz ../PACKAGES/


apps: mmap acq400_stream bigmac

modules:
	make -C $(KERN_SRC) ARCH=arm M=`pwd` modules
	
clean:
	@rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions mmap acq400_stream
	
mmap: mmap.o
	$(CC) -o mmap mmap.o -L../lib -lpopt
	
acq400_stream: acq400_stream.o
	$(CXX) -O3 -o acq400_stream acq400_stream.o -L../lib -lpopt

bigmac: bigmac.o
	$(CXX) -mcpu=cortex-a9 -mfloat-abi=softfp -mfpu=neon \
		-DHASNEON \
		-O3 -o bigmac bigmac.o -L../lib -lpopt

bigmac.x86: bigmac.o
	$(CXX) 	-O3 -o bigmac bigmac.o -L../lib -lpopt	

		
zynq:
	./make.zynq
		
