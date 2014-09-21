# Cross compiler makefile for FIFO DMA example
KERN_SRC=~/PROJECTS/ACQ400_314/linux-xlnx
#KERN_SRC=~/PROJECTS/ACQ400/linux-xlnx-github/linux-xlnx
obj-m += acq420fmc.o
obj-m += dmatest_pgm.o
obj-m += debugfs2.o
obj-m += pl330_fs.o
obj-m += pl330.o
#obj-m += bolo8_drv.o
obj-m += acq425_drv.o
obj-m += lwnfs.o
#obj-m += acq4xx_fs.o

DC=$(shell date +%y%m%d%H%M%S)
SEQ=10

CPPFLAGS += -O3

acq420fmc-objs := acq400_drv.o  acq400_fs.o \
	acq400_debugfs.o acq400_sysfs.o acq400_lists.o \
	acq400_proc.o hbm.o zynq-timer.o \
	bolo8_core.o dio432_drv.o

dmatest_pgm-objs := dmatest.o zynq-timer.o

APPS := mmap acq400_stream bigmac permute acq435_decode \
	acq400_knobs udp_client is_ramp mmaptest wavegen \
	dsp_coprocessor

all: modules apps
	
date:
	echo $(DC)

package: all
	echo do NOT rm -Rf opkg/*
	mkdir -p opkg/usr/local/bin opkg/usr/local/lib/modules \
		opkg/usr/share opkg/usr/local/CARE \
		opkg/usr/local/init/pl330dbg opkg/usr/local/cal \
		opkg/etc/profile.d 
	cp cal/* opkg/usr/local/cal
	cp -a $(APPS) monitorregs scripts/* opkg/usr/local/bin
	cp *.ko opkg/usr/local/lib/modules
	cp -a doc opkg/usr/share
	cp bos.sh opkg/etc/profile.d
	cp acq435_decode CARE/* opkg/usr/local/CARE
	cp init/* opkg/usr/local/init
	cp pl330dbg/* opkg/usr/local/init/pl330dbg
	tar czf release/$(SEQ)-acq420-$(DC).tgz -C opkg .
	@echo created package release/$(SEQ)-acq420-$(DC).tgz
	rm -f ../PACKAGES/$(SEQ)-acq420*
	cp release/$(SEQ)-acq420-$(DC).tgz ../PACKAGES/

tarball:
	echo Please make sure directory is clean first ..
	tar cvzf acq420fmc-$(DC).tgz --exclude=release/* *


apps: $(APPS)

modules:
	make -C $(KERN_SRC) ARCH=arm M=`pwd` modules
	
clean:
	@rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions $(APPS)
	
mmap: mmap.o
	$(CC) -o $@ $^ -L../lib -lpopt

dsp_coprocessor: dsp_coprocessor.o
	$(CC) -o $@ $^ -L../lib -lpopt -lrt -lm

mmaptest: mmaptest.o
	$(CC) -o $@ $^ -L../lib -lpopt
		
udp_client: udp_client.o
	$(CC) -o $@ $^ -L../lib -lpopt
	
acq400_stream: acq400_stream.o
	$(CXX) -O3 -o $@ $^ -L../lib -lpopt -lpthread -lrt

is_ramp: is_ramp.o
	$(CXX) -O3 -o $@ $^ -L../lib -lpopt
	
acq400_knobs: acq400_knobs.o tcp_server.o
	$(CXX) -O3 -o $@ $^ -L../lib -lpopt

wavegen: wavegen.o
	$(CXX) -O3 -o $@ $^ -L../lib -lpopt
	
acq435_decode: acq435_decode.o
	$(CXX) -O3 -o $@ $^ -L../lib -lpopt -lpthread
	
bigmac: bigmac.o
	$(CXX) -mcpu=cortex-a9 -mfloat-abi=softfp -mfpu=neon \
		-DHASNEON \
		-O3 -o bigmac bigmac.o -L../lib -lpopt

bigmac.x86: bigmac.o
	$(CXX) -O3 -o $@ $^ -lpopt	

		
zynq:
	./make.zynq
		
