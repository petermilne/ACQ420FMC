# Cross compiler makefile for AO421ELF


DC=$(shell date +%y%m%d%H%M%S)
SEQ=66


APPS := dawg

all: apps
	
date:
	echo $(DC)



apps: $(APPS)


clean:
	@rm -rf *.o $(APPS)
	
dawg: dawg.o
	$(CXX) -O3 -o dawg dawg.o -L../lib -lpopt

zynq:
	./make.zynq
		
