/*
 * acq480_knobs.cpp
 *
 *  Created on: 15 Feb 2015
 *      Author: pgm
 */


#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <vector>
using namespace std;

#include "ads5294.h"

struct Command;

class Acq480FMC {
	Ads5294 chip;
	int site;

	vector<Command*> commands;

	void init_commands();
public:
	Acq480FMC(int _site) : site(_site) {
		char fname[80];
		snprintf(fname, 80, "/dev/acq480.%d", site);
		FILE* fp = fopen(fname, "r+");
		if (!fp){
			perror(fname);
			exit(1);
		}
		chip.regs = static_cast<struct Ads5294Regs *>(
				mmap(NULL, ADS5294_SPI_MIRROR_SZ,
						PROT_READ|PROT_WRITE,
						MAP_SHARED,
						fileno(fp), 0));
		init_commands();
	}

	int operator() (int argc, char* argv[]);

	Reg* regs() {
		return chip.regs->regs;
	}
	friend class HelpCommand;
};

struct Command {
	const char* cmd;
	virtual int operator() (class Acq480FMC module, int argc, char* argv[]) = 0;

	Command(const char* _cmd) : cmd(_cmd) {}
};
typedef vector<Command*>::iterator VCI;

class HelpCommand: public Command {
public:
	HelpCommand() : Command("acq480_help") {}

	int operator() (class Acq480FMC module, int argc, char* argv[]) {
		for (VCI it = module.commands.begin(); it != module.commands.begin(); ++it){
			printf("%s\n", (*it)->cmd);
		}
	}
};

class DumpCommand: public Command {
public:
	DumpCommand() : Command("acq480_dump") {}

	int operator() (class Acq480FMC module, int argc, char* argv[]) {
		FILE *fp = popen("hexdump -e \'\"%02_ax:\" 16/2 \"%02x \"\'", "w");
		fwrite(module.regs(), sizeof(short), NREGS, fp);
		pclose(fp);
	}
};
void Acq480FMC::init_commands()
{
	commands.push_back(new DumpCommand());
	commands.push_back(new HelpCommand());
}

int  Acq480FMC::operator() (int argc, char* argv[])
{

	return 0;
}
int main(int argc, char* argv[])
{
	Acq480FMC module(1);

	return module(argc, argv);
}
