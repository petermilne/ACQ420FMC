/*
 * acq480_knobs.cpp
 *
 *  Created on: 15 Feb 2015
 *      Author: pgm
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <vector>
using namespace std;

#include "ads5294.h"

struct Command;

class Acq480FMC {

	int site;

	vector<Command*> commands;

	void init_commands();
public:
	Ads5294 chip;

	Acq480FMC(int _site) : site(_site) {
		char fname[80];
		snprintf(fname, 80, "/dev/acq480.%d", site);
		FILE* fp = fopen(fname, "r+");
		if (!fp){
			perror(fname);
			exit(1);
		}
		chip.regs = new Ads5294Regs;
		chip.regs->regs = static_cast<Reg *>(
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
		for (VCI it = module.commands.begin(); it != module.commands.end(); ++it){
			printf("%s\n", (*it)->cmd);
		}
	}
};

class DumpCommand: public Command {
public:
	DumpCommand() : Command("acq480_dump") {}

	int operator() (class Acq480FMC module, int argc, char* argv[]) {
		FILE *fp = popen("hexdump -ve \'16/2 \"%04x \" \"\n\"\'", "w");
		fwrite(module.regs(), sizeof(short), NREGS, fp);
		pclose(fp);
	}
};

class SetGainCommand: public Command {
public:
	SetGainCommand() : Command("acq480_setGain") {}

	int operator() (class Acq480FMC module, int argc, char* argv[]) {
		if (argc < 3) die("acq480_setGain CHAN GAIN");

		int rc = module.chip.setGain(
				static_cast<Ads5294::Chan>(atoi(argv[1])),
				static_cast<Ads5294::Gain>(atoi(argv[2])));
		if (rc != 0) die("setGain failed");
		return 0;
	}
};

class GetGainCommand: public Command {
public:
	GetGainCommand() : Command("acq480_getGain") {}

	int operator() (class Acq480FMC module, int argc, char* argv[]) {
		if (argc < 2) die("acq480_getGain CHAN");
		printf("%d\n", module.chip.getGain(
				static_cast<Ads5294::Chan>(atoi(argv[1]))));
		return 0;
	}

};
void Acq480FMC::init_commands()
{
	commands.push_back(new SetGainCommand);
	commands.push_back(new GetGainCommand);
	commands.push_back(new DumpCommand);
	commands.push_back(new HelpCommand);
}

int  Acq480FMC::operator() (int argc, char* argv[])
{
	char** arg0 = argv;
	if (strcmp(argv[0], "acq480_knobs") == 0){
		arg0 = &argv[1];
		argc--;
	}

	for (VCI it = commands.begin(); it != commands.end(); ++it){
		if (strcmp(arg0[0], (*it)->cmd) == 0){
			(*(*it))(*this, argc, arg0);
		}
	}
	return 0;
}
int main(int argc, char* argv[])
{
	Acq480FMC module(1);

	return module(argc, argv);
}
