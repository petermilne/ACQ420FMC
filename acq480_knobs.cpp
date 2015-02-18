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
#include <libgen.h>
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
		Reg *r0 = module.regs();
		for (int ii = 0; ii < NREGS; ++ii){
			if (ii%16 == 0){
				printf("%02x:", ii);
			}
			printf("%04x%c", r0[ii], (ii+1)%16==0? '\n': ' ');
		}
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

class SetDecimationFilterCommand: public Command {
public:
	SetDecimationFilterCommand() : Command("acq480_setDecimationFilter") {}

	int operator() (class Acq480FMC module, int argc, char* argv[]) {
		if (argc < 3) die("acq480_setDecimationFilter CHAN FILTER [odd]");
		int rc = module.chip.setDecimationFilter(
				static_cast<Ads5294::Chan>(atoi(argv[1])),
				static_cast<Ads5294::Filter>(atoi(argv[2])),
				argc<4? false: atoi(argv[3]));
		if (rc != 0) die("setGain failed");
		return 0;
	}
};

class SetFilterCoefficientsCommand: public Command {
	static void setCoeffs(short coeffs[], int argc, char** argv) {
		if (argc > NTAPS) argc = NTAPS;
		for (int ii = 0; ii < argc; ++ii){
			coeffs[ii] = strtol(argv[ii], 0, 0);
		}
	}
public:
	SetFilterCoefficientsCommand() :
		Command("acq480_setFilterCoefficients") {}

	int operator() (class Acq480FMC module, int argc, char* argv[]) {
		if (argc < 2) die("acq480_setFilterCoefficients CHAN [coeff [coeff]");
		short coeffs[NTAPS] = {};
		if (argc > 2){
			setCoeffs(coeffs, argc-2, argv+2);
		}
		int rc = module.chip.setCustomCoefficients(
			static_cast<Ads5294::Chan>(atoi(argv[1])),
			argc > 2? coeffs: 0);
		return rc;
	}
};

class SetHiPassFilterCommand: public Command {
public:
	SetHiPassFilterCommand() :
		Command("acq480_setHiPassFilter") {}
	int operator() (class Acq480FMC module, int argc, char* argv[]) {
		if (argc < 2) die("acq480_setHiPassFilter CHAN [enable hpfun]");

		int rc = module.chip.setHiPassFilter(
				static_cast<Ads5294::Chan>(atoi(argv[1])),
				argc > 2? atoi(argv[2]): false,
				argc > 3? strtoul(argv[3], 0, 0): 0
				);
		return rc;
	}
};

class SetDataRateCommand: public Command {
public:
	SetDataRateCommand() :
		Command("acq480_setDataRate") {}
	int operator() (class Acq480FMC module, int argc, char* argv[]) {
		if (argc < 2) die("acq480_setDataRate RATE");
		int rc = module.chip.setDataRate(
				static_cast<Ads5294::DataRate>(atoi(argv[1])));
	}
};

class SetAverageSelectCommand: public Command {
public:
	SetAverageSelectCommand() :
		Command("acq480_setAverageSelect") {}
	int operator() (class Acq480FMC module, int argc, char* argv[]) {
		if (argc < 2) die("acq480_setAverageSelect CHAN [ENABLE RATE]");
		int rc = module.chip.setAverageSelect(
			static_cast<Ads5294::Chan>(atoi(argv[1])),
			argc>2? atoi(argv[2]): false,
			argc>3? strtoul(argv[3], 0, 0): 0
		);
	}
};

class SetInvertCommand: public Command {
public:
	SetInvertCommand() :
		Command("acq480_SetInvert") {}
	int operator() (class Acq480FMC module, int argc, char* argv[]) {
		if (argc < 2) die("acq480_SetInvert CHAN [disable]");
		int rc = module.chip.setInvert(
			static_cast<Ads5294::Chan>(atoi(argv[1])),
			argc>2? atoi(argv[2]): true
		);
	}
};
class SetLFNSCommand: public Command {
public:
	SetLFNSCommand() :
		Command("acq480_SetLFNS") {}
	int operator() (class Acq480FMC module, int argc, char* argv[]) {
		if (argc < 2) die("acq480_SetLFNS CHAN [disable]");
		int rc = module.chip.setLFNS(
			static_cast<Ads5294::Chan>(atoi(argv[1])),
			argc>2? atoi(argv[2]): true
		);
	}
};

void Acq480FMC::init_commands()
{
	commands.push_back(new SetInvertCommand);
	commands.push_back(new SetLFNSCommand);
	commands.push_back(new SetAverageSelectCommand);
	commands.push_back(new SetDataRateCommand);
	commands.push_back(new SetHiPassFilterCommand);
	commands.push_back(new SetFilterCoefficientsCommand);
	commands.push_back(new SetDecimationFilterCommand);
	commands.push_back(new SetGainCommand);
	commands.push_back(new GetGainCommand);
	commands.push_back(new DumpCommand);
	commands.push_back(new HelpCommand);
}

int  Acq480FMC::operator() (int argc, char* argv[])
{
	char** arg0 = argv;
	char* verb = basename(argv[0]);

	if (strcmp(verb, "acq480_knobs") == 0){
		arg0 = &argv[1];
		verb = arg0[0];
		argc--;
	}
	if (argc == 0){
		printf("usage: acq480_knobs command [acq480_help]\n");
		return 0;
	}

	for (VCI it = commands.begin(); it != commands.end(); ++it){
		Command &command = *(*it);
		if (strcmp(verb, command.cmd) == 0){
			command(*this, argc, arg0);
		}
	}
	return 0;
}
int main(int argc, char* argv[])
{
	Acq480FMC module(1);

	return module(argc, argv);
}
