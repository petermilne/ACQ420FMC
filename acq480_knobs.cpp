/*
 * acq480_knobs.cpp
 *
 *  Created on: 15 Feb 2015
 *      Author: pgm
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <vector>
#include <libgen.h>
#include <unistd.h>

using namespace std;

#include "ads5294.h"
#include "acq480_ioctl.h"

struct Command;

class Acq480FMC {

	int site;

	vector<Command*> commands;
	FILE* fp;

	void init_commands();
public:
	Ads5294 chip;

	Acq480FMC(int _site) : site(_site) {
		char fname[80];
		snprintf(fname, 80, "/dev/acq480.%d", site);
		fp = fopen(fname, "r+");
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

	~Acq480FMC() {
		flush();
		/*
		munmap(chip.regs->regs, ADS5294_SPI_MIRROR_SZ);
		printf("about to close\n");
		fclose(fp);
		printf("closed\n");
		*/
	}
	int operator() (int argc, char* argv[]);


	Reg* regs() {
		return chip.regs->regs;
	}
	int invalidate() {
		return ioctl(fileno(fp), ACQ480_CACHE_INVALIDATE);
	}
	int flush() {
		return ioctl(fileno(fp), ACQ480_CACHE_FLUSH);
	}
	int reset() {
		ioctl(fileno(fp), ACQ480_RESET);
		sleep(1);
		return ioctl(fileno(fp), ACQ480_CACHE_INVALIDATE);
	}
	friend class HelpCommand;
};

struct Command {
	const char* cmd;
	virtual int operator() (class Acq480FMC module, int argc, char* argv[]) = 0;
	/* return > 0 if flush recommended */

	Command(const char* _cmd) : cmd(_cmd) {}
};
typedef vector<Command*>::iterator VCI;

class HelpCommand: public Command {
public:
	HelpCommand() : Command("help") {}

	int operator() (class Acq480FMC module, int argc, char* argv[]) {
		for (VCI it = module.commands.begin(); it != module.commands.end(); ++it){
			printf("%s\n", (*it)->cmd);
		}
	}
};

class ResetCommand: public Command {
public:
	ResetCommand() : Command("reset") {}

	int operator() (class Acq480FMC module, int argc, char* argv[]) {
		return module.reset();
	}
};

class ReadAllCommand: public Command {
public:
	ReadAllCommand() : Command("readall") {}

	int operator() (class Acq480FMC module, int argc, char* argv[]) {
		return module.invalidate();
	}
};
class FlushCommand: public Command {
public:
	FlushCommand() : Command("flush") {}
	int operator() (class Acq480FMC module, int argc, char* argv[]) {
		return 1;
	}
};

class DumpCommand: public Command {
public:
	DumpCommand() : Command("dump") {}

	int operator() (class Acq480FMC module, int argc, char* argv[]) {
		Reg *r0 = module.regs();
		for (int ii = 0; ii < NREGS; ++ii){
			if (ii%16 == 0){
				printf("%02x:", ii);
			}
			printf("%04x%c", r0[ii], (ii+1)%16==0? '\n': ' ');
		}
		return 0;
	}
};

class SetGainCommand: public Command {
public:
	SetGainCommand() : Command("setGain") {}

	int operator() (class Acq480FMC module, int argc, char* argv[]) {
		if (argc < 3) die("setGain CHAN GAIN");

		int rc = module.chip.setGain(
				static_cast<Ads5294::Chan>(atoi(argv[1])),
				static_cast<Ads5294::Gain>(atoi(argv[2])));
		if (rc != 0) die("setGain failed");
		return 1;
	}
};

class GetGainCommand: public Command {
public:
	GetGainCommand() : Command("getGain") {}

	int operator() (class Acq480FMC module, int argc, char* argv[]) {
		if (argc < 2) die("getGain CHAN");
		printf("%d\n", module.chip.getGain(
				static_cast<Ads5294::Chan>(atoi(argv[1]))));
		return 0;
	}

};

class SetDecimationFilterCommand: public Command {
public:
	SetDecimationFilterCommand() : Command("setDecimationFilter") {}

	int operator() (class Acq480FMC module, int argc, char* argv[]) {
		if (argc < 3) die("setDecimationFilter CHAN FILTER [odd]");
		int rc = module.chip.setDecimationFilter(
				static_cast<Ads5294::Chan>(atoi(argv[1])),
				static_cast<Ads5294::Filter>(atoi(argv[2])),
				argc<4? false: atoi(argv[3]));
		if (rc != 0) die("setGain failed");
		return 1;
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
		Command("setFilterCoefficients") {}

	int operator() (class Acq480FMC module, int argc, char* argv[]) {
		if (argc < 2) die("setFilterCoefficients CHAN [coeff [coeff]");
		short coeffs[NTAPS] = {};
		if (argc > 2){
			setCoeffs(coeffs, argc-2, argv+2);
		}
		int rc = module.chip.setCustomCoefficients(
			static_cast<Ads5294::Chan>(atoi(argv[1])),
			argc > 2? coeffs: 0);
		return 1;
	}
};

class SetHiPassFilterCommand: public Command {
public:
	SetHiPassFilterCommand() :
		Command("setHiPassFilter") {}
	int operator() (class Acq480FMC module, int argc, char* argv[]) {
		if (argc < 2) die("setHiPassFilter CHAN [enable hpfun]");

		int rc = module.chip.setHiPassFilter(
				static_cast<Ads5294::Chan>(atoi(argv[1])),
				argc > 2? atoi(argv[2]): false,
				argc > 3? strtoul(argv[3], 0, 0): 0
				);
		return 1;
	}
};

class SetDataRateCommand: public Command {
public:
	SetDataRateCommand() :
		Command("setDataRate") {}
	int operator() (class Acq480FMC module, int argc, char* argv[]) {
		if (argc < 2) die("setDataRate RATE");
		int rc = module.chip.setDataRate(
				static_cast<Ads5294::DataRate>(atoi(argv[1])));
		return 1;
	}
};

class SetAverageSelectCommand: public Command {
public:
	SetAverageSelectCommand() :
		Command("setAverageSelect") {}
	int operator() (class Acq480FMC module, int argc, char* argv[]) {
		if (argc < 2) die("setAverageSelect CHAN [ENABLE RATE]");
		int rc = module.chip.setAverageSelect(
			static_cast<Ads5294::Chan>(atoi(argv[1])),
			argc>2? atoi(argv[2]): false,
			argc>3? strtoul(argv[3], 0, 0): 0
		);
		return 1;
	}
};

class SetInvertCommand: public Command {
public:
	SetInvertCommand() :
		Command("SetInvert") {}
	int operator() (class Acq480FMC module, int argc, char* argv[]) {
		if (argc < 2) die("SetInvert CHAN [disable]");
		int rc = module.chip.setInvert(
			static_cast<Ads5294::Chan>(atoi(argv[1])),
			argc>2? atoi(argv[2]): true
		);
		return 1;
	}
};
class SetLFNSCommand: public Command {
public:
	SetLFNSCommand() :
		Command("SetLFNS") {}
	int operator() (class Acq480FMC module, int argc, char* argv[]) {
		if (argc < 2) die("SetLFNS CHAN [disable]");
		int rc = module.chip.setLFNS(
			static_cast<Ads5294::Chan>(atoi(argv[1])),
			argc>2? atoi(argv[2]): true
		);
		return 1;
	}
};

class SetLvdsTestPatRamp: public Command {
public:
	SetLvdsTestPatRamp() :
		Command("SetLvdsTestPatRamp") {}
	int operator() (class Acq480FMC module, int argc, char* argv[]) {
		if (argc < 1) die("SetLvdsTestPatRamp enable");
		int rc = module.chip.SetLvdsTestPatRamp(atoi(argv[1]));
		return 1;
	}
};

class SetLvdsTestPatDeskew: public Command {
public:
	SetLvdsTestPatDeskew() :
		Command("SetLvdsTestPatDeskew") {}
	int operator() (class Acq480FMC module, int argc, char* argv[]) {
		if (argc < 1) die("SetLvdsTestPatRamp enable");
		int rc = module.chip.SetLvdsTestPatDeskew(atoi(argv[1]));
		return 1;
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
	commands.push_back(new SetLvdsTestPatRamp);
	commands.push_back(new SetLvdsTestPatDeskew);
	commands.push_back(new DumpCommand);
	commands.push_back(new FlushCommand);
	commands.push_back(new ReadAllCommand);
	commands.push_back(new ResetCommand);
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
	/* handle busy box style verbs */
	if (strncmp(verb, "acq480_", 7) == 0){
		verb += 7;
	}
	if (argc == 0){
		printf("usage: acq480_knobs command [acq480_help]\n");
		return 0;
	}

	for (VCI it = commands.begin(); it != commands.end(); ++it){
		Command &command = *(*it);
		if (strcmp(verb, command.cmd) == 0){
			if (command(*this, argc, arg0) > 0){
				flush();
			}
		}
	}
	return 0;
}
int main(int argc, char* argv[])
{
	Acq480FMC module(1);

	return module(argc, argv);
}
