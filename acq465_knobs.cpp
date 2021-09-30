/*
 * acq465_knobs.cpp
 *
 *  Created on: 30 Sep 2021
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

#include "acq465_ioctl.h"


void die(const char *fmt)
{
	fprintf(stderr, "ERROR: %s", fmt);
	exit(1);
}

struct Command;


class Acq465ELF {
	int site;
	unsigned lcs;
	unsigned char* clibuf;

	vector<Command*> commands;
	FILE* fp;

	void init_commands();
public:
	Acq465ELF(int _site, unsigned _lcs) : site(_site), lcs(_lcs)
	{
		char fname[80];
		snprintf(fname, 80, "/dev/acq465.%d", site);
		fp = fopen(fname, "r+");
		if (!fp){
			perror(fname);
			exit(1);
		}
		clibuf = (unsigned char*)mmap(NULL, TOTAL_SPI_BUFFER_LEN,
				PROT_READ|PROT_WRITE, MAP_SHARED, fileno(fp), 0);
		if (clibuf == MAP_FAILED){
			die("MAP_FAILED");
		}
		init_commands();
	}

	int operator() (int argc, char* argv[]);

	int invalidate() {
		return ioctl(fileno(fp), ACQ465_CACHE_INVALIDATE, lcs);
	}
	int flush() {
		return ioctl(fileno(fp), ACQ465_CACHE_FLUSH, lcs);
	}
	int reset() {
		return ioctl(fileno(fp), ACQ465_RESET, lcs);
	}

	unsigned char* cache() {
		return clibuf + (site-1)*MODULE_SPI_BUFFER_LEN + lcs*REGS_LEN;
	}

	friend class HelpCommand;
	friend class MakeLinksCommand;
};


struct Command {
	const char* cmd;
	const char* args_help;

	char* _help;
	virtual int operator() (class Acq465ELF& module, int argc, char* argv[]) = 0;
	/* return > 0 if flush recommended */

	Command(const char* _cmd, const char* _args_help = "") :
		cmd(_cmd), args_help(_args_help) {
		_help = new char[25 + strlen(args_help) + 2];
		sprintf(_help, "%-25s %s", cmd, args_help);
	}

	const char* help() {
		return _help;
	}
};


typedef vector<Command*>::iterator VCI;

class HelpCommand: public Command {
public:
	HelpCommand() : Command("help") {}

	int operator() (class Acq465ELF& module, int argc, char* argv[]) {
		for (VCI it = module.commands.begin(); it != module.commands.end(); ++it){
			printf("%s\n", (*it)->help());
		}
		return 0;
	}
};


class MakeLinksCommand: public Command {
public:
	MakeLinksCommand() : Command("makeLinks") {}

	int operator() (class Acq465ELF& module, int argc, char* argv[]) {
		for (VCI it = module.commands.begin(); it != module.commands.end(); ++it){
			printf("ln -s %s acq465_%s\n", "/usr/local/bin/acq465_knobs", (*it)->cmd);
		}
		return 0;
	}
};
class ResetCommand: public Command {
public:
	ResetCommand() : Command("reset") {}

	int operator() (class Acq465ELF& module, int argc, char* argv[]) {
		return module.reset();
	}
};

class ReadAllCommand: public Command {
public:
	ReadAllCommand() : Command("readall") {}

	int operator() (class Acq465ELF& module, int argc, char* argv[]) {
		return module.invalidate();
	}
};
class FlushCommand: public Command {
public:
	FlushCommand() : Command("flush") {}
	int operator() (class Acq465ELF& module, int argc, char* argv[]) {
		return module.flush();
	}
};

class SetReg: public Command {
public:
	SetReg() :
		Command("reg", "REG [VALUE]") {}
	int operator() (class Acq465ELF& module, int argc, char* argv[]) {
		if (argc < 2) die(help());
		unsigned reg = strtoul(argv[1], 0, 0);
		unsigned regval;

		if (reg >= REGS_LEN){
			return -1;
		}

		if (argc == 2){
			regval = module.cache()[reg];
			printf("%02x=%04x\n", reg, regval);
		}else{
			regval = strtoul(argv[2], 0, 0);
			if (regval >= 256){
				return -2;
			}
			module.cache()[reg] = regval;
		}
		return 0;
	}
};

void Acq465ELF::init_commands()
{

	commands.push_back(new SetReg);
	commands.push_back(new FlushCommand);
	commands.push_back(new ReadAllCommand);
	commands.push_back(new ResetCommand);

	commands.push_back(new HelpCommand);
	commands.push_back(new MakeLinksCommand);
}

int  Acq465ELF::operator() (int argc, char* argv[])
{
	char** arg0 = argv;
	char* verb = basename(argv[0]);

	if (strcmp(verb, "acq465_knobs") == 0){
		arg0 = &argv[1];
		verb = arg0[0];
		argc--;
	}

	if (argc == 0){
		printf("usage: acq465_knobs command [acq465_help]\n");
		return 0;
	}

	/* handle busy box style verbs */
	if (strncmp(verb, "acq465_", 7) == 0){
		verb += 7;
	}

	for (VCI it = commands.begin(); it != commands.end(); ++it){
		Command &command = *(*it);
		if (strcmp(verb, command.cmd) == 0){
			if (command(*this, argc, arg0) > 0){
				flush();
			}
			break;
		}
	}
	return 0;
}

int main(int argc, char* argv[])
{
	int site = 1;
	unsigned lcs = 0x0;
	if (getenv("SITE")){
		site = atoi(getenv("SITE"));
	}
	if (getenv("LCS")){
		lcs = strtoul(getenv("LCS"), 0, 0);
	}

	Acq465ELF module(site, lcs);


	return module(argc, argv);
}
