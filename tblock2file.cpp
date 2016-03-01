/*
 * tblock2file.cpp
 *
 *  Created on: 29 Feb 2016
 *      Author: pgm
 *      ref
 *      /home/pgm/PROJECTS.HOY3/GEOSTORE3/TBLOCK2BURST
 */

/*
 *  block on bq
 *  read the tblock. filter the channels, output to file.
 *  new file every block
 *  new dir
 *  /data/JOB/YYYYMMDD-HH/MM/nnnnn.{dat/txt}
 */
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <assert.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <string>
#include <vector>

#include "popt.h"
#include "local.h"
#include "knobs.h"

#define NCHAN	4

using namespace std;

namespace G {
	unsigned int nchan = NCHAN;
	int wordsize = 2;
	const char *fmt  = "%Y-%j/%H/%M";
	int buflen;
};

void init_globs(void)
{
	getKnob(0, "/etc/acq400/0/NCHAN", &G::nchan);
	unsigned int data32 = false;
	getKnob(0, "/etc/acq400/0/data32", &data32);
	G::wordsize = data32? sizeof(int): sizeof(short);
}

struct poptOption opt_table[] = {
		POPT_AUTOHELP
		POPT_TABLEEND
};

void init(int argc, const char** argv) {
	init_globs();
	poptContext opt_context =
			poptGetContext(argv[0], argc, argv, opt_table, 0);
	int rc;
	bool fill_ramp = false;

	while ( (rc = poptGetNextOpt( opt_context )) >= 0 ){
		switch(rc){
			;
		}
	}
}

template <class T>
class Mapping {
	T* _data;
public:
	Mapping(string fname, int len) {

	}
	~Mapping() {

	}
	T* data() {
		return _data;
	}
};

class File {
	FILE *_fp;
public:
	File(const char *fname, const char* mode){
		_fp = fopen(fname, mode);
		if (_fp == 0){
			perror(fname);
			exit(1);
		}
	}
	~File() {
		fclose(_fp);
	}
	FILE* operator() () {
		return _fp;
	}
};

template <class T>
class Archiver {
	FILE *bq;
	string job;
	string jobroot;
	string base;
	string dat;
	string txt;
	int seq;
	char ydhm[32];

	char outbase[128];

	char* makeDateRoot(char ydmh[], int maxstr = 32) {
	        time_t t = time(NULL);
	        struct tm *tmp = localtime(&t);
	        strftime(ydmh, maxstr, G::fmt, tmp);
	}
	void mkdir(char* root){
		snprintf(outbase, 128, "%s/%s", jobroot.data(), root);
		char cmd[128];
		strcpy(cmd, "mkdir -p ");
		strcat(cmd, outbase);
		system(cmd);
	}

	T* mmap(string buf) {

	}
public:
	Archiver(string _job) :
		job(_job), jobroot("/data/"), base ("/dev/acq400.0.hb/"), dat(".dat"), txt(".txt"),
		seq(0)	{
		jobroot = jobroot + job;
		bq = fopen("/dev/acq400.0.bqf", "r");
		if (bq == 0){
			perror("/dev/acq400.0.bqf");
			exit(1);
		}
		makeDateRoot(ydhm);
		mkdir(ydhm);
	}

	int operator() () {
		char bufn[32];

		for(; fgets(bufn, 32, bq); ++seq){
			checkTime();
			string _bufn(chomp(bufn));
			process(base + bufn);
			processAux();
		}

		return 0;
	}

	void checkTime() {
		char ydhm1[32];
		makeDateRoot(ydhm1);
		if (strcmp(ydhm, ydhm1) != 0){
			strcpy(ydhm, ydhm1);
			mkdir(ydhm);
		}
	}
	void process(string bufn){
		Mapping<T> m(base + bufn, G::buflen);
		char fname[128];
		snprintf(fname, 128, "%s/%06d%s", outbase, seq, dat.data());
		File fout(fname, "w");

		// mmap the file, read the data, write the channels */

	}
	void processAux(void) {
		char cmd[128];
		snprintf(cmd, 128, "cat /dev/shm/sensors > %s/%06d%s", outbase, seq, txt.data());
		system(cmd);
	}
};

int main(int argc, const char** argv)
{
	init(argc, argv);

	Archiver<int> archiver("job");
	return archiver();
}

