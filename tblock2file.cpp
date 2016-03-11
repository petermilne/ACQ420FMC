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
#include <fcntl.h>
#include <unistd.h>

#include <signal.h>
#include <string>
#include <vector>

#include "popt.h"
#include "local.h"
#include "knobs.h"

#include "File.h"

#include "acq-util.h"

#define NCHAN	4
#define MAXJOBS_PER_DAY	24

using namespace std;

namespace G {
	int devnum = 0;
	unsigned int nbuffers;
	unsigned int bufferlen;
	unsigned int nchan = NCHAN;
	int wordsize = 2;
	const char *jobfmt = "%Y-%j";		// do this ONCE
	const char *datfmt  = "%H/%M";		// do this every
	int *channels;				// index from 1
	int nchan_selected;
	int shr = 4;				// scale 20 bit to 16 bit. Gain possible ..
	bool raw = false;
	const char* job;
	int runtime;
	int verbose;
};

void init_globs(void)
{
	getKnob(0, "/etc/acq400/0/NCHAN", &G::nchan);
	unsigned int data32 = false;
	getKnob(0, "/etc/acq400/0/data32", &data32);
	G::wordsize = data32? sizeof(int): sizeof(short);
	getKnob(G::devnum, "nbuffers",  &G::nbuffers);
	getKnob(G::devnum, "bufferlen", &G::bufferlen);

	G::channels = new int[G::nchan+1];
	memset(G::channels, 0, (G::nchan+1)*sizeof(int));

	File env("/etc/sysconfig/tblock2burst.ini", "r", File::NOCHECK);
	if (env.fp()){
		char ebuf[128];
		while (fgets(ebuf, 128, env.fp())){
			char *cursor;

			if ((cursor = strstr(ebuf, "CH=")) != 0){
				G::nchan_selected =
					acqMakeChannelRange(G::channels, G::nchan, cursor+3);
			}
			if ((cursor = strstr(ebuf, "SHR=")) != 0){
				G::shr = atoi(cursor+4);
			}
		}
	}


}

static const char *chandef;
static const char *pidname;

struct poptOption opt_table[] = {
	{ "runtime", 'R', POPT_ARG_INT, &G::runtime, 'R',
				"duration of run in seconds" },
	{ "channel-mask", 'C', POPT_ARG_STRING, &chandef, 'C',
				"channel mask" 		},
	{ "SHR", 'S', POPT_ARG_INT, &G::shr, 0,
				"right shift scaling factor [4] 0..4"
	},
	{ "pidname", 'p', POPT_ARG_STRING, &pidname, 'p', "pid file" },
	{ "verbose", 'v', POPT_ARG_INT, &G::verbose, 0, "" },
	POPT_AUTOHELP
	POPT_TABLEEND
};

static void cleanup(const char* message) {
	rename("/mnt/local/pig-job", "/mnt/local/pig-job.old");
	if (pidname) unlink(pidname);
	syslog(LOG_DEBUG, message);
	fprintf(stderr, message);
	exit(0);
}
static void alarm_handler(int signum) {
	cleanup("runtime complete\n");
}

static void quit_handler(int signum) {
	cleanup("told to quit\n");
}

static bool file_exists (const char *filename)
{
  struct stat   buffer;
  return (stat (filename, &buffer) == 0);
}

static void install_handlers(void) {
        struct sigaction sa;
        sa.sa_handler = alarm_handler;
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = 0;
        if (sigaction(SIGALRM, &sa, NULL)) perror ("sigaction");

        struct sigaction saq;
        saq.sa_handler = quit_handler;
        sigemptyset(&saq.sa_mask);
        saq.sa_flags = 0;
        if (sigaction(SIGINT, &saq, NULL)) perror ("sigaction");
}


void init(int argc, const char** argv) {
	openlog("tblock2file", LOG_PID, LOG_USER);
	init_globs();
	poptContext opt_context =
			poptGetContext(argv[0], argc, argv, opt_table, 0);
	int rc;
	bool fill_ramp = false;

	while ( (rc = poptGetNextOpt( opt_context )) >= 0 ){
		switch(rc){
		case 'R':
			if (G::runtime > 60) alarm(G::runtime);
			break;
		case 'C':
			G::nchan_selected = acqMakeChannelRange(
					G::channels, G::nchan, chandef);
			if (G::verbose > 1){
				printf("mask for chandef \"%s\" %d %d\n", chandef, G::nchan, G::nchan_selected);
				for (int ii = 1; ii <= G::nchan; ++ii){
					printf("%d,", G::channels[ii]);
				}
				printf("\n");
			}
			syslog(LOG_DEBUG, "channel-mask selected %d\n", G::nchan_selected);
			break;
		case 'p':
			if (file_exists(pidname)){
				syslog(LOG_DEBUG, "existing process at \"%s\" quitting\n", pidname);
				exit(1);
			} else {
				File pid(pidname, "w");
				fprintf(pid.fp(), "%d\n", getpid());
			}
			break;
		}
	}
	const char* job = poptGetArg(opt_context);
	G::job = job==0? "job": job;

	if (G::nchan_selected == 0){
		G::nchan_selected = acqMakeChannelRange(
				G::channels, G::nchan, ":");
	}
	install_handlers();
}



class Archiver {
protected:
	FILE *bq;
	string job;
	string jobroot;
	string base;
	string dat;
	string txt;
	int seq;
	char hm[32];

	char outbase[128];

	File* aux_file;		// per minute nav data

	File* aux_summary;	// per shot nav data, one entry per minute
	File* raw_summary;	// per shot raw data, one entry per minute

	bool is_new_minute;



	static char* makeDateRoot(char hm[], int maxstr = 32) {
	        time_t t = time(NULL);
	        struct tm *tmp = localtime(&t);
	        strftime(hm, maxstr, G::datfmt, tmp);
	}

	static void _mkdir(char* dirname) {
		char cmd[128];
		strcpy(cmd, "mkdir -p ");
		strncat(cmd, dirname, 128-strlen(cmd));
		system(cmd);
	}
	void mkdir(char* root){
		snprintf(outbase, 128, "%s/%s", jobroot.data(), root);
		_mkdir(outbase);
	}


	void notify() {
		File fn("/dev/shm/tblock2file", "w");
		fprintf(fn(), "FILE=%s/%06d\n", outbase, seq);
	}

	void createSummaryFiles() {
		fprintf(stderr, "createSummaryFiles() %s/%s\n", jobroot.c_str(), "summary.txt");
		aux_summary = new File(jobroot.c_str(), "summary.txt", "w");
		raw_summary = new File(jobroot.c_str(), "summary.bin", "w");
	}
	void createAuxFile() {
		if (aux_file) delete aux_file;

		char auxfn[128];
		snprintf(auxfn, 128, "%s/%06d%s", outbase, seq, txt.data());
		aux_file = new File(auxfn, "w");
	}
	void stashAux(FILE *fout, const char* in_fname){
		File fp_sensors(in_fname, "r", File::NOCHECK);
		if (fp_sensors()){
			char buf[256];
			fgets(buf, 256, fp_sensors());
			fputs(chomp(buf), fout);
		}else{
			fprintf(stderr, "file \"%s\" not available", in_fname);
		}
	}
	static char* makeJobRoot(const char* root) {
		static char jr[80];
		time_t t = time(NULL);
		struct tm *tmp = localtime(&t);

		strncpy(jr, root, 80);
		strcat(jr, "/");
		char *bname = jr+strlen(jr);
		strftime(jr+strlen(jr), 80-strlen(jr), G::jobfmt, tmp);

		int len1 = strlen(jr);
		int seq = 1;

		for (int seq = 0; seq <= MAXJOBS_PER_DAY; ++seq){
			sprintf(jr+len1, ".%02d", seq);

			if (!file_exists(jr)){
				_mkdir(jr);
				File ident("/var/run/tblock2file.job", "w");
				fputs(bname, ident.fp());
				return jr;
			}
		}

		fprintf(stderr, "maximum 24 jobs per day exceeded\n");
		exit(1);
		return 0;
	}
	void processAux(FILE *fp) {
		fprintf(fp, "%06d ", seq);
		stashAux(fp, "/dev/shm/sensors");
		stashAux(fp, "/dev/shm/imu");
		fputs("\n", fp);
	}
public:
	Archiver(string _job) :
		job(_job), jobroot(makeJobRoot("/data/job")),
		base ("/dev/acq400.0.hb/"), dat(".dat"), txt(".txt"),
		seq(0), aux_file(0), is_new_minute(false)	{
		bq = fopen("/dev/acq400.0.bqf", "r");
		if (bq == 0){
			perror("/dev/acq400.0.bqf");
			exit(1);
		}
		createSummaryFiles();
		makeDateRoot(hm);
		mkdir(hm);

		createAuxFile();
	}

	int operator() () {
		char bufn[32];

		for(; fgets(bufn, 32, bq); ++seq){
			is_new_minute = checkTime();
			string _bufn(chomp(bufn));

			process(base + bufn);
			processAux();
		}

		return 0;
	}

	bool checkTime() {
		char ydhm1[32];
		makeDateRoot(ydhm1);
		if (strcmp(hm, ydhm1) != 0){
			strcpy(hm, ydhm1);
			mkdir(hm);
			createAuxFile();
			return true;
		}else{
			return false;
		}
	}
	virtual void process(string bufn) = 0;

	void processAux(void) {
		processAux(aux_file->fp());
		if (is_new_minute){
			fprintf(stderr, "processAux %d\n", is_new_minute);
			processAux(aux_summary->fp());
			fflush(aux_summary->fp());
		}
	}

	static Archiver& create(string _job);
};

template <class T>
class RawArchiver : public Archiver {
	friend class Archiver;
protected:
	RawArchiver(string _job) : Archiver(_job) {
		fprintf(stderr, "RawArchiver<%d>\n", sizeof(T));
	}
public:
	virtual void process(string bufn){
		Mapping<T> m(bufn, G::bufferlen);
		char fname[128];
		snprintf(fname, 128, "%s/%06d%s", outbase, seq, dat.data());
		File fout(fname, "w");
		fwrite(m(), sizeof(T), G::bufferlen/sizeof(T), fout());
		if (is_new_minute){
			fwrite(m(), sizeof(T), G::nchan_selected, aux_summary->fp());
		}
	}
};

template <class FROM, class TO>
class ChannelArchiver : public Archiver {
	friend class Archiver;
protected:
	TO *tobuf;

	ChannelArchiver(string _job) : Archiver(_job) {
		fprintf(stderr, "ChannelArchiver<%d,%d>\n", sizeof(FROM), sizeof(TO));
		tobuf = new TO [G::nchan_selected];
	}
public:
	virtual ~ChannelArchiver() {
		delete [] tobuf;
	}
	virtual void process(string bufn){
		Mapping<FROM> m(bufn, G::bufferlen);
		char fname[128];
		snprintf(fname, 128, "%s/%06d%s", outbase, seq, dat.data());
		File fout(fname, "w");

		const FROM* cmax = m() + G::bufferlen/sizeof(FROM);

		for (const FROM* cursor = m(); cursor < cmax; cursor += G::nchan){
			TO* toc = tobuf;
			for (int ic = 0;
				ic < G::nchan && toc-tobuf < G::nchan_selected; ++ic){
				if (G::channels[ic+1]){
					*toc++ = (cursor[ic] >> G::shr) & 0x0000FFFF;
				}
			}
			fwrite(tobuf, sizeof(TO), G::nchan_selected, fout());
			if (is_new_minute){
				fwrite(tobuf, sizeof(TO), G::nchan_selected, raw_summary->fp());
			}
		}
	}
};
Archiver& Archiver::create(string _job) {
	if (G::raw){
		return * new RawArchiver<int> (_job);
	}else{
		return * new ChannelArchiver<int, short> (_job);
	}
}

int main(int argc, const char** argv)
{
	init(argc, argv);
	Archiver& archiver = Archiver::create(G::job);
	return archiver();
}

