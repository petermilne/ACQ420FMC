/*
 * mr_offload.cpp
 *
 *  Created on: 4 May 2020
 *      Author: pgm
 *
 *      offload to nfs mount:
 *      acq2106_182> /mnt/local/mr_offload | pv > /remote/shot4
 185MiB 0:00:07 [26.4MiB/s] [                          <=>                                                                                                                                          ]
acq2106_182> time /mnt/local/mr_offload > /remote/shot4
real	0m 6.28s
user	0m 0.00s
sys	0m 2.22s
 *
 *Faster:
 acq2106_182> nc -l -p 6677 -e /mnt/local/mr_offload
 [root@brotto ~]# nc -i 1 acq2106_182 6677 | pv > /data/shot2
 185MiB 0:00:04 [44.2MiB/s] [       <=>

[root@brotto ~]# nc -i 0.1 acq2106_182 6677 | pv > /data/shot2
 185MiB 0:00:03 [  48MiB/s] [      <=>


 Demux time:
 [peter@andros EPICS]$ camonitor acq2106_182:MODE:TRANS_ACT:STATE acq2106_182:MODE:TRANS_ACT:POST
acq2106_182:MODE:TRANS_ACT:STATE 2020-05-04 22:00:55.889770 IDLE
acq2106_182:MODE:TRANS_ACT:POST 2020-05-04 22:00:55.891951 2000000
acq2106_182:MODE:TRANS_ACT:POST 2020-05-04 22:01:38.355773 0
acq2106_182:MODE:TRANS_ACT:STATE 2020-05-04 22:01:38.355918 ARM
acq2106_182:MODE:TRANS_ACT:STATE 2020-05-04 22:01:39.455395 RUN_POST
acq2106_182:MODE:TRANS_ACT:POST 2020-05-04 22:01:39.459934 43648
acq2106_182:MODE:TRANS_ACT:POST 2020-05-04 22:01:39.496949 218240
acq2106_182:MODE:TRANS_ACT:POST 2020-05-04 22:01:40.598544 349184
acq2106_182:MODE:TRANS_ACT:POST 2020-05-04 22:01:40.600690 480128
acq2106_182:MODE:TRANS_ACT:POST 2020-05-04 22:01:41.700206 1134848
acq2106_182:MODE:TRANS_ACT:POST 2020-05-04 22:01:42.802288 1789568
acq2106_182:MODE:TRANS_ACT:STATE 2020-05-04 22:01:42.802395 POST_PROCESS
acq2106_182:MODE:TRANS_ACT:POST 2020-05-04 22:01:42.804388 2000000
acq2106_182:MODE:TRANS_ACT:STATE 2020-05-04 22:01:48.394864 CLEANUP
acq2106_182:MODE:TRANS_ACT:STATE 2020-05-04 22:01:50.493408 IDLE

=> 7s max.

ie 10s until data.. for 2M samples, 192MB.
 */

#include "local.h"		/* chomp() hopefully, not a lot of other garbage */
#include "knobs.h"
#include "Env.h"
#include "popt.h"

#include <assert.h>

#define NCHAN 48

#define BUFLEN 0x400000
#define FOUT	stdout

struct mr_offload {
	unsigned int shot;	// shot number, set automatically by the box
	unsigned long long TAI;	// time in nsec at start trigger
	float DT;	// sample clock period, nsec
	int nsam;     	// total samples in the shot eg max 2M
	int nchan;    	// number of channels, usually 48
	float ESLO[NCHAN];        // SLOPE,
	float EOFF[NCHAN];        // OFFSET   volts[ch][sam] = raw[ch][sam] * ESLO[ch] + EOFF[ch]
};
/*
	char decims[nsamples];    // array with decimation factor in force at each sample.
	                         // T[sam] = T[sam-1] + DT*decims[sam-1]
	short chdata[NCHAN][NSAM]; // 2D array (or, concatenation of 1D arrays, raw channel data)
*/

struct poptOption opt_table[] = {
		POPT_AUTOHELP
		POPT_TABLEEND
};


void cat(const char* fn, void* buf)
{
	FILE *fp = fopen(fn, "r");
	int nread;
	if (fp == 0){
		fprintf(stderr, "ERROR, failed to open \"%s\"\n", fn);
		exit(1);
	}
	while((nread = fread(buf, 1, BUFLEN, fp)) > 0){
		fwrite(buf, 1, nread, FOUT);
	}
}



namespace G {
	int nsam;
}

const char* init(int argc, const char** argv) {
	poptContext opt_context =
			poptGetContext(argv[0], argc, argv, opt_table, 0);

	int rc;
	while ( (rc = poptGetNextOpt( opt_context )) >= 0 ){
		switch(rc){
		default:
			;
		}
	}

	return poptGetArg(opt_context);
}

struct mr_offload mro = {};

int get_nsam(void)
{
        FILE *pp = popen("get.site 1 TRANS_ACT:POST", "r");

        int post = 0;
        fscanf(pp, "TRANS_ACT:POST %d", &post);
        pclose(pp);
	return post;
}

void offload_header() {
	unsigned tai, tai_vernier;

	getKnob(0, "/dev/acq400.1.knobs/shot", &mro.shot);
	getKnob(0, "/dev/acq400.0.knobs/wr_tai_trg", &tai_vernier);
	getKnob(0, "/dev/acq400.0.knobs/wr_tai_cur", &tai);		// @@TODO : should be time at TRG time ..0
	mro.TAI = tai;
	mro.TAI <<=32;
	mro.TAI = tai_vernier;
	mro.DT = 25;
	mro.nsam = get_nsam();
	mro.nchan = NCHAN;
	for (int ii = 0; ii < NCHAN; ++ii){
		mro.ESLO[ii] = 3.0517578e-5;
	}
	fwrite(&mro, sizeof(struct mr_offload), 1, stdout);
}

int offload(void) {
	offload_header();
	void* buf = new short* [BUFLEN];
	cat("/dev/shm/decims", buf);
	char fname[80];

	int maxsite = NCHAN/8;
	for (int site = 1; site <= maxsite; ++site){
		for (int ch = 1; ch <= 8; ++ch){
			sprintf(fname, "/dev/acq400/data/%d/%02d", site, ch);
			cat(fname, buf);
		}
	}
	return 0;
}

int writebin(const char* fmt, const char* uutname, int shot, void* data, int len)
{
	char fname[80];

	sprintf(fname, fmt, uutname, shot);
	FILE *fp = fopen(fname, "w");
	assert(fp);
	fwrite(data, len, 1, fp);
	fclose(fp);
	return 0;
}

int client(const char* uutname) {
	mr_offload header;

	fread(&header, sizeof(mr_offload), 1, stdin);

	writebin("%s.%d.hdr", uutname, header.shot, &header, sizeof(mr_offload));

	char* decims = new char[header.nsam];
	fread(decims, sizeof(char), header.nsam, stdin);

	writebin("%s.%d.dec", uutname, header.shot, decims, header.nsam);

	const int len = header.nchan * header.nsam;
	short* payload = new short[len];
	fread(payload, sizeof(short), len, stdin);

	writebin("%s.%d.dat", uutname, header.shot, payload, sizeof(short)*len);

	return 0;
}

int main(int argc, const char** argv)
{
	const char* uutname = init(argc, argv);
	if (uutname != 0){
		return client(uutname);
	}else{
		return offload();
	}
	return 0;
}
