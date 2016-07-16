/*
 * muxdec.cpp


 *
 *  Created on: 16 Feb 2016
 *      Author: pgm
 *      inotifywait /dev/shm/AI.1.wf.fin
 *      then decimate/read
 */

#include <stdio.h>
#include <assert.h>
#include <stdlib.h>

#include <syslog.h>
#include <string.h>

#include <vector>

#include "acq-util.h"

template <class T>
struct File {
	char fname[80];
	FILE *fp;
	T* buf;
	int ns;

	File() : fp(0), buf(0) {}
	File(const char* fmt, int cx, int nsam) {
		snprintf(fname, 80, fmt, cx);
		fp = fopen(fname, "r");
		if (fp == 0){
			perror(fname);
			exit(-1);
		}
		buf = new T[nsam];
		ns = fread(buf, sizeof(T), nsam, fp);
	}
	~File() {
		if (fp) fclose(fp);
		if (buf) delete [] buf;
	}
};


class DataFormatter {
protected:
	const int nchan;
	const int stride;
	const int shr;
	std::vector<int> channels;

	static int getNC(char* def) {
		char* cursor;
		int nc = 8;

		if ((cursor = strstr(def, "NC=")) != 0){
			if (sscanf(cursor, "NC=%d", &nc) == 1){
				;
			}
		}
		return nc;
	}
	void getChannels(char *def){
		char *cursor;
		int* _channels = new int[nchan];
		char _def[128];
		int ic;

		memset(_channels, 0, nchan*sizeof(int));

		if ((cursor = strstr(def, "CH=")) != 0 &&
			sscanf(cursor, "CH=%s", _def) == 1){
			int nprint = acqMakeChannelRange(_channels, nchan, _def);

			if (nprint == 0){
				fprintf(stderr, "ERROR: empty channel range\n");
				exit(1);
			}
			for (ic = 1; ic <= nchan; ++ic){
				if (_channels[ic]){
					channels.push_back(ic);
				}
			}
		}else{
			for (ic = 1; ic <= nchan; ++ic){
				channels.push_back(ic);
			}
		}
	}
	static int getStride(char* def) {
		char *cursor;
		int stride = 1;

		if ((cursor = strstr(def, "ST=")) != 0){
		     sscanf(cursor, "ST=%d", &stride);
		}
		return stride;
	}

	static int getShr(char* def) {
		char* cursor;
		int shr = 0;

		if ((cursor = strstr(def, "SHR=")) != 0){
			sscanf(cursor, "SHR=%d", &shr);
		}
		return shr;
	}
public:
// NC=n ; CH=channel-def
	DataFormatter(char *def) : nchan(getNC(def)),stride(getStride(def)),shr(getShr(def)) {
		channels.reserve(nchan);
		getChannels(def);
	}

	virtual void dump(const char* fmt, int nsam) = 0;

	static DataFormatter& create(char *defaults);

};


template <class T>
class DataFormatterImpl : public DataFormatter {
	friend class DataFormatter;
protected:
	virtual void onDump(File<T> **files, int nchan, int nsam){
		for (int isam = 0; isam < nsam; ++isam){
			char row[80];
			char* cursor = row;

			cursor += sprintf(cursor, "[ \"d\", ");
			cursor += sprintf(cursor, "%d,", isam*stride);
			for (int cx = 0; cx < nchan; ++cx){
				cursor += sprintf(cursor, "%d%s",
					files[cx]->buf[isam*stride] >> shr,
					cx+1==nchan? "]\n": ",");
			}
			fputs(row, stdout);
		}
	}
	DataFormatterImpl(char* defaults) : DataFormatter(defaults)
	{}

public:
	virtual void dump(const char* fmt, int nsam) {
		int ccount = channels.size();
		File<T> **files = new File<T> * [ccount];
		int cx;

		printf("[ \"l\", \"sample\",");
		for (cx = 0; cx < ccount; ++cx){
			files[cx] = new File<T>(fmt, channels[cx], nsam*stride);
			printf("\"CH%02d\"%s", channels[cx], cx+1==ccount? "]\n": ",");
		}

		onDump(files, ccount, nsam);
	}

};

DataFormatter& DataFormatter::create(char *defaults) {
	fprintf(stderr, "DataFormatter::create(%s)\n", defaults);

	char *cursor = strstr(defaults, "SZ");
	if (cursor && strncmp(cursor, "SZ=short", 5) == 0){
		fprintf(stderr, "DataFormatterImpl<short>\n");
		return * new DataFormatterImpl<short>(defaults);
	}else{
		return * new DataFormatterImpl<int>(defaults);
	}
}


int main(int argc, char* argv[])
{
	openlog("muxdec", LOG_PID, LOG_USER);
	//syslog(LOG_DEBUG, "muxdec 01\n");
	char defaults[256];
	FILE* fp = fopen("/dev/shm/muxdec.sh", "r");
	if (fp){
		fgets(defaults, 128, fp);
		fclose(fp);
	}

	/* DataFormatter uses strstr(), overrides from the beginning of str */
	char cmd[256];
	fgets(cmd, 256, stdin);
	strncat(cmd, " ", 256);
	strncat(cmd, defaults, 256);
	DataFormatter& df =  DataFormatter::create(cmd);
	// strcat augment defaults ?
	//syslog(LOG_DEBUG, "muxdec 44\n");
	system("inotifywait /dev/shm/AI.0.wf.fin 2>/dev/null >/dev/null");
	df.dump("/dev/shm/AI.0.wf/CH%02d", 512);
	//syslog(LOG_DEBUG, "muxdec 99\n");
}

