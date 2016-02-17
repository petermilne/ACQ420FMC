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
		assert(fp != 0);
		buf = new T[nsam];
		ns = fread(buf, sizeof(T), nsam, fp);
	}
	~File() {
		if (fp) fclose(fp);
		if (buf) delete [] buf;
	}
};

template <class T>
void dump_channels(const char* fmt, int c1, int c2, int nsam)
{
	int ccount = c2 - c1 + 1;
	File<T> **files = new File<T> * [ccount];
	T* raw = new T[nsam*ccount];
	T* dest = raw;

	int cx;

	for (cx = c1; cx <= c2; ++cx){
		files[cx-c1] = new File<T>(fmt, cx, nsam);
	}

	for (int isam = 0; isam < nsam; ++isam){
		for (cx = c1; cx <= c2; ++cx){
			*dest++ = files[cx-c1]->buf[isam];
		}
	}
	fwrite(raw, sizeof(T), nsam, stdout);
}

main(int argc, char* argv[])
{

	system("inotifywait /dev/shm/AI.1.wf.fin 2>&1 >/dev/null");
	dump_channels<int>("/dev/shm/AI.1.wf/CH%02d", 1, 8, 512);
}

