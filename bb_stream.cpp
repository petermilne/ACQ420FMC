/* streaming DAC */


#include <stdio.h>
#include <unistd.h>
#include <vector>
#include "popt.h"

using namespace std;

#include "Buffer.h"
#include "knobs.h"
#include "tcp_server.h"

namespace G {
	unsigned play_bufferlen;		// Change bufferlen on play
	char* port = 0;				// 0 no server (inetd), else make a server
	char* host = 0;
	int verbose = 0;
};

struct poptOption opt_table[] = {
	{
	  "port", 'p', POPT_ARG_STRING, &G::port, 0, "server port 0: no tcp server (using inetd)"
	},
	{
	  "host", 'H', POPT_ARG_STRING, &G::host, 0, "server host 0: allow any host"
	},
	{
	  "verbose", 'v', POPT_ARG_INT, &G::verbose, 0, "debug"
	},
	POPT_AUTOHELP
	POPT_TABLEEND
};
inline int inc(int cursor, int max)
{
	if (++cursor == max){
		return 0;
	}else{
		return cursor;
	}
}


int read_buffers(unsigned descriptors[], int ndesc)
{
	int totbytes = 0;
	for (int id = 0; id < ndesc; ++id){
		char* bp = Buffer::the_buffers[0]->getBase() + descriptors[id]*Buffer::bufferlen;
		int nb = fread(bp, 1, G::play_bufferlen, stdin);
		if (nb <= 0){
			fprintf(stderr, "read_buffers fread returned %d\n", nb);
			return 0;
		}else{
			totbytes += nb;
		}
	}
	return totbytes;
}

int playloop() {
	unsigned descriptors[128];

	FILE* fp = fopen("/dev/acq400.0.dac", "a+");
	if (fp == 0){
		fprintf(stderr, "ERROR failed to open file\n");
		return -1;
	}
	int nbuf = fread(descriptors, sizeof(unsigned), 30, fp);
	printf("nbuf:%d\n", nbuf);

	int put = 0;
	int get = 0;
	while(1){
		int nbytes = read_buffers(descriptors+get, 2);
		if (nbytes <= 0){
			return nbytes;
		}

		int nw = fwrite(descriptors+get, sizeof(unsigned), 2, fp);
		if (nw == 2){
			printf("wr %08x %08x\n", descriptors[get], descriptors[get+1]);
		}else{
			printf("write %d returned %d\n", get, nw);
		}
		get = inc(get, nbuf);
		get = inc(get, nbuf);

		int nr = fread(descriptors+put, sizeof(unsigned), 2, fp);
		if (nr == 2){
			printf("rd %08x %08x\n", descriptors[put], descriptors[put+1]);
		}else{
			printf("read %d returned %d\n", put, nr);
		}
		put = inc(put, nbuf);
		put = inc(put, nbuf);
	}
	return 0;
}

int playloop_redirect(FILE* fin, FILE* fout)
{
	close(0); dup(fileno(fin));
//	close(1); dup(fileno(fout));
//	close(2); dup(fileno(fout));
	return playloop();
}

#define MODPRAMS "/sys/module/acq420fmc/parameters/"
#define BUFLEN	 MODPRAMS "bufferlen"
#define NBUF	 MODPRAMS "nbuffers"

void ui(int argc, const char** argv)
{
	poptContext opt_context =
			poptGetContext(argv[0], argc, argv, opt_table, 0);

	int rc;

	while ( (rc = poptGetNextOpt( opt_context )) >= 0 ){
		switch(rc){
		default:
			;
		}
	}
	getKnob(-1, BUFLEN, &Buffer::bufferlen);
	getKnob(-1, "/etc/acq400/0/dist_bufferlen_play", &G::play_bufferlen);
	if (G::play_bufferlen == 0){
		G::play_bufferlen = Buffer::bufferlen;
	}
}
int main(int argc, const char* argv[]) {
	ui(argc, argv);
	BufferManager bm("/dev/acq400.0", 0);
	if (G::port){
		return tcp_server(G::host, G::port, playloop_redirect);
	}else{
		return playloop();
	}
}
