/*
 * slowmon.cpp : read slow mon data, aggregate and output
 *
 *  Created on: 18 Mar 2022
 *      Author: pgm
 */

#include "popt.h"

#include "local.h"
#include "Env.h"
#include "File.h"
#include "knobs.h"


#include "split2.h"

#include <unistd.h>
#include <signal.h>


/* meta data front and back. no math! */
#define META1	sizeof(long)
#define META2	sizeof(long)

namespace G {
	unsigned usec = 250000;
	unsigned nacc = 1;

	unsigned isam = 0;
}

#define NSPAD	4
#define SPADLEN	(NSPAD*sizeof(unsigned))


class Sampler {
public:
	virtual void onSample(int sig) = 0;
	virtual ~Sampler() {}
};

Sampler* G_sampler;

void onSample(int sig){
	return G_sampler->onSample(sig);
}

template <class T>
class SamplerImpl: public Sampler {
	const int ndata;		// number of T elements in data
	T* data;			// ndata elements of T + SPAD
	long* sums;			// ndata elements of long;
	FILE* fp;

	void clear_sums() {
		memset(sums, 0, ndata*sizeof(long));
	}
	ssize_t len(){
		return (ndata+SPADLEN/sizeof(T))*sizeof(T);
	}
public:
	SamplerImpl(unsigned ssb): ndata(ssb/sizeof(T)) {
		data = new T[ndata+SPADLEN/sizeof(T)];
		sums = new long[ndata];
		fp = fopen("/dev/acq400.0.subr", "r");
	}
	virtual ~SamplerImpl() {
		delete [] data;
		delete [] sums;
		fclose(fp);
	}
	void sumUp(void);

	void onSample(int sig)
	{
		ssize_t nread = read(fileno(fp), data, len());
		if (nread != len()){
			perror("read fail");
			exit(1);
		}

		if (G::nacc == 1){
			write(1,data, len());
		}else{
			for (int ii = 0; ii < ndata; ++ii){
				sums[ii] += data[ii];
			}
		}
		if (++G::isam == G::nacc){
			for (int ii = 0; ii < ndata; ++ii){
				data[ii] = sums[ii] / G::nacc;
			}
			write(1, data, len());
			clear_sums();
			G::isam = 0;
		}
	}
};


typedef std::vector<std::string> VS;

void init(int argc, const char* argv[])
{
	unsigned ssb = 0;
	bool data32 = false;
	char sites[16];
	VS sitelist;
	getKnob(0, "/etc/acq400/0/sites", sites);
	split2(sites, sitelist, ',');
	for (std::string st: sitelist){
		int site = atoi(st.c_str());
		unsigned _ssb;
		unsigned _data32;
		char _ssb_knob[32];
		snprintf(_ssb_knob, 32, "/etc/acq400/%d/ssb", site);
		getKnob(0, _ssb_knob, &_ssb);
		ssb += _ssb;
		getKnob(site, "data32", &_data32);
		if (_data32){
			data32 = true;
		}
	}
	if (data32){
		G_sampler = new SamplerImpl<unsigned>(ssb);
	}else{
		G_sampler = new SamplerImpl<short>(ssb);
	}
}

void ui()
{
	unsigned usec;
	unsigned fs;
	unsigned fin;

	getKnob(0, "/etc/acq400/0/slowmon_fs", &fs);
	getKnob(0, "/etc/acq400/0/slowmon_fin", &fin);
	G::nacc = fin/fs;
	usec = 1000000/fs;

	if (usec != G::usec){
		G::usec = usec;
		ualarm(G::usec, G::usec);
	}
}

int main(int argc, const char* argv[])
{
	init(argc, argv);
	signal(SIGALRM, onSample);
	ualarm(G::usec, G::usec); //alarm in a second, and every second after that.

	for(;;){
		pause();
		ui();
	}
	return 0;
}


