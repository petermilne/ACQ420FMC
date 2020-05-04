/*
 * mr_offload.cpp
 *
 *  Created on: 4 May 2020
 *      Author: pgm
 */

#include "local.h"		/* chomp() hopefully, not a lot of other garbage */
#include "knobs.h"


#define NCHAN 48

struct mr_offload {
	int shot;	// shot number, set automatically by the box
	long long TAI;	// time in nsec at start trigger
	float DT;	// sample clock period, nsec
	int nsam;     	// total samples in the shot eg max 2M
	int nchan;    	// number of channels, usually 48
	float ESLO[NCHAN];        // SLOPE,
	float EOFF[NCHAN];        // OFFSET   volts[ch][sam] = raw[ch][sam] * ESLO[ch] + EOFF[ch]
/*
	char decims[nsamples];    // array with decimation factor in force at each sample.
	                         // T[sam] = T[sam-1] + DT*decims[sam-1]
	short chdata[NCHAN][NSAM]; // 2D array (or, concatenation of 1D arrays, raw channel data)
*/
};

void init(int argc, const char** argv) {
	poptContext opt_context =
			poptGetContext(argv[0], argc, argv, opt_table, 0);

	while ( (rc = poptGetNextOpt( opt_context )) >= 0 ){
		switch(rc){
		default:
		}
	}
}

void offload(void) {

}

int main(int argc, const char** argv)
{
	init(argc, argv);
	offload();
	return 0;
}
