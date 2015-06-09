/* ------------------------------------------------------------------------- *
 * transition_counter.cpp  		                     	                    
 * ------------------------------------------------------------------------- *
 *   Copyright (C) 2014 Peter Milne, D-TACQ Solutions Ltd                
 *                      <peter dot milne at D hyphen TACQ dot com>          
 *                         www.d-tacq.com
 *   Created on: 12 Apr 2014  
 *    Author: pgm                                                         
 *                                                                           *
 *  This program is free software; you can redistribute it and/or modify     *
 *  it under the terms of Version 2 of the GNU General Public License        *
 *  as published by the Free Software Foundation;                            *
 *                                                                           *
 *  This program is distributed in the hope that it will be useful,          *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            *
 *  GNU General Public License for more details.                             *
 *                                                                           *
 *  You should have received a copy of the GNU General Public License        *
 *  along with this program; if not, write to the Free Software              *
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.                */
/* ------------------------------------------------------------------------- */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define NCHAN	1
#define MAXSAM	10000000

#define THRESHOLD 10	/* a jump this fraction of max-min is an EDGE */
#define DELTA_SAM 4	/* look back this many samples .. slow edges .. */

long long sum[NCHAN];
long long sumsq[NCHAN];
long long nn = 0;
int report_num;

struct Stats {
	float mean;
	float stddev;
	int max;
	int min;
} stats = {
	0, 0, -32768, 32767
};

enum Edge { FALLING = -1, NEUTRAL, RISING };

#define C_FALLING '\\'
#define C_RISING  '/'
#define C_FLAT	  '-'

static int debug = 0;


template <class T> 
class TransitionCounter {
	T* buf;
public:
void compute(int nsam)
{
	long long sum = 0;
	long long sumsq = 0;
	int ii;

	for (ii = 0; ii < nsam; ++ii){
		T xx = buf[ii];
		if (xx < stats.min) stats.min = xx;
		if (xx > stats.max) stats.max = xx;
		sum += xx;
		sumsq += (long long)xx * xx;
	}

	printf("%10s %10s %10s %10s %10s\n",
			"nsam", "mean", "stddev", "min", "max");
	printf("%10lld ", nsam);
	stats.mean = sum/nsam;
	stats.stddev = sqrt((sumsq - sum*sum/nsam)/nsam);
	printf("%10.2f %10.2f", stats.mean, stats.stddev);
	printf("%10d %10d\n", stats.min, stats.max);
}

static void _report(int ii, T xm1, T xx, enum Edge edge)
{

	printf("%10d %6d %+8d %6d\n", ii, xm1, edge*9999999, xx);
}
void report_transitions(int nsam)
{
	int threshold = (stats.max - stats.min)/THRESHOLD;
	Edge state = NEUTRAL;
	for (int ii = DELTA_SAM; ii < nsam; ++ii){
		int xx = buf[ii];
		int xm1 = buf[ii-DELTA_SAM];
		if (debug){
			char c_state = state==RISING? C_RISING: state==FALLING? C_FALLING: C_FLAT;
			printf("%10d xm1=%d xx=%d threshold=%d %c %s\n",
				ii, xm1, xx, threshold, c_state,
				abs(xx-xm1) > threshold? "OVER":"");
		}
		if (abs(xx-xm1) > threshold){
			if (xx-xm1 > 0){
				if (state != RISING){
					_report(ii, xm1, xx, state = RISING);
				}
			}else{
				if (state != FALLING){
					_report(ii, xm1, xx, state = FALLING);
				}
			}
		}else{
			state = NEUTRAL;
		}
	}
}
TransitionCounter() : buf(new T[MAXSAM]) {
	const char* chdef = getenv("TRANSITION_COUNTER_CH");
	int nsam;

	if (chdef){
		int ch, nchan;
		if (sscanf(chdef, "%d,%d", &ch, &nchan) == 2){
			if (ch < 1){
				fprintf(stderr, "ERROR:ch >=1\n");
				exit(1);
			}else if (ch > nchan){
				fprintf(stderr, "ERROR:ch < nchan\n");
				exit(1);
			}else{
				fprintf(stderr, "assume MUX data NCHAN=%d CH=%d\n", nchan, ch);
			}
			T* mbuf = new T[MAXSAM*nchan];
			int nw = fread(buf, sizeof(T), MAXSAM*nchan, stdin);
			nsam = nw/nchan;
			int ix = ch-1;

			for (int isam = 0; isam < nsam; ++isam){
				buf[isam] = mbuf[isam*nchan + ix];
			}	
			delete [] mbuf;
		}else{
			fprintf(stderr, "ERROR:TRANSITION_COUNTER_CH=ch,nchan\n");
			exit(1);
		}
	}else{
		nsam = fread(buf, sizeof(T), MAXSAM, stdin);
	}
	compute(nsam);
	report_transitions(nsam);
}
virtual ~TransitionCounter() {
	delete [] buf;
}

};

int main(int argc, char* argv[])
{
	int sz = 2;
	if (getenv("TRANSITION_COUNTER_SZ")){
		sz = atoi(getenv("TRANSITION_COUNTER_SZ"));
	}
	if (sz == 4){
		new TransitionCounter<long>();
	}else{
		new TransitionCounter<short>();
	}
}

