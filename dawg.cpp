/* ------------------------------------------------------------------------- *
 * dawg.c++  	Digital AWG
 * ------------------------------------------------------------------------- *
 *   Copyright (C) 2013 Peter Milne, D-TACQ Solutions Ltd                
 *                      <peter dot milne at D hyphen TACQ dot com>          
 *                         www.d-tacq.com
 *   Created on: 4 Nov 2013  
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


/*
 * AWG def a series of entries
 * delta-time switchspec
 * switchspec is a glob pattern CH* in /dev/acq400.5.knobs
 * switches in spec are CLOSED, all others OPEN
 * delta-time begins 0, must be monotonic, increasing. time in msec.

 eg
  0 			 # OPEN all channels
  100  CH01.01 CH01.02   # equiv: CH01.0[12]   close CH01.01,CH01.02
  200  CH02.0[23]	 # OPEN previous channels. close CH01.02,CH01,03
  300  			 # OPEN previous channels

 */

#include <stdio.h>
#include "popt.h"
#include <list>
using namespace std;

typedef unsigned int u32;

class DawgEntry {
	u32 ch01;
	u32 ch02;
	const char* def;
	const char* glob;
	DawgEntry* prev;
	list<const char*> disable_list;
	list<const char*> enable_list;

public:
	const u32 dt;
	DawgEntry(const char* _def, DawgEntry* _prev);
	const list<const char*>& get_disables() {
		return disable_list;
	}
	const list<const char*>& get_enables() {
		return enable_list;
	}
};

FILE* seqfile;
int dry_run;
int verbose;

struct poptOption opt_table[] = {
	{ "dry-run", 'D', POPT_ARG_INT, &dry_run, 0,    "go through the motions, no action" },
	{ "verbose", 'v', POPT_ARG_INT, &verbose, 0, 	"verbose" },
	POPT_AUTOHELP
	POPT_TABLEEND
};
void ui(int argc, const char* argv[])
{
	poptContext opt_context =
			poptGetContext(argv[0], argc, argv, opt_table, 0);

	int rc;
	while ( (rc = poptGetNextOpt( opt_context )) > 0 ){
		switch(rc){
		default:
			;
		}
	}
}

void build_sequence(void)
{

}

void run_sequence(void)
{

}

int main(int argc, const char* argv[])
{
	ui(argc, argv);
	build_sequence();
	run_sequence();
	return 0;
}


