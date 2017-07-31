/* ------------------------------------------------------------------------- *
 * is_ramp.cpp  		                     	                    
 * ------------------------------------------------------------------------- *
 *   Copyright (C) 2014 Peter Milne, D-TACQ Solutions Ltd                
 *                      <peter dot milne at D hyphen TACQ dot com>          
 *                         www.d-tacq.com
 *   Created on: 3 Mar 2014  
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

#include "popt.h"

namespace G {
	int step = 1;
	int show_first = 0;
	int emax = 10;
};

int process()
{
	unsigned uu;
	unsigned uu1;
	int ii = 0;
	int ecount = 0;

	fread(reinterpret_cast<char*>(&uu1), sizeof(uu), 1, stdin);

	if (G::show_first){
		fprintf(stderr, "start:%08x\n", uu1);
	}
	for (; fread(reinterpret_cast<char*>(&uu), sizeof(uu), 1, stdin) == 1;
								++ii, uu1 = uu){
		if (uu != uu1+G::step){
			fprintf(stderr, "ERROR at %d %08x + %d != %08x\n",
				ii, uu1, G::step, uu);
			++ecount;
			if (ecount > G::emax){
				return -1;
			}
		}
	}

	return ecount;
}

struct poptOption opt_table[] = {
	{ "step", 's', POPT_ARG_INT, &G::step, 0, "step size" },
	{ "show-first", 'f', POPT_ARG_INT, &G::show_first, 0, "show first entry" },
	{ "emax", 'e', POPT_ARG_INT, &G::emax, 0, "max errors to accept before quitting" },
	POPT_AUTOHELP
	POPT_TABLEEND
};

void ui(int argc, const char** argv)
{
	poptContext opt_context =
			poptGetContext(argv[0], argc, argv, opt_table, 0);
	int rc;

	while ( (rc = poptGetNextOpt(opt_context)) >= 0 ){
		switch(rc){
		default:
			;
		}
	}
}

int main(int argc, const char** argv)
{
	ui(argc, argv);
	return process();
}
