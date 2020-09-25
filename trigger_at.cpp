/*
 * trigger_at.cpp
 *
 *  Created on: 25 Sep 2020
 *      Author: pgm
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "popt.h"
#include <sys/time.h>

#include "local.h"
#include "Env.h"
#include "File.h"
#include "Knob.h"


namespace G {
	const char* group;
	int port;
}

struct poptOption opt_table[] = {
	POPT_AUTOHELP
	POPT_TABLEEND
};


int validate(const char* message)
{
	printf("time_t %d\n", sizeof(time_t));
	if (message[0] == 'q'){
		printf("job\n");
	}
	return 0;
}
int main(int argc, const char* argv[])
{
	poptContext opt_context =
			poptGetContext(argv[0], argc, argv, opt_table, 0);
	int rc;
        while ((rc = poptGetNextOpt( opt_context )) >= 0 ){
                switch(rc){
                default:
                        ;
                }
        }

        return validate(poptGetArg(opt_context));
}





