/*
 * dump_regs.c
 *
 *  Created on: 4 Sep 2019
 *      Author: pgm
 */

// replace dump_regs script: taking 3s on acq400.0


		/* enable versionsort */


#define _FILE_OFFSET_BITS 64

#include <dirent.h>
#include <fnmatch.h>
#include <stdio.h>
#include <stdlib.h>

static const char* pattern = "*0x*";

int filter(const struct dirent *dir)
{
        return fnmatch(pattern, dir->d_name, 0) == 0;
}

void scancat(const char* dir)
{
	struct dirent **namelist;
	int nn = scandir(dir, &namelist, filter, versionsort);
	for (int ii = 0; ii < nn; ++ii){
		printf( "%34s %s\n", namelist[ii]->d_name, "nocat");
	}
}

int main(int argc, char* argv[])
{
	if (getenv("DUMP_REGS_PATTERN")){
		pattern = getenv("DUMP_REGS_PATTERN");
	}

	for (int ii = 1; ii < argc; ++ii){
		scancat(argv[ii]);
	}
}

