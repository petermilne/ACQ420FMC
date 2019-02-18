/* ------------------------------------------------------------------------- */
/* watchdog_PIL.cpp watchdog Process In The Looop 			     */
/* ------------------------------------------------------------------------- */
/* acq400_sls.c  D-TACQ ACQ400 FMC  DRIVER
 * Project: ACQ420_FMC
 * Created: 27 Mar 2016  			/ User: pgm
 * acq400_sls --sls [start,stride,length] [CH-DEF]
 * ------------------------------------------------------------------------- *
 *   Copyright (C) 2016 Peter Milne, D-TACQ Solutions Ltd         *
 *                      <peter dot milne at D hyphen TACQ dot com>           *
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
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.                *
 *
 * TODO
 * TODO
\* ------------------------------------------------------------------------- */

/*
 *  example:
 *  watchdog_PIL -s 10 -- ping -c1 server
 *  .. run ping every 10s, if status is good, refresh the wdt, else let it die.
 *  If the process takes a long time, sleep can be zero, the process creates the delay
 *  If the process is quick, then add a sleep to reduce system load.
 *  defaults:
 *  -s 10 true
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h> // sigaction(), sigsuspend(), sig*()
#include <unistd.h> // alarm()
#include <fcntl.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/wait.h>

#include "popt.h"


namespace G {
	int sleep = 10;
	int verbose = 0;
	const char** proc;
}
struct poptOption opt_table[] = {
	{ "sleep", 	's', POPT_ARG_INT, &G::sleep, 0, "sleep in seconds" },
	{ "verbose", 	'v', POPT_ARG_INT, &G::verbose, 0, "" },
	POPT_AUTOHELP
	POPT_TABLEEND
};



int cli(int argc, const char* argv[])
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

	G::proc = poptGetArgs(opt_context);
	if (G::proc == NULL){
		static const char* default_proc[] = { "true", NULL };
		G::proc = default_proc;
	}
	return 0;
}

class Watchdog {
	int fd;
public:
	Watchdog() {
		//fd = open("/dev/nowhere", O_WRONLY);
		fd = open("/dev/watchdog0", O_WRONLY);
		if (fd == -1){
			perror("/dev/watchdog0");
			exit(1);
		}
	}
	~Watchdog() {
		close(fd);
	}
	void operator() () {
		write(fd, "1", 1);
	}
};

int run_proc()
{
	pid_t p = fork();
	if ( p == -1 ) {
		perror("fork failed");
		return EXIT_FAILURE;
	}
	else if ( p == 0 ) {
		execvp(G::proc[0], (char* const*)G::proc);
		return EXIT_FAILURE;
	}

	int status;
	if ( waitpid(p, &status, 0) == -1 ) {
		perror("waitpid failed");
		return EXIT_FAILURE;
	}

	if ( WIFEXITED(status) ) {
		const int es = WEXITSTATUS(status);
		printf("exit status was %d\n", es);
		return es;
	}else{
		return -1;
	}
}
int run(void)
{
	Watchdog w;

	while(1){
		int rc = run_proc();
		if (rc != 0){
			fprintf(stderr, "run_proc ERROR %d, GOING DOWN!", rc);
			exit(1);
		}
		if (G::sleep){
			sleep(G::sleep);
		}
		w();
	}
}

int main(int argc, const char* argv[])
{
	cli(argc, argv);
	run();
}
