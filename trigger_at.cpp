/*
 * trigger_at.cpp
 *
 *  Created on: 25 Sep 2020
 *      Author: pgm
 *
 *  trigger_at <time_t>
 *
 *  schedule a trigger for time_t in background
 *
 *  creates:
 *  /var/run/trigger_at.pid PID file if we have to kill the trigger routine
 *  format: pid  t2 t1 usecs_sleep (tuned usleep value)
 *  logger item
 *  format: pid t2 t1 usecs_residue (measure of time after t1 that the trigger went out)
 *
 *  trigger_at +secs       # delta time, really a test mode, not suitable for multi uut (think why not)
 *  trigger_at q           # query
 *  trigger_at k           # kill
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "popt.h"
#include <sys/time.h>
#include <sys/types.h>
#include <signal.h>

#include "local.h"
#include "Env.h"
#include "File.h"
#include "Knob.h"


#define MAXWAIT	3600
#define MINWAIT 2

#define RADCELF_SITE 2

#define PIDF  "/var/run/trigger_at.pid"
namespace G {
	int site = Env::getenv("SITE", RADCELF_SITE);
	const char* tknob = "dds_gps_arm_pps";
}

struct poptOption opt_table[] = {
	{
		"tknob", 0, POPT_ARG_INT, &G::tknob, 0, "trigger knob"
	},
	POPT_AUTOHELP
	POPT_TABLEEND
};

int report_job()
{
	FILE* fp = fopen(PIDF, "r");
	if (fp){
		char buf[80];
		fgets(buf, 80, fp);
		fclose(fp);
		printf(buf);
	}else{
		printf("nothing to report\n");
	}
	return 0;
}

void _write_pid(FILE* pf, pid_t pid, time_t t2, time_t tnow, unsigned usecs)
{
	fprintf(pf, "%u ;# %ld %ld %u\n", pid, t2, tnow, usecs);
}
void _set_pidf(pid_t pid, time_t t2, time_t tnow, unsigned usecs)
{
	FILE* pf = fopen(PIDF, "w");
	_write_pid(pf, pid, t2, tnow, usecs);
	fclose(pf);
}

void _set_log(pid_t pid, time_t t2, time_t tnow, unsigned usecs)
{
	FILE *pf = popen("/usr/bin/logger -t trigger_at", "w");
	_write_pid(pf, pid, t2, tnow, usecs);
	pclose(pf);
}
time_t _gettimeofday(unsigned *usecs = 0)
{
	struct timeval tv;
	int rc = gettimeofday(&tv, NULL);
	if (rc == -1){
		perror("gettimeofday");
		exit(1);
	}
	if (usecs){
		*usecs = tv.tv_usec;
	}
	return tv.tv_sec;
}

int _kill_job()
{
	FILE* fp = fopen(PIDF, "r");
	if (fp == 0){
		return 0;    // gone already, no worries
	}else{
		pid_t pid;
		if (fscanf(fp, "%u", &pid) == 1){
			int rc = kill(pid, SIGKILL);
			if (rc != 0){
				perror("kill");
				exit(-1);
			}
		}else{
			unlink(PIDF);
		}
		fclose(fp);
		return 0;
	}
}

int kill_job()
{
	printf("kill_job\n");
	return _kill_job();
	return 0;
}

void wait_for(time_t t2)
{
	unsigned usecs_late;
	time_t t1;

	while((t1 = _gettimeofday(&usecs_late)) < t2 - 1){
		unsigned usecs_adj = 1000000 - usecs_late;
		_set_pidf(getpid(), t2, t1, usecs_adj);
		usleep(usecs_adj);
	}
	Knob k(G::site, G::tknob);
	k.set(1);
	_set_log(getpid(), t2, t1, usecs_late);
	usleep(1000000);
	k.set(0);
	unlink(PIDF);
}
int schedule(time_t t2)
{
	pid_t cpid;

	Knob(G::site, G::tknob).set(0);

	if ((cpid = fork()) == 0){
		wait_for(t2);
	}else{
		_set_pidf(cpid, t2, _gettimeofday(), 0);
	}
	return 0;
}

int wait_is_busy()
{
	return _kill_job();
}
int _validate(unsigned long delta)
{
	wait_is_busy();
	if (delta < MINWAIT){
		printf("sorry too soon\n");
		return -1;
	}else if (delta > MAXWAIT){
		printf("sorry max delay 3600\n");
		return -1;
	}else{
		return schedule(_gettimeofday() + delta);
	}
}
int _validate(const char* message)
{
	wait_is_busy();
	time_t t2 = strtoul(message, 0, 0);
	time_t t1 = _gettimeofday(0);
	if (t2 <= t1 + 2){
		printf("sorry, too close\n");
		return -1;
	}else{
		return schedule(t2);
	}
}
int validate(const char* message)
{
	if (message == 0){
		return report_job();
	}
	switch(message[0]){
	case 'q':
		return report_job();
	case 'k':
		return kill_job();
	case '+':
		return _validate(strtoul(message+1, 0, 0));
	default:
		return _validate(message);
	}
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





