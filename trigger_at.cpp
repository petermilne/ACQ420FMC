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
#include <fcntl.h>
#include <signal.h>
#include <time.h>

#include "local.h"
#include "Env.h"
#include "File.h"
#include "Knob.h"


#define MAXWAIT	3600
#define MINWAIT 2

#define RADCELF_SITE 2
#define DDSA_ARM_PPS "/etc/acq400/4/gps_arm_pps"
#define DDSB_ARM_PPS "/etc/acq400/5/gps_arm_pps"

#define PIDF  "/var/run/trigger_at.pid"


struct poptOption opt_table[] = {

	POPT_AUTOHELP
	POPT_TABLEEND
};

int report_job()
{
	FILE* fp = fopen(PIDF, "r");
	if (fp){
		char buf[80];
		while(fgets(buf, 80, fp)){
			printf(buf);
		}
		fclose(fp);
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
	FILE* pf = fopen(PIDF, "a");
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
		char aline[128];
		while(fgets(aline, 128, fp) && sscanf(aline, "%u", &pid) == 1){
			int rc = kill(pid, SIGKILL);
			if (rc != 0){
				perror("kill");
				exit(-1);
			}
		}
		fclose(fp);
		unlink(PIDF);
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

	if((t1 = _gettimeofday(&usecs_late)) < t2 - 10){
		sleep(t2 - t1 - 10);
	}
	while((t1 = _gettimeofday(&usecs_late)) < t2 - 1){
		unsigned usecs_adj = 1000000 - usecs_late;
		_set_pidf(getpid(), t2, t1, usecs_adj);
		usleep(usecs_adj);
	}
	Knob kA(DDSA_ARM_PPS);
	Knob kB(DDSB_ARM_PPS);
	kA.set(1); kB.set(1);
	_set_log(getpid(), t2, t1, usecs_late);
	usleep(1000000);
	kA.set(0); kB.set(0);
	unlink(PIDF);
}
int schedule(time_t t2)
{
	pid_t cpid;

	Knob(DDSA_ARM_PPS).set(0);
	Knob(DDSB_ARM_PPS).set(0);

	if ((cpid = fork()) == 0){
		int rc = setsid();
		if (rc == -1){
			perror("setsid");
			return 0;
		}
		close(0);
		open("/dev/null", O_RDONLY);
		close(1);
		open("/dev/console", O_WRONLY);
		close(2);
		open("/dev/console", O_WRONLY);
		if ((cpid = fork()) == 0){
			wait_for(t2);
		}else{
			_set_pidf(cpid, t2, _gettimeofday(), 0);
		}
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
int validate_today(struct tm trig_time, const char* at_time)
{
	printf("%s at_time %s\n", __FUNCTION__, at_time);
	int hms[3];
	int nscan;

	nscan = sscanf(at_time, "%02d:%02d:%02d", hms, hms+1, hms+2);
	switch(nscan){
	case 3:
		trig_time.tm_sec  = hms[2];	// fall thru
	case 2:
		trig_time.tm_min  = hms[1];     // fall thru
	case 1:
		trig_time.tm_hour = hms[0];
		printf("%04d:%02d:%02d %02d:%02d:%02d\n",
				trig_time.tm_year+1900, trig_time.tm_mon, trig_time.tm_mday, trig_time.tm_hour, trig_time.tm_min, trig_time.tm_sec);
		schedule(mktime(&trig_time));
		return 0;
	default:
		fprintf(stderr, "ERROR decoding \"%s\" H:M:S expected\n", at_time);
		return -1;
	}
}
int run_today(const char** times)
{
	time_t nowseconds = time(0);
	struct tm now;
	gmtime_r(&nowseconds, &now);

	printf("%04d:%02d:%02d %02d:%02d:%02d\n",
			now.tm_year+1900, now.tm_mon, now.tm_mday, now.tm_hour, now.tm_min, now.tm_sec);
	now.tm_hour = now.tm_min = now.tm_sec = 0;

	while(const char* cursor = *times++){
		if (validate_today(now, cursor) != 0){
			return 1;
		}
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
        const char* key = poptGetArg(opt_context);
        if (key == 0){
        	return validate(0);
        }else if (strcmp(key, "today") == 0){
        	return run_today(poptGetArgs(opt_context));
        }else{
        	return validate(key);
        }
}





