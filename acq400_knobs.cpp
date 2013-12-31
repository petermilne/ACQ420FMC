/* ------------------------------------------------------------------------- *
 * acq400_knobs.cpp  		                     	                    
 * ------------------------------------------------------------------------- *
 *   Copyright (C) 2013 Peter Milne, D-TACQ Solutions Ltd                
 *                      <peter dot milne at D hyphen TACQ dot com>          
 *                         www.d-tacq.com
 *   Created on: 30 Dec 2013  
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
What does acq400_knobs do?.

- r : readable
- w : writable
- x : executes it
- wildcard query
- help
- help2

- ranges wbn too.

- load the directory at start. precompute all status.

- .ranges file?.
*/


#define _GNU_SOURCE
#include <dirent.h>
#include <fnmatch.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <vector>

const char* pattern = "";

using namespace std;

class File {
public:
	FILE *fp;
	File(const char* name, const char* mode = "r") {
		fp = fopen(name, mode);
	}
	virtual ~File() {
		fclose(fp);
	}
};

class Pipe {
public:
	FILE* fp;
	Pipe(const char* name, const char* mode = "r") {
		fp = popen(name, mode);
	}
	virtual ~Pipe() {
		int rc = pclose(fp);
		if (rc == -1){
			perror("pclose");
		}
	}
};
class Knob {

protected:
	Knob(const char* _name) {
		name = new char[strlen(_name)+1];
		strcpy(name, _name);
	}
	char* name;

	void cprint(const char* ktype) {
		printf("%8s %s\n", ktype, name);
	}
public:
	virtual ~Knob() {
		delete [] name;
	}


	char* getName() { return name; }

	/* return >0 on success, <0 on fail */
	virtual int set(char* buf, int maxbuf, const char* args) = 0;
	virtual int get(char* buf, int maxbuf) = 0;
	virtual void print(void) { cprint("Knob"); }


	static Knob* create(const char* _name, mode_t mode);
};

class KnobRO : public Knob {
public:
	KnobRO(const char* _name) : Knob(_name) {}

	virtual int set(char* buf, int maxbuf, const char* args) {
		return -snprintf(buf, maxbuf, "ERROR: \"%s\" is read-only", name);
	}
	virtual int get(char* buf, int maxbuf) {
		File knob(name, "r");
		if (knob.fp == NULL) {
			return snprintf(buf, maxbuf, "ERROR: failed to open \"%s\"\n", name);
		}else{
			return fgets(buf, maxbuf, knob.fp) != NULL;
		}
	}
	virtual void print(void) { cprint("KnobRO"); }
};

class KnobRW : public KnobRO {
public:
	KnobRW(const char* _name) : KnobRO(_name) {
	}

	virtual int set(char* buf, int maxbuf, const char* args) {
		File knob(name, "w");
		if (knob.fp == NULL){
			return -snprintf(buf, maxbuf, "ERROR: failed to open \"%s\"\n", name);
		}else{
			return fputs(args, knob.fp) > 0? 0: -snprintf(buf, maxbuf, "ERROR:");
		}
	}
	virtual void print(void) { cprint ("KnobRW"); }
};

class KnobX : public Knob {
public:
	KnobX(const char* _name) : Knob(_name) {}

	virtual int set(char* buf, int maxbuf, const char* args) {
		char cmd[128];
		snprintf(cmd, 128, "%s %s", name, args);
		Pipe knob(cmd, "r");
		if (knob.fp == NULL) {
			return -snprintf(buf, maxbuf, "ERROR: failed to open \"%s\"\n", name);
		}else{
			return fgets(buf, maxbuf, knob.fp) != NULL? 1:
					-snprintf(buf, maxbuf, "ERROR");;
		}
	}
	virtual int get(char* buf, int maxbuf) {
		char cmd[128];
		snprintf(cmd, 128, "%s ", name);		// @@todo no trailing space, no work
		Pipe knob(cmd, "r");
		if (knob.fp == NULL) {
			return -snprintf(buf, maxbuf, "ERROR: failed to open \"%s\"\n", name);
		}else{
			return fgets(buf, maxbuf, knob.fp) != NULL? 1:
					-snprintf(buf, maxbuf, "ERROR");
		}
	}
	virtual void print(void) { cprint("KnobX"); }
};

#define HASX(mode) 	(((mode)&(S_IXUSR|S_IXGRP|S_IXOTH)) != 0)
#define HASW(mode)	(((mode)&(S_IWUSR|S_IWGRP|S_IWOTH)) != 0)

Knob* Knob::create(const char* _name, mode_t mode)
{
	if (HASX(mode)){
		return new KnobX(_name);
	}else if (HASW(mode)){
		return new KnobRW(_name);
	}else{
		return new KnobRO(_name);
	}
}


vector<Knob*> KNOBS;
typedef vector<Knob*>::iterator VKI;

int filter(const struct dirent *dir)
{
        return fnmatch(pattern, dir->d_name, 0);
}
int do_scan()
{
	struct dirent **namelist;
	int nn = scandir(".", &namelist, filter, versionsort);

	if (nn < 0){
		perror("scandir");
		exit(1);
	}

	for (int ii = 0; ii < nn; ++ii){
		char* alias = namelist[ii]->d_name;
		struct stat sb;
		if (stat(alias, &sb) == -1){
			perror("stat");
		}else{
			if (!S_ISREG(sb.st_mode)){
				;
				//fprintf(stderr, "not a regular file:%s", alias);
			}else{
				KNOBS.push_back(Knob::create(alias, sb.st_mode));
			}
		}
	}

	free(namelist);
/*
	for (VKI it = KNOBS.begin(); it != KNOBS.end(); ++it){
		(*it)->print();
	}
*/
	char newpath[1024];
	snprintf(newpath, 1023, "%s:%s", get_current_dir_name(), getenv("PATH"));

	setenv("PATH", newpath, 1);
	printf("set PATH: %s\n", getenv("PATH"));
}

char* chomp(char *str) {
	char* cursor = str + strlen(str)-1;
	while (*cursor == '\n' && cursor >= str){
		*cursor = '\0';
	}
	return str;
}

bool prompt_enabled;
bool err;
void prompt() {
	if (prompt_enabled){
		printf("acq400 %d >", err);
		fflush(stdout);
	}else if (err){
		;
	}
}

int match(const char* name, const char* key)
{
	if (strcmp(name, key) == 0){
		return 1;
	}else if (fnmatch(key, name, 0) == 0){
		return -1;
	}else{
		return 0;
	}
}

int main(int argc, char* argv[])
{
	do_scan();
	char* ibuf = new char[128];
	char* obuf = new char[4096];


	for (; fgets(ibuf, 128, stdin); prompt()){
		char *args = 0;
		int cursor;
		char *key = 0;
		bool is_query = false;

		chomp(ibuf);

		int len = strlen(ibuf);
		if (len == 0){
			continue;
		}else{
			int isep = strcspn(ibuf, "= ");
			if (isep != strlen(ibuf)){
				args = ibuf + isep+ strspn(ibuf+isep, "= ");
				ibuf[isep] = '\0';
			}else{
				is_query = true;
			}
			key = ibuf;
		}

		bool found = false;
		for (VKI it = KNOBS.begin(); it != KNOBS.end(); ++it){
			Knob* knob = *it;
			err = false;
			int rc;
			bool is_glob = true;
			switch(match(knob->getName(), key)){
			case 0:
				continue;
			case 1:
				is_glob = false;
				it = KNOBS.end() - 1; // fall thru, drop out
			case -1:
				if (is_query){
					rc = knob->get(obuf, 4096);
					if (is_glob){
						if (!strstr(obuf, knob->getName())){
							printf("%s ", knob->getName());
						}
					}
				}else{
					rc = knob->set(obuf, 4096, args);
				}
				if (rc){
					puts(chomp(obuf));
				}
				err = rc < 0;
				found = true;
			}
		}

		if (!found){
			printf("ERROR:\%s\" not found\n", key);
		}
	}
}
