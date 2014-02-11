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
	int close() {
		int rc = 0;
		if (fp) {
			rc = pclose(fp);
			fp = 0;
		}
		return rc;
	}
	virtual ~Pipe() {
		if (close() == -1){
			perror("pclose");
		}
	}
};

class Validator {
public:
	virtual ~Validator() {}
	virtual bool isValid(char* buf, int maxbuf, const char* args) = 0;
};

class NullValidator : public Validator
/* singleton */
{
	NullValidator() {}
public:
	virtual bool isValid(char* buf, int maxbuf, const char* args) {
		return true;
	}

	static Validator* instance() {
		static Validator* _instance;
		if (!_instance){
			return _instance = new NullValidator;
		}else{
			return _instance;
		}
	}
};

class NumericValidator : public Validator {
	int rmin, rmax;
	NumericValidator(int _rmin, int _rmax) : rmin(_rmin), rmax(_rmax) {

	}
public:
	virtual bool isValid(char* buf, int maxbuf, const char* args) {
		int tv;
		if (sscanf(args, "%d", &tv) != 1 ){
			snprintf(buf, maxbuf,
				"ERROR: NumericValidator %s not numeric", args);
			return false;
		}else{
			bool ok = tv >= rmin && tv <= rmax;
			if (!ok){
				snprintf(buf, maxbuf,
				"ERROR: NumericValidator %d not in range %d,%d",
					tv, rmin, rmax);
			}
			return ok;
		}
	}

	static Validator* create(const char* def){
		const char* ndef;
		if (ndef = strstr(def, "numeric=")){
			int _rmin, _rmax;
			if (sscanf(ndef, "numeric=%d,%d", &_rmin, &_rmax) == 2){
				return new NumericValidator(_rmin, _rmax);
			}
		}

		return 0;
	}
};
class Knob {

protected:
	Knob(const char* _name) {
		name = new char[strlen(_name)+1];
		strcpy(name, _name);
		validator = NullValidator::instance();
	}
	char* name;
	Validator *validator;

	void cprint(const char* ktype) {
		printf("%8s %s\n", ktype, name);
	}
	bool isValid(char* buf, int maxbuf, const char* args){
		return validator->isValid(buf, maxbuf, args);
	}

public:
	virtual ~Knob() {
		delete [] name;
	}



	char* getName() { return name; }
	virtual const char* getAttr() {
		return "";
	}
	/* return >0 on success, <0 on fail */
	virtual int set(char* buf, int maxbuf, const char* args) = 0;
	virtual int get(char* buf, int maxbuf) = 0;
	virtual void print(void) { cprint("Knob"); }


	static Knob* create(const char* _name, mode_t mode);

	static int match(const char* name, const char* key) {
		if (strcmp(name, key) == 0){
			return 1;
		}else if (fnmatch(key, name, 0) == 0){
			return -1;
		}else{
			return 0;
		}
	}
};

class KnobRO : public Knob {
protected:

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
	virtual const char* getAttr() {
		return "r";
	}
};

class KnobRW : public KnobRO {
protected:

public:
	KnobRW(const char* _name) : KnobRO(_name) {
	}

	virtual int set(char* buf, int maxbuf, const char* args) {
		if (!isValid(buf, maxbuf, args)){
			return -1;
		}
		File knob(name, "w");
		if (knob.fp == NULL){
			return -snprintf(buf, maxbuf, "ERROR: failed to open \"%s\"\n", name);
		}else{
			return fputs(args, knob.fp) > 0? 0: -snprintf(buf, maxbuf, "ERROR:");
		}
	}
	virtual void print(void) { cprint ("KnobRW"); }
	virtual const char* getAttr() {
			return "rw";
	}
};


class KnobX : public Knob {
	int runcmd(const char* cmd, char* buf, int maxbuf);
protected:

public:
	KnobX(const char* _name) : Knob(_name) {}

	virtual int set(char* buf, int maxbuf, const char* args) {
		if (!isValid(buf, maxbuf, args)){
			return -1;
		}
		char cmd[128];
		snprintf(cmd, 128, "%s %s", name, args);
		return runcmd(cmd, buf, maxbuf);

	}
	virtual int get(char* buf, int maxbuf) {
		char cmd[128];
		snprintf(cmd, 128, "%s ", name);		// @@todo no trailing space, no work
		return runcmd(cmd, buf, maxbuf);
	}
	virtual void print(void) { cprint("KnobX"); }
	virtual const char* getAttr() {
			return "rwx";
	}
};

int KnobX::runcmd(const char* cmd, char* buf, int maxbuf){
	Pipe knob(cmd, "r");
	if (knob.fp == NULL) {
		return -snprintf(buf, maxbuf,
				"ERROR: failed to open \"%s\"\n", name);
	}
	char* cursor;
	for (char* cursor = buf;
		(cursor = fgets(cursor, maxbuf-(cursor-buf), knob.fp)) != NULL;
		cursor += strlen(cursor)){
		;
	}
	if (knob.close() == -1){
		return cursor-buf > 0? 1:
			-snprintf(buf, maxbuf, "ERROR on close");
	}
	return 1;

}



#define HASX(mode) 	(((mode)&(S_IXUSR|S_IXGRP|S_IXOTH)) != 0)
#define HASW(mode)	(((mode)&(S_IWUSR|S_IWGRP|S_IWOTH)) != 0)

Knob* Knob::create(const char* _name, mode_t mode)
{
	if (HASX(mode)){
		return new KnobX(_name);
	}else if (HASW(mode)){
		Knob* knob = new KnobRW(_name);
		char cmd[128];
		char reply[128];
		sprintf(cmd, "grep %s /usr/share/doc/numerics", _name);
		Pipe pn(cmd, "r");
		if (fgets(reply, 128, pn.fp)){
			Validator* v = NumericValidator::create(reply);
			if (v){
				knob->validator = v;
			}
		}
		return knob;
	}else{
		return new KnobRO(_name);
	}
}

class Prompt: public Knob
/* singleton */
{
	static bool enabled;

	Prompt(): Knob("prompt") {

	}
public:

	char* getName() { return name; }

	/* return >0 on success, <0 on fail */
	virtual int set(char* buf, int maxbuf, const char* args) {
		if (strcmp(args, "on") == 0){
			enabled = 1;
		}else if (strcmp(args, "off") == 0){
			enabled = 0;
		}
		return 1;
	}
	virtual int get(char* buf, int maxbuf) {
		return snprintf(buf, maxbuf, "%s", enabled? "on": "off");
	}
	virtual void print(void) { cprint("Knob"); }

	static void prompt();

	static class Knob* create() {
		static Knob* _instance;
		if (!_instance){
			return _instance = new Prompt;
		}else{
			_instance;
		}
	}
};

bool Prompt::enabled;




vector<Knob*> KNOBS;
typedef vector<Knob*>::iterator VKI;

/*
for file in $(ls -1)
do
        echo $file:
        HTEXT="$(grep -m1 ^$file $HROOT/acq400_help* | cut -f2-)"
        if [ $? -eq 0 ]; then
                echo "  $HTEXT";
        else
                echo $file;
        fi
done
*/

#define HROOT "/usr/share/doc"

class Help: public Knob {

protected:
	virtual int query(Knob* knob){
		printf("%s\n", knob->getName());
		return 1;
	}
public:
	Help() : Knob("help") {}
	Help(const char* _key) : Knob(_key) {}
	virtual int get(char* buf, int maxbuf) {
		for (VKI it = KNOBS.begin(); it != KNOBS.end(); ++it){
			query(*it);
		}
		return 1;
	}
	virtual int set(char* buf, int maxbuf, const char* args) {
		for (VKI it = KNOBS.begin(); it != KNOBS.end(); ++it){
			if (Knob::match((*it)->getName(), args)){
				query(*it);
			}
		}
		return 1;
	}
};

class Help2: public Help {
protected:
	virtual int query(Knob* knob){
		char cmd[128];
		char reply[128];
		sprintf(cmd, "grep -m1 ^%s %s/acq400_help* | cut -f2 -",
					knob->getName(), HROOT);
		Pipe grep(cmd, "r");
		if (fgets(reply, 128, grep.fp)){
			printf("%-20s : %s\n\t%s",
				knob->getName(), knob->getAttr(), reply);
		}
		return 1;
	}
public:
	Help2() : Help("help2") {}
};
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
	KNOBS.push_back(Prompt::create());
	KNOBS.push_back(new Help);
	KNOBS.push_back(new Help2);

	free(namelist);

	char newpath[1024];
	snprintf(newpath, 1023, "%s:%s", get_current_dir_name(), getenv("PATH"));

	setenv("PATH", newpath, 1);
}

char* chomp(char *str) {
	char* cursor = str + strlen(str)-1;
	while (*cursor == '\n' && cursor >= str){
		*cursor = '\0';
	}
	return str;
}

bool err;
int site;



void Prompt::prompt() {
	if (enabled){
		printf("acq400.%d %d >", site, err);
	}else if (err){
		;
	}
	fflush(stdout);
}


void cli(int argc, char* argv[])
{
	char *dir;
	char dbuf[128];
	char sname[32];

	if (argc > 2){
		dir = argv[2];
	}else if (argc > 1){
		site = atoi(argv[1]);
		sprintf(dbuf, "/etc/acq400/%d", site);
		dir = dbuf;
		sprintf(sname, "%d", site);
	}else{
		return;
	}
	setenv("SITE", sname, 0);
	chdir(dir);
}
int main(int argc, char* argv[])
{
	cli(argc, argv);
	do_scan();
	char* ibuf = new char[128];
	char* obuf = new char[4096];


	for (; fgets(ibuf, 128, stdin); Prompt::prompt()){
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
			obuf[0] = '\0';
			switch(Knob::match(knob->getName(), key)){
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
