/* ------------------------------------------------------------------------- */
/* knobs.cpp  D-TACQ ACQ400 FMC  DRIVER                                   
 * Project: ACQ420_FMC
 * Created: 1 Mar 2016  			/ User: pgm
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


#include <stdio.h>

#include "local.h"		/* chomp() hopefully, not a lot of other garbage */
#include "knobs.h"
#include "Knob.h"

#define NOMAPPING
#include "File.h"

Knob::Knob(const char* abs_path): cbuf(0)
{
	kpath = new char [strlen(abs_path)+1];
	strcpy(kpath, abs_path);
}

Knob::Knob(int site, const char* knob) : cbuf(0)
{
	const char* fmt = "/dev/acq400.%d.knobs/%s";
	int maxlen = strlen(fmt) + 3 + strlen(knob) + 1;
	kpath = new char [maxlen];
	snprintf(kpath, maxlen, fmt, site, knob);
}

Knob::~Knob()
{
	if (cbuf) delete [] cbuf;
	delete [] kpath;
}

int Knob::get(unsigned *value)
{
	File file(kpath, "r");
	return fscanf(file(), "%u", value);
}
int Knob::get(char *value)
{
	File file(kpath, "r");
	return fscanf(file(), "%s", value);
}

const char* Knob::operator() (void) {
	if (!cbuf){
		cbuf = new char[128];
	}
	get(cbuf);
	return cbuf;
}

int Knob::set(int value)
{
	File file(kpath, "w");
	return fprintf(file(), "%d\n", value);
}
int Knob::set(const char* value)
{
	File file(kpath, "w");
	return fprintf(file(), "%s\n", value);
}
int Knob::setX(unsigned value)
{
	File file(kpath, "w");
	return fprintf(file(), "%x\n", value);
}

int getKnob(int idev, const char* knob, unsigned* value)
{
	char kpath[128];
	if (knob[0] == '/'){
		strncpy(kpath, knob, 128);
	}else{
		snprintf(kpath, 128, "/dev/acq400.%d.knobs/%s", idev, knob);
	}
	FILE *fp = fopen(kpath, "r");
	if (fp){
		int rc = fscanf(fp, "%u", value);
		fclose(fp);
		return rc;
	} else {
		return -1;
	}
}

int getKnob(int idev, const char* knob, char* value)
{
	char kpath[128];
	if (knob[0] == '/'){
		strncpy(kpath, knob, 128);
	}else{
		snprintf(kpath, 128, "/dev/acq400.%d.knobs/%s", idev, knob);
	}
	FILE *fp = fopen(kpath, "r");
	if (fp){
		int rc = fscanf(fp, "%s", value);
		fclose(fp);
		return rc;
	} else {
		return -1;
	}
}

int setKnob(int idev, const char* knob, const char* value)
{
	char kpath[128];
	if (knob[0] == '/'){
		strncpy(kpath, knob, 128);
	}else{
		snprintf(kpath, 128, "/dev/acq400.%d.knobs/%s", idev, knob);
	}
	FILE *fp = fopen(kpath, "w");
	if (fp){
		int rc = fprintf(fp, "%s\n", value);
		fclose(fp);
		return rc;
	} else {
		return -1;
	}
}

int setKnob(int idev, const char* knob, int value)
{
	char vx[32]; snprintf(vx, 32, "%d", value);
	return setKnob(idev, knob, vx);
}


