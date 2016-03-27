/* ------------------------------------------------------------------------- */
/* File.h  D-TACQ ACQ400 FMC  DRIVER                                   
 * Project: ACQ420_FMC
 * Created: 5 Mar 2016  			/ User: pgm
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
/* ------------------------------------------------------------------------- */

#ifndef FILE_H_
#define FILE_H_

#include <fcntl.h>
#include <stdio.h>
#include <sys/mman.h>

#include <string>


class File {
	FILE *_fp;

	void _file(const char *fname, const char* mode, bool check = true) {
		_fp = fopen(fname, mode);
		if (check && _fp == 0){
			perror(fname);
			exit(1);
		}
	}
public:
	static const bool NOCHECK = false;

	File(const char *fname, const char* mode, bool check = true){
		_file(fname, mode, check);
	}
	File(const char* path, const char *fname, const char* mode, bool check = true){
		char* buf = new char[strlen(path)+1+strlen(fname)+1];
		strcpy(buf, path);
		strcat(buf, "/");
		strcat(buf, fname);
		_file(buf, mode, check);
		delete [] buf;
	}
	~File() {
		if (_fp) fclose(_fp);
	}
	FILE* fp() {
		return _fp;
	}
	int fd() {
		return fileno(_fp);
	}
	FILE* operator() () {
		return _fp;
	}
};


template <class T>
class Mapping {
	int len;
	T* _data;
	int fd;
public:
	Mapping(std::string fname, int _len) : len(_len) {
		int fd = open(fname.data(), O_RDWR, 0777);
		_data = static_cast<T*>(mmap(0, len,
			PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0));
		if (_data == MAP_FAILED){
			perror(fname.data());
			exit(1);
		}
	}
	~Mapping() {
		munmap(_data, len);
		close(fd);
	}
	const T* operator() () {
		return _data;
	}
};


#endif /* FILE_H_ */
