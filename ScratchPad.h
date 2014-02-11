/* ------------------------------------------------------------------------- *
 * ScratchPad.h  		                     	                    
 * ------------------------------------------------------------------------- *
 *   Copyright (C) 2013 Peter Milne, D-TACQ Solutions Ltd                
 *                      <peter dot milne at D hyphen TACQ dot com>          
 *                         www.d-tacq.com
 *   Created on: 5 Nov 2013  
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


#ifndef SCRATCHPAD_H_
#define SCRATCHPAD_H_


/** singleton .. */
class Scratchpad {
	char *root;
	Scratchpad(int site) {
		root = new char[80];
		snprintf(root, 80, "/dev/acq400.%d.knobs", site);
	}
public:
	enum SP_INDEX {
		SP_SAMPLE_COUNT,
		SP_MUX_STATUS,
		SP_MUX_CH01,
		SP_MUX_CH02,
		SP_AWG_G1,
		SP_AWG_G2,
		SP_AWG_G3,
		SP_AWG_G4,
	};
	enum SP_STATUS {
		SP_MUX_STATUS_BUSY = 0xb5b5b5b5,
		SP_MUX_STATUS_DONE = 0xd00e0000
	};
	static Scratchpad& instance(int site) {
		static Scratchpad* instances[7];	/* index from 1 */
		assert(site ==0);


		if (!instances[site]){
			instances[site] = new Scratchpad(site);
		}
		return *instances[site];
	}

	virtual void set(enum SP_INDEX idx, u32 value){
		char knob[80];
		snprintf(knob, 80, "%s/spad%d", root, idx);
		FILE* fp = fopen(knob, "w");
		if (fp == 0){
			perror(knob);
			exit(1);
		}
		fprintf(fp, "%08x", value);
		fclose(fp);
	}
};

#endif /* SCRATCHPAD_H_ */
