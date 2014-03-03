/* ------------------------------------------------------------------------- *
 * is_ramp.cpp  		                     	                    
 * ------------------------------------------------------------------------- *
 *   Copyright (C) 2014 Peter Milne, D-TACQ Solutions Ltd                
 *                      <peter dot milne at D hyphen TACQ dot com>          
 *                         www.d-tacq.com
 *   Created on: 3 Mar 2014  
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




#include <stdio.h>


int main(int argc, char* argv[])
{
	unsigned uu;
	unsigned uu1;
	int ii = 0;
	int ecount = 0;

	fread(reinterpret_cast<char*>(&uu1), sizeof(uu), 1, stdin);

	for (; fread(reinterpret_cast<char*>(&uu), sizeof(uu), 1, stdin) == 1;
								++ii, uu1 = uu){
		if (uu != uu1+1){
			fprintf(stderr, "ERROR at %d %08x + 1 != %08x\n",
				ii, uu1, uu);
			++ecount;
		}
	}

	return ecount;
}
