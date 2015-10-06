/* ------------------------------------------------------------------------- *
 * acq400_lists.h  		                     	                    
 * ------------------------------------------------------------------------- *
 *   Copyright (C) 2014 Peter Milne, D-TACQ Solutions Ltd                
 *                      <peter dot milne at D hyphen TACQ dot com>          
 *                         www.d-tacq.com
 *   Created on: 9 Mar 2014  
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

/* all functions call mutex_lock(&adev->list_mutex) so clients don't have to */

#ifndef ACQ400_LISTS_H_
#define ACQ400_LISTS_H_

/* AI ops */
struct HBM * getEmpty(struct acq400_dev* adev);
void putFull(struct acq400_dev* adev);
struct HBM * getEmptyFromRefills(struct acq400_dev* adev);
int getFull(struct acq400_dev* adev, struct HBM **first);
void putEmpty(struct acq400_dev* adev);

/* AO ops */

struct HBM *ao_getEmpty(struct acq400_dev* adev);
void ao_putFull(struct acq400_dev* adev, struct HBM* hbm_full);
struct HBM *ao_getFull(struct acq400_dev* adev);
void ao_putEmpty(struct acq400_dev* adev, struct HBM* hbm_empty);

void move_list_to_empty(struct acq400_dev *adev, struct list_head* elist);
void move_list_to_stash(struct acq400_dev *adev, struct list_head* elist);

int reserve(struct acq400_path_descriptor* pd, int ibuf);
int replace(struct acq400_path_descriptor* pd, int ibuf);
void replace_all(struct acq400_path_descriptor* pd);

void empty_lists(struct acq400_dev *adev);
#endif /* ACQ400_LISTS_H_ */
