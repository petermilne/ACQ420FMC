/* ------------------------------------------------------------------------- *
 * acq400_lists.c  		                     	                    
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

#include "acq400.h"
#include "hbm.h"
#include "acq400_lists.h"

int run_buffers = 0;
module_param(run_buffers, int, 0644);
MODULE_PARM_DESC(run_buffers, "#buffers to process in continuous (0: infinity)");

struct HBM * getEmpty(struct acq400_dev* adev)
{
	if (!list_empty(&adev->EMPTIES)){
		struct HBM *hbm;
		mutex_lock(&adev->list_mutex);
		hbm = list_first_entry(
				&adev->EMPTIES, struct HBM, list);
		list_move_tail(&hbm->list, &adev->INFLIGHT);
		hbm->bstate = BS_FILLING;
		mutex_unlock(&adev->list_mutex);

		++adev->rt.nget;
		return hbm;
	} else {
		dev_warn(DEVP(adev), "get Empty: Q is EMPTY!\n");
		return 0;
	}
}

void putFull(struct acq400_dev* adev)
{
	if (!list_empty(&adev->INFLIGHT)){
		struct HBM *hbm;
		mutex_lock(&adev->list_mutex);
		hbm = list_first_entry(
				&adev->INFLIGHT, struct HBM, list);
		hbm->bstate = BS_FULL;
		list_move_tail(&hbm->list, &adev->REFILLS);
		mutex_unlock(&adev->list_mutex);

		++adev->rt.nput;
		if (run_buffers && adev->rt.nput >= run_buffers){
			adev->rt.please_stop = 1;
		}
		wake_up_interruptible(&adev->refill_ready);
	}else{
		dev_warn(DEVP(adev), "putFull: Q is EMPTY!\n");
	}
}

struct HBM * getEmptyFromRefills(struct acq400_dev* adev)
{
	if (!list_empty(&adev->REFILLS)){
		struct HBM *hbm;
		mutex_lock(&adev->list_mutex);
		hbm = list_first_entry(
				&adev->REFILLS, struct HBM, list);
		list_move_tail(&hbm->list, &adev->INFLIGHT);
		hbm->bstate = BS_FILLING;
		mutex_unlock(&adev->list_mutex);

		++adev->rt.nget;
		return hbm;
	} else {
		dev_warn(DEVP(adev), "getEmptyFromRefills: Q is EMPTY!\n");
		return 0;
	}
}
int getFull(struct acq400_dev* adev)
{
	struct HBM *hbm;
	if (wait_event_interruptible(
			adev->refill_ready,
			!list_empty(&adev->REFILLS) ||
			adev->rt.refill_error ||
			adev->rt.please_stop)){
		return -EINTR;
	} else if (adev->rt.please_stop){
		return GET_FULL_DONE;
	} else if (adev->rt.refill_error){
		return GET_FULL_REFILL_ERR;
	}

	mutex_lock(&adev->list_mutex);
	hbm = list_first_entry(&adev->REFILLS, struct HBM, list);
	list_move_tail(&hbm->list, &adev->OPENS);
	hbm->bstate = BS_FULL_APP;
	mutex_unlock(&adev->list_mutex);
	return GET_FULL_OK;
}

void putEmpty(struct acq400_dev* adev)
{
	struct HBM *hbm;
	mutex_lock(&adev->list_mutex);
	hbm = list_first_entry(&adev->OPENS, struct HBM, list);
	hbm->bstate = BS_EMPTY;
	list_move_tail(&hbm->list, &adev->EMPTIES);
	mutex_unlock(&adev->list_mutex);
}


struct HBM * ao_getEmpty(struct acq400_dev* adev)
{
	if (!list_empty(&adev->EMPTIES)){
		struct HBM *hbm;
		mutex_lock(&adev->list_mutex);
		hbm = list_first_entry(
				&adev->EMPTIES, struct HBM, list);
		list_move_tail(&hbm->list, &adev->INFLIGHT);
		hbm->bstate = BS_FILLING;
		mutex_unlock(&adev->list_mutex);

		++adev->rt.nget;
		return hbm;
	} else {
		return 0;
	}
}

void ao_putFull(struct acq400_dev* adev, struct HBM* hbm_full)
{
	mutex_lock(&adev->list_mutex);
	hbm_full->bstate = BS_FULL;
	list_move_tail(&hbm_full->list, &adev->REFILLS);
	mutex_unlock(&adev->list_mutex);
	++adev->rt.nput;
	wake_up_interruptible(&adev->w_waitq);
}

struct HBM * ao_getFull(struct acq400_dev* adev)
{
	struct HBM *hbm = 0;
	if (!list_empty(&adev->REFILLS)){
		mutex_lock(&adev->list_mutex);
		hbm = list_first_entry(&adev->REFILLS, struct HBM, list);
		list_del(&hbm->list);
		mutex_unlock(&adev->list_mutex);
	}

	return hbm;
}

void ao_putEmpty(struct acq400_dev* adev, struct HBM* hbm_empty)
{
	mutex_lock(&adev->list_mutex);
	hbm_empty->bstate = BS_EMPTY;
	list_add_tail(&hbm_empty->list, &adev->EMPTIES);
	mutex_unlock(&adev->list_mutex);
}

void move_list_to_empty(struct acq400_dev *adev, struct list_head* elist)
{
	struct HBM *cur;
	struct HBM *tmp;
	mutex_lock(&adev->list_mutex);
	list_for_each_entry_safe(cur, tmp, elist, list){
		cur->bstate = BS_EMPTY;
		list_move_tail(&cur->list, &adev->EMPTIES);
	}
	mutex_unlock(&adev->list_mutex);
}
