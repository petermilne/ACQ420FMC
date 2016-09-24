/* ------------------------------------------------------------------------- */
/* acq400_set.c  D-TACQ ACQ400 FMC  DRIVER                                   
 * Project: ACQ420_FMC
 * Created: 24 Sep 2016  			/ User: pgm
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
 * ------------------------------------------------------------------------- */

#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/pci.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/timex.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>

#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include <asm/uaccess.h>  /* VERIFY_READ|WRITE */

#include "lk-shim.h"


#include <linux/device.h>
#include <linux/module.h>
#include <linux/user.h>

#include "acq400.h"

extern int good_sites[];
extern int good_sites_count;


int acq400_get_site(struct acq400_dev *adev, char s)
{
	int ii;
	int site = s-'0';

	for(ii = 0; ii < good_sites_count; ++ii){
		if (site == good_sites[ii]){
			if (IS_ACQ1001SC(adev) && site == 4){
				/* @@todo site 3/4 frig */
				return 3;
			}else{
				return site;
			}
		}
	}

	return -1;
}

int acq400_add_set(struct acq400_dev* set[], struct acq400_dev *adev, int site)
{
	int ia = 0;
	struct acq400_dev *slave = acq400_lookupSite(site);
	for (ia = 0; ia < MAXSITES; ++ia){
		dev_dbg(DEVP(adev), "add_set [%d]=%p site:%d %p",
				ia, set[ia], site, slave);
		if (set[ia] == slave){
			return 0;
		}else if (set[ia] == 0){
			set[ia] = slave;
			return 0;
		}
	}
	dev_err(DEVP(adev), "ERROR: add_set() failed");
	return 1;
}

void acq400_clear_set(struct acq400_dev* set[])
{
	int ia = 0;
	for (ia = 0; ia < MAXSITES; ++ia){
		set[ia] = 0;
	}
}

int acq400_read_set(struct acq400_dev* set[],
		struct acq400_dev *adev, char *buf, int maxbuf)
{
	int ia = 0;
	int cursor = 0;

	for (ia = 0; ia < MAXSITES; ++ia){
		if (set[ia] != 0){
			int site = set[ia]->of_prams.site;
			if (cursor){
				cursor += sprintf(buf+cursor, ",%d", site);
			}else{
				cursor += sprintf(buf+cursor, "%d", site);
			}

			if (cursor > maxbuf - 3){
				break;
			}
		}
	}
	cursor += sprintf(buf+cursor, "\n");
	return cursor;
}


int acq400_add_aggregator_set(struct acq400_dev *adev, int site)
{
	return acq400_add_set(adev->aggregator_set, adev, site);
}

int acq400_read_aggregator_set(struct acq400_dev *adev, char *buf, int maxbuf)
{
	return acq400_read_set(adev->aggregator_set, adev, buf, maxbuf);
}

void acq400_clear_aggregator_set(struct acq400_dev *adev)
{
	acq400_clear_set(adev->aggregator_set);
}
int acq400_add_distributor_set(struct acq400_dev *adev, int site)
{
	return acq400_add_set(adev->distributor_set, adev, site);
}
void acq400_clear_distributor_set(struct acq400_dev *adev)
{
	acq400_clear_set(adev->distributor_set);
}

void acq400_visit_set(struct acq400_dev *set[], void (*action)(struct acq400_dev *adev))
{
	int site;
	for (site = 0; site < MAXSITES; ++site){
		if (set[site]){
			dev_dbg(DEVP(set[site]), "visit_set [%d] %p", site, set[site]);
			action(set[site]);
		}
	}
}
