/* ------------------------------------------------------------------------- */
/* acq400_sysfs_utils.c  D-TACQ ACQ400 FMC  DRIVER                                   
 * Project: ACQ420_FMC
 * Created: 16 Nov 2016  			/ User: pgm
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

#include <linux/ctype.h>

#include "acq400.h"
#include "bolo.h"

#include "acq400_sysfs.h"

ssize_t acq400_show_triplet(
	struct device * dev,
	struct device_attribute *attr,
	char * buf,
	unsigned REG,
	unsigned T1,
	unsigned T2,
	unsigned T3)
{
	u32 regval = acq400rd32(acq400_devices[dev->id], REG);

	return sprintf(buf, "%d,%d,%d\n",
		MASKSHR(regval, T1), MASKSHR(regval, T2), MASKSHR(regval, T3));
}


#define SHLMASK(x, f) (((x)<<getSHL(f))&(f))

u32 get_edge_sense(char edge[], u32 vv)
{
	int len = strlen(edge);
	if (len){
		int ii;
		for(ii = 0; ii < len; ++ii){
			edge[ii] = tolower(edge[ii]);
		}
		vv = strcmp(edge, "rising") == 0;
	}
	return vv;
}
ssize_t acq400_store_triplet(
		struct device * dev,
		struct device_attribute *attr,
		const char * buf,
		size_t count,
		unsigned REG,
		unsigned T1,
		unsigned T2,
		unsigned T3)
{
	u32 v1, v2, v3;
	char edge[21];
	edge[0] = '\0';

	if (sscanf(buf, "%d,d%d,%20s", &v1, &v2, edge) == 3 ||
	    sscanf(buf, "%d,%d,%d", &v1, &v2, &v3) == 3		){

		u32 regval = acq400rd32(acq400_devices[dev->id], REG);
		v3 = get_edge_sense(edge, v3);


		regval &= ~(T1|T2|T3);
		regval |= SHLMASK(v1, T1) | SHLMASK(v2, T2) | SHLMASK(v3, T3);

		acq400wr32(acq400_devices[dev->id], REG, regval);
		return count;
	}else{
		return -1;
	}
}

/* generic show_bits, store_bits */

ssize_t acq400_show_bits(
	struct device * dev,
	struct device_attribute *attr,
	char * buf,
	unsigned REG,
	unsigned SHL,
	unsigned MASK)
{
	u32 regval = acq400rd32(acq400_devices[dev->id], REG);
	u32 field = (regval>>SHL)&MASK;

	return sprintf(buf, "%x\n", field);
}

ssize_t acq400_store_bits(
		struct device * dev,
		struct device_attribute *attr,
		const char * buf,
		size_t count,
		unsigned REG,
		unsigned SHL,
		unsigned MASK,
		unsigned show_warning)
{
	u32 field;
	if (sscanf(buf, "%x", &field) == 1){
		u32 regval = acq400rd32(acq400_devices[dev->id], REG);
		regval &= ~(MASK << SHL);
		regval |= (field&MASK) << SHL;
		if (show_warning){
			dev_warn(dev, "deprecated %04x = %08x", REG, regval);
		}
		acq400wr32(acq400_devices[dev->id], REG, regval);
		return count;
	}else{
		return -1;
	}
}


ssize_t acq400_show_bitN(
	struct device * dev,
	struct device_attribute *attr,
	char * buf,
	unsigned REG,
	unsigned SHL,
	unsigned MASK)
{
	u32 regval = acq400rd32(acq400_devices[dev->id], REG);
	u32 field = (regval>>SHL)&MASK;

	return sprintf(buf, "%x\n", !field);
}

ssize_t acq400_store_bitN(
		struct device * dev,
		struct device_attribute *attr,
		const char * buf,
		size_t count,
		unsigned REG,
		unsigned SHL,
		unsigned MASK,
		unsigned show_warning)
{
	u32 field;
	if (sscanf(buf, "%x", &field) == 1){
		u32 regval = acq400rd32(acq400_devices[dev->id], REG);
		regval &= ~(MASK << SHL);
		regval |= ((!field)&MASK) << SHL;
		if (show_warning){
			dev_warn(dev, "deprecated %04x = %08x", REG, regval);
		}
		acq400wr32(acq400_devices[dev->id], REG, regval);
		return count;
	}else{
		return -1;
	}
}


ssize_t acq400_show_dnum(
	struct device * dev,
	struct device_attribute *attr,
	char * buf,
	unsigned REG,
	unsigned SHL,
	unsigned MASK)
{
	u32 regval = acq400rd32(acq400_devices[dev->id], REG);
	u32 field = (regval>>SHL)&MASK;
	u32 sbit = (MASK+1) >> 1;
	if ((sbit&field) != 0){
		while(sbit <<= 1){
			field |= sbit;
		}
	}

	return sprintf(buf, "%d\n", (int)field);
}
ssize_t acq400_store_dnum(
		struct device * dev,
		struct device_attribute *attr,
		const char * buf,
		size_t count,
		unsigned REG,
		unsigned SHL,
		unsigned MASK)
{
	int field;
	if (sscanf(buf, "%d", &field) == 1){
		u32 regval = acq400rd32(acq400_devices[dev->id], REG);
		regval &= ~(MASK << SHL);
		regval |= (field&MASK) << SHL;
		acq400wr32(acq400_devices[dev->id], REG, regval);
		return count;
	}else{
		return -1;
	}
}

