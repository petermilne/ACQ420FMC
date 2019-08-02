/* ------------------------------------------------------------------------- */
/* acq400_dspfs.c ACQ420_FMC						     */
/*
 * acq400_dspfs.c
 *
 *  Created on: 18 Aug 2015
 *      Author: pgm
 */

/* ------------------------------------------------------------------------- */
/*   Copyright (C) 2015 Peter Milne, D-TACQ Solutions Ltd                    *
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
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.                */
/* ------------------------------------------------------------------------- */



#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/cdev.h>
#include <linux/irqreturn.h>
#include <linux/interrupt.h>

#include "acq400_dsp.h"
#include "regfs.h"

#define REVID	"1.2"
#define MAXDEV 2


int acq400dsp_devicetree_init(struct platform_device *pdev, struct device_node *of_node)
{
	if (of_node == 0){
		return -1;
	}else{
		int ii;
		unsigned site = -1;
		const char* name = 0;
		if (of_property_read_string(of_node, "name", &name) < 0){
			dev_warn(&pdev->dev, "warning: name NOT specified in DT");
		}
		if (of_property_read_u32(of_node, "site", &site) < 0){
			dev_warn(&pdev->dev, "warning: site NOT specified in DT");
		}
		for (ii = 0; ii < pdev->num_resources; ++ii){
			if (name){
				pdev->resource[ii].name = name;
			}
		}
		if (site != -1){
			pdev->id = site;
		}
		return 0;
	}
}
int acq400dsp_probe(struct platform_device *pdev)
{
	int rc;

	dev_info(&pdev->dev, "acq400dsp_probe id:%d site:%d num_resources:%d",
			pdev->id, 99, pdev->num_resources);
	dev_info(&pdev->dev, "acq400dsp_probe [%d] site:%d num_resources:%d",
			pdev->id, 99, pdev->num_resources);

	rc = acq400dsp_devicetree_init(pdev, pdev->dev.of_node);
	if (rc != 0){
		return rc;
	}

	return regfs_probe(pdev);
}

int acq400dsp_remove(struct platform_device *pdev)
{
       dev_info(&pdev->dev, "acq400dsp_probe id:%d site:%d num_resources:%d",
                       pdev->id, 99, pdev->num_resources);
       return regfs_remove(pdev);
}


#ifdef CONFIG_OF
static struct of_device_id acq400dsp_of_match[] /* __devinitdata */ = {
        { .compatible = "D-TACQ,acq400dsp"  },
        { /* end of table */}
};
MODULE_DEVICE_TABLE(of, acq400dsp_of_match);
#else
#define acq400dsp_of_match NULL
#endif /* CONFIG_OF */

#define MODULE_NAME "acq400_dspfs"

static struct platform_driver acq400dsp_driver = {
        .driver = {
                .name = MODULE_NAME,
                .owner = THIS_MODULE,
                .of_match_table = acq400dsp_of_match,
        },
        .probe = acq400dsp_probe,
        .remove = acq400dsp_remove,
};

static int __init acq400_dsp_init(void)
{
        int status;

	printk("D-TACQ ACQ400 DSP Driver %s\n", REVID);

	status = platform_driver_register(&acq400dsp_driver);
        return status;
}

static void __exit acq400_dsp_exit(void)
{

}

module_init(acq400_dsp_init);
module_exit(acq400_dsp_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("D-TACQ DSP filesystem device descriptions");
MODULE_AUTHOR("D-TACQ Solutions.");
MODULE_VERSION(REVID);
