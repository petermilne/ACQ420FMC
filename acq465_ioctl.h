/*
 * acq465_ioctl.h
 *
 *  Created on: 28 September 2021
 *      Author: pgm
 */

#ifndef ACQ465_IOCTL_H_
#define ACQ465_IOCTL_H_

#include <linux/ioctl.h>

#define MAGIC	0xa465

#define CMASK	unsigned			/* Chip mask 0x80 .. 0x01 .. select any of 8 */

#define ACQ465_RESET		_IOW(MAGIC, 1, CMASK)
#define ACQ465_CACHE_INVALIDATE	_IOW(MAGIC, 2, CMASK)
#define ACQ465_CACHE_FLUSH	_IOW(MAGIC, 3, CMASK)


#endif /* ACQ465_IOCTL_H_ */
