/*
 * ads5294.h
 *
 *  Created on: 15 Feb 2015
 *      Author: pgm
 *
 *  Plan:
 *  1. build a mirror of all the regs in memory
 *  2. create methods for every function to set appropriate bits in mem
 *  3. detect changes to mirror and write to device
 *  Device STATE is held in user space. This could be SHM for umon access.
 */

#ifndef ADS5294_H_
#define ADS5294_H_


typedef unsigned short Reg;


#define NREGS	0x100

struct Ads5294Regs {
	Reg regs[NREGS];

	enum RegAddrs {					/* address, and index to regs */
		RA_RST			= 0x00,
		RA_EN_1 		= 0x01,
		RA_EN_2			= 0x02,
		RA_RAMP_PAT_RESET_VAL   = 0x0a,		/* @@todo NOT DOC */
		RA_PDN			= 0x0f,
		RA_LFNS			= 0x14,
		RA_EN_FRAME		= 0x1c,		/* shared reg */
		RA_ADCLK		= 0x1c,
		RA_PRBS_SEED_LO		= 0x23,
		RA_INVERT_CH		= 0x24,		/* shared reg */
		RA_PRBS_SEED_HI		= 0x24,
		RA_TEST25		= 0x25,
		RA_BITS_CUSTOM1		= 0x26,
		RA_BITS_CUSTOM2		= 0x27,
		RA_BITORDER		= 0x28,
		RA_GLOBAL_EN_FILTER	= 0x29,
		RA_GAIN_14		= 0x2a,
		RA_GAIN_58		= 0x2b,
		RA_AVG_14		= 0x2c,
		RA_AVG_58		= 0x2d,
		RA_FILTER_1		= 0x2e,
		RA_FILTER_2		= 0x2f,
		RA_FILTER_3		= 0x30,
		RA_FILTER_4		= 0x31,
		RA_FILTER_5		= 0x32,
		RA_FILTER_6		= 0x33,
		RA_FILTER_7		= 0x34,
		RA_FILTER_8		= 0x35,

		RA_DATA_RATE		= 0x38,
		RA_EXT_REF_VCM		= 0x42,		/* shared .. */
		RA_PHASE_DDR		= 0x42,

		RA_PAT			= 0x45,

		RA_WIRE_MODE		= 0x46,

		/* MAP : assume 1:1 on ACQ480 */

		RA_EXT_REF		= 0xf0
	};
};


#define RA_RST_RST_BIT		0

#define RA_EN_1_READOUT_BIT	0
#define RA_EN_1_HIGH_ADDR_BIT	4

#define RA_EN_2_SYNC_BIT	13

#define RA_PDN_NORMAL		0
#define RA_PDN_CH_MASK		0x00ff
#define RA_PDN_CH_BIT(n)	((n)-1)			/* n = 1..8 */
#define RA_PDN_PART_BIT		8
#define RA_PDN_COMPLETE_BIT	9
#define RA_PDN_PIN_CFG_BIT	10

#define RA_LFNC_CH_MASK		0x00ff
#define RA_LFNS_CH_BIT(n)	((n)-1)			/* n = 1..8 */

#define RA_EN_FRAME_BIT		14

#define RA_ADCLK_ADCLKOUT	0x3ff

#define RA_INVERT_CH_MASK	0x00ff
#define RA_INVERT_CH_BIT(n)	((n)-1)			/* n = 1..8 */

#define RA_PRBS_SEED_HI_SHL	9

#define RA_TEST25_EN_MODE_MASK	0x0070
#define RA_TEST25_EN_RAMP	0x0040
#define RA_TEST25_DUAL_CUST_PAT 0x0020
#define RA_TEST25_SGL_CUT_PAT   0x0010
#define RA_TEST25_BITS_CUST1_BIT 	0
#define RA_TEST25_BITS_CUST2_BIT	2
#define RA_TEST25_TP_SOFT_SYNC_BIT	8
#define RA_TEST25_PRBS_TP_EN_BIT	12
#define RA_TEST25_PRBS_MODE_2_BIT	13
#define RA_TEST25_PRBS_SEED_FRM_REG_BIT 14
#define RA_TEST25_TP_HARD_SYNC_BIT	15

#define RA_BITS_CUSTOM_MASK	0x0fff
#define RA_BITS_CUSTOM_SHL	4

#define RA_BITORDER_WW_BIT	15
#define RA_BITORDER_BW_BIT	8
#define RA_BITORDER_WW_CHAN(n)	((n-1))			/* n = 1..8 */

#define RA_GLOBAL_EN_FILTER_EN_BIT	1
#define RA_GLOBAL_EN_FILTER_CHAVG_BIT	0

#define RA_GAIN_CH_MASK		0xf
#define RA_GAIN_14_CH(n)	(4*(n-1))		/* n = 1..4 */
#define RA_GAIN_58_CH(n)	(4*(n-5))		/* n = 5..8 */

#define RA_AVG_MASK		0x3
#define RA_AVG_14(n)		(3*(n-1))		/* n = 1..4 */
#define RA_AVG_58(n)		(3*(n-5))		/* n = 5..8 */

#define RA_FILTER_HPF_EN_BIT	14
#define RA_FILTER_HPF_CRNR_MASK 0xf
#define RA_FILTER_HPF_CRNR_SHL	10
#define RA_FILTER_COEFF_MASK	0x7
#define RA_FILTER_COEFF_SHL	7
#define RA_FILTER_RATE_MASK	0x7
#define RA_FILTER_RATE_SHL	4
#define RA_FILTER_ODD_TAP_BIT	2
#define RA_FILTER_ENABLE	0

#define RA_DATA_RATE_MASK	0x3
#define RA_DATA_RATE_DIV_1	0x0
#define RA_DATA_RATE_DIV_2	0x1
#define RA_DATA_RATE_DIV_4	0x2
#define RA_DATA_RATE_DIV_8	0x3

#define RA_EXT_REF_VCM_MASK	0x8008
#define RA_PHASE_DDR_MASK	0x3
#define RA_PHASE_DDR_SHL	5

#define RA_PAT_MASK		0x3
#define RA_PAT_INACTIVE		0x0
#define RA_PAT_DESKEW		0x1
#define RA_PAT_SYNC		0x2

#define RA_WIRE_MODE_EN_2WIRE	0x8001
#define RA_WIRE_MODE_BTC	0x8004			/* Binary 2's comp: YES! */
#define RA_WIRE_MODE_MSB_FIRST	0x8008
#define RA_WIRE_MODE_EN_SDR	0x8010

#define RA_WIRE_MODE_EN_BIT_SER_MASK 0xf
#define RA_WIRE_MODE_EN_BIT_SER_SHL  8
#define RA_WIRE_MODE_EN_10BIT	0x1
#define RA_WIRE_MODE_EN_12BIT	0x2
#define RA_WIRE_MODE_EN_14BIT	0x4
#define RA_WIRE_MODE_EN_16BIT	0x8


#define RA_WIRE_MODE_FALL_SDR	0xa000

#define RA_EXT_REF_EN_BIT	15




#endif /* ADS5294_H_ */
