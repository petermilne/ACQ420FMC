/*
 * ads5294.cpp
 *
 *  Created on: 15 Feb 2015
 *      Author: pgm
 */


#include "ads5294.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void die(const char *fmt)
{
	fprintf(stderr, "%s\n", fmt);
	exit(1);
}

int Ads5294::setGain(Chan chan, Gain gain)
{
	if (!isValidChan(chan)) die("channel not valid");
	if (!isValidGain(gain)) die("gain not valid");

	if (chan <= 4){
		Reg& reg = regs->regs[Ads5294Regs::RA_GAIN_14];
		reg &= ~(RA_GAIN_CH_MASK << RA_GAIN_14_CH(chan));
		reg |= gain << RA_GAIN_14_CH(chan);
	}else{
		Reg& reg = regs->regs[Ads5294Regs::RA_GAIN_58];
		reg &= ~(RA_GAIN_CH_MASK << RA_GAIN_58_CH(chan));
		reg |= gain << RA_GAIN_58_CH(chan);
	}
	return 0;
}
Ads5294::Gain Ads5294::getGain(Chan chan)
{
	if (!isValidChan(chan)) die("channel not valid");

	Reg reg;

	if (chan <= 4){
		Reg reg = regs->regs[Ads5294Regs::RA_GAIN_14];
		reg >>= RA_GAIN_14_CH(chan);
	}else{
		Reg reg = regs->regs[Ads5294Regs::RA_GAIN_58];
		reg >>= RA_GAIN_58_CH(chan);
	}

	return static_cast<Gain>(reg&RA_GAIN_CH_MASK);
}

int Ads5294::setCustomCoefficients(Chan chan, short* coeffs)
{
	if (!isValidChan(chan)) die("channel not valid");

	Reg* dst = &regs->regs[RA_CUSTOM_COEFF(chan, 0)];

	if (coeffs){
		memcpy(dst, coeffs,  NTAPS*sizeof(Reg));
	}
	for (int tap = 0; tap < NTAPS; ++tap){
		if (coeffs){
			dst[tap] |= 1<<RA_CUSTOM_COEFF_EN_BIT;
		}else{
			dst[tap] &= ~(1<<RA_CUSTOM_COEFF_EN_BIT);
		}
	}
	return 0;
}

short* Ads5294::getCustomCoefficients(Chan chan)
{
	if (!isValidChan(chan)) die("channel not valid");

	return reinterpret_cast<short*>(&regs->regs[RA_CUSTOM_COEFF(chan, 0)]);
}

int _setDecimationFilter(
	Reg& freg, bool enable, bool odd_tap, FilterCoeffSelect fcs, FilterRate rate)
{
	if (enable){
		freg |= (1<<RA_FILTER_ENABLE);
	}else{
		freg &= ~ (1<<RA_FILTER_ENABLE);
	}
	if (odd_tap){
		freg |= (1<<RA_FILTER_ODD_TAP_BIT);
	}else{
		freg &= ~(1<<RA_FILTER_ODD_TAP_BIT);
	}
	freg &= ~(RA_FILTER_COEFF_MASK << RA_FILTER_COEFF_SHL);
	freg |= fcs << RA_FILTER_COEFF_SHL;

	freg &= ~(RA_FILTER_RATE_MASK << RA_FILTER_RATE_SHL);
	freg |= rate << RA_FILTER_RATE_SHL;

	return 0;
}
int Ads5294::setDecimationFilter(Chan chan, Filter filter, bool odd_tap)
{
	if (!isValidChan(chan)) die("channel not valid");
	if (!isValidFilter(filter)) die("filer not valid");
	const bool filen = true;
	const bool odd = true;
	const bool even = false;

	Reg& freg = regs->regs[Ads5294Regs::RA_FILTER_1+CHIX(chan)];

	switch(filter){
	case F_DISABLE:
	case F_CUSTOM_D1:
		regs->regs[Ads5294Regs::RA_DATA_RATE] = RA_DATA_RATE_DIV_1;
		break;
	case F_LP_ODD_D2:
	case F_HP_ODD_D2:
	case F_CUSTOM_D2:
		regs->regs[Ads5294Regs::RA_DATA_RATE] = RA_DATA_RATE_DIV_2;
		break;
	case F_LP_EVEN_D4:
	case F_BP1_EVEN_D4:
	case F_BP2_EVEN_D4:
	case F_HP_ODD_D4:
	case F_CUSTOM_D4:
		regs->regs[Ads5294Regs::RA_DATA_RATE] = RA_DATA_RATE_DIV_4;
		break;
	case F_CUSTOM_D8:
		regs->regs[Ads5294Regs::RA_DATA_RATE] = RA_DATA_RATE_DIV_8;
		break;
	}
	switch(filter){
	case F_DISABLE:
		return _setDecimationFilter(freg, !filen, even, FCS_DISABLE, FR_D2);
	case F_LP_ODD_D2:
		return _setDecimationFilter(freg, filen, odd, FCS_LP_ODD, FR_D2);
	case F_HP_ODD_D2:
		return _setDecimationFilter(freg, filen, odd, FCS_HP_ODD, FR_D2);
	case F_LP_EVEN_D4:
		return _setDecimationFilter(freg, filen, even, FCS_LP_EVEN, FR_D4);
	case F_BP1_EVEN_D4:
		return _setDecimationFilter(freg, filen, even, FCS_BP1_EVEN, FR_D4);
	case F_BP2_EVEN_D4:
		return _setDecimationFilter(freg, filen, even, FCS_BP2_EVEN, FR_D4);
	case F_HP_ODD_D4:
		return _setDecimationFilter(freg, filen, odd, FCS_HP_ODD2, FR_D4);
	case F_CUSTOM_D2:
		return _setDecimationFilter(freg, filen, odd_tap, FCS_CUSTOM, FR_D2);
	case F_CUSTOM_D4:
		return _setDecimationFilter(freg, filen, odd_tap, FCS_CUSTOM, FR_D4);
	case F_CUSTOM_D8:
		return _setDecimationFilter(freg, filen, odd_tap, FCS_CUSTOM, FR_D8);
	case F_CUSTOM_D1:
		return _setDecimationFilter(freg, filen, odd_tap, FCS_CUSTOM, FR_D1);
	}

	die("should not get here ..");
	return 0;
}

Ads5294::Filter Ads5294::getDecimationFilter(Chan chan)
{
	return F_DISABLE;
}

int Ads5294::setHiPassFilter(Chan chan, bool enable, unsigned hpfun)
{
	if (!isValidChan(chan)) die("channel not valid");
	Reg& freg = regs->regs[Ads5294Regs::RA_FILTER_1+CHIX(chan)];
	const unsigned en = 1<<RA_FILTER_HPF_EN_BIT;

	if (enable){
		freg &= ~(RA_FILTER_HPF_CRNR_MASK << RA_FILTER_HPF_CRNR_SHL);
		freg |= en | (hpfun << RA_FILTER_HPF_CRNR_SHL);
	}else{
		freg &= ~en;
	}
	return 0;
}

unsigned Ads5294::getHiPassFilter(Chan chan)
{
	if (!isValidChan(chan)) die("channel not valid");
	Reg& freg = regs->regs[Ads5294Regs::RA_FILTER_1+CHIX(chan)];

	return (freg >> RA_FILTER_HPF_CRNR_SHL)&RA_FILTER_HPF_CRNR_MASK;
}

int Ads5294::setDataRate(DataRate dr)
{
	Reg& freg = regs->regs[Ads5294Regs::RA_DATA_RATE];
	freg = dr;
	return 0;
}

Ads5294::DataRate Ads5294::getDataRate()
{
	return static_cast<DataRate>(regs->regs[Ads5294Regs::RA_DATA_RATE]);
}

int Ads5294::setAverageSelect(Chan bin, bool enable, unsigned avsel)
{
	if (!isValidChan(bin)) die("bin not valid");
	unsigned chav = 1<<RA_GLOBAL_EN_FILTER_CHAVG_BIT;
	if (enable){
		regs->regs[Ads5294Regs::RA_GLOBAL_EN_FILTER] |= chav;

		if (bin <= 4){
			Reg& freg = regs->regs[Ads5294Regs::RA_AVG_14];
			freg &= ~(RA_AVG_MASK << RA_AVG_14(bin));
			freg |= (avsel&RA_AVG_MASK) << RA_AVG_14(bin);
		}else{
			Reg& freg = regs->regs[Ads5294Regs::RA_AVG_58];
			freg &= ~(RA_AVG_MASK << RA_AVG_58(bin));
			freg |= (avsel&RA_AVG_MASK) << RA_AVG_58(bin);
		}
	}else{
		regs->regs[Ads5294Regs::RA_GLOBAL_EN_FILTER] &= ~chav;
	}
	return 0;
}
unsigned Ads5294::Ads5294::getAverageSelect(Chan bin)
{
	if (!isValidChan(bin)) die("bin not valid");
	unsigned chav = 1<<RA_GLOBAL_EN_FILTER_CHAVG_BIT;

	if (regs->regs[Ads5294Regs::RA_GLOBAL_EN_FILTER]&chav){
		if (bin <= 4){
			Reg freg = regs->regs[Ads5294Regs::RA_AVG_14];
			return (freg >> RA_AVG_14(bin)) & RA_AVG_MASK;
		}else{
			Reg freg = regs->regs[Ads5294Regs::RA_AVG_58];
			return (freg >> RA_AVG_58(bin)) & RA_AVG_MASK;
		}
	}else{
		return 0xffffffff;
	}
}

int Ads5294::setInvert(Chan chan, bool invert)
{
	if (!isValidChan(chan)) die("chan not valid");

	Reg& freg = regs->regs[Ads5294Regs::RA_INVERT_CH];

	if (invert){
		freg |= CHAN2BIT(chan);
	}else{
		freg &= ~CHAN2BIT(chan);
	}
	return 0;
}
unsigned Ads5294::getInvert(Chan chan)
{
	if (!isValidChan(chan)) die("chan not valid");

	unsigned inverts = regs->regs[Ads5294Regs::RA_INVERT_CH];

	return inverts&CHAN2BIT(chan) != 0;
}

int Ads5294::setLFNS(Chan chan, bool enable)
{
	if (!isValidChan(chan)) die("chan not valid");

	Reg& freg = regs->regs[Ads5294Regs::RA_LFNS];

	if (enable){
		freg |= CHAN2BIT(chan);
	}else{
		freg &= ~CHAN2BIT(chan);
	}
	return 0;
}
bool Ads5294::getLFNS(Chan chan)
{
	if (!isValidChan(chan)) die("chan not valid");

	unsigned lfns = regs->regs[Ads5294Regs::RA_LFNS];

	return lfns&CHAN2BIT(chan) != 0;
}

