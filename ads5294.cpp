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

int Ads5294::SetLvdsTestPatRamp(bool enable)
{
	Reg& reg = regs->regs[Ads5294Regs::RA_TEST25];

	reg &= ~0x70;
	if (enable){
		reg |= 0x40;
		SetLvdsTestPatDeskew(false);
	}
	return 1;
}
int Ads5294::SetLvdsTestPatDeskew(bool enable)
{
	Reg& reg = regs->regs[Ads5294Regs::RA_TEST25];

	reg &= ~0x03;
	if (enable){
		reg |= 0x01;
		SetLvdsTestPatRamp(false);
	}
	return 1;
}

int Ads5294::setPatDeskew(bool enable)
{
	Reg& reg = regs->regs[Ads5294Regs::RA_PAT];

	reg &= ~0x03;
	if (enable){
		reg |= 0x01;
	}
	return 1;
}
int Ads5294::setPatSync(bool enable)
{
	Reg& reg = regs->regs[Ads5294Regs::RA_PAT];

	reg &= ~0x03;
	if (enable){
		reg |= 0x02;
	}
	return 1;
}
int Ads5294::setDataPattern(unsigned short regval)
{
	Reg& reg = regs->regs[Ads5294Regs::RA_WIRE_MODE];

	reg = regval;
	return 1;
}

struct KeyLut {
	const char* key;
	unsigned short pat;
	unsigned short pat_field;
};

KeyLut keyLut[] = {
		{ "EN_2WIRE", 	1<<0 },
		{ "BTC_MODE", 	1<<2 },
		{ "MSB_FIRST", 	1<<3 },
		{ "EN_SDR",     1<<4 },
		{ "EN_14BIT",   0x40, 0xf0 },
		{ "EN_16BIT",   0x80, 0xf0 },
		{ "FALL_SDR",   1<<13 },
};
#define NKEYS (sizeof(keyLut)/sizeof(KeyLut))

int _help() {
	printf("_help() NKEYS %d\n", NKEYS);
	for (int ikey = 0; ikey < NKEYS; ++ikey){
		const char* k = keyLut[ikey].key;
		unsigned short bv = keyLut[ikey].pat;

		printf("%s ", k);
	}
	printf("\n");
	return 0;
}
int Ads5294::setDataPattern(int argc, char* argv[])
{
	int ii;
	for (ii = 0; ii < argc; ++ii){
		const char* tok = argv[ii];
		if (strcmp(tok, "help") == 0){
			return _help();
		}else{
			Reg& reg = regs->regs[Ads5294Regs::RA_WIRE_MODE];
			bool negate = false;

			if (tok[0] == '-'){
				negate = true;
				tok += 1;
			}
			for (int ikey = 0; ikey < NKEYS; ++ikey){
				const char* k = keyLut[ikey].key;
				unsigned short bv = keyLut[ikey].pat;
				unsigned clr = ~keyLut[ikey].pat_field;

				if (strcmp(k, tok) == 0){
					reg &= clr;
					if (negate){
						reg &= ~bv;
					}else{
						reg |= bv;
					}
					goto next_arg;
				}

			}
			printf("Ads5294::setDataPattern() ERROR: \"%s\" not supported\n", tok);
			return 0;
		}
next_arg:
		;
	}
	printf("\n");
	return 0;
}

int Ads5294::getDataPattern()
{
	Reg& reg = regs->regs[Ads5294Regs::RA_WIRE_MODE];

	for (int ikey = 0; ikey < NKEYS; ++ikey){
		unsigned rv = reg;
		if (keyLut[ikey].pat_field){
			rv &= keyLut[ikey].pat_field;
			if (rv == keyLut[ikey].pat){
				printf("%s ", keyLut[ikey].key);
			}
		}else{
			const char* k = keyLut[ikey].key;
			unsigned short bv = keyLut[ikey].pat;
			printf("%c%s ", (rv&bv) == bv? ' ':'-', k);
		}
	}
	printf("\n");
}
int Ads5294::setReg(unsigned reg, unsigned pattern)
{
	printf("setReg() %02x %04x\n", reg);
	if (reg > 0 && reg < 255){
		Reg& _reg = regs->regs[reg];
		_reg = pattern;
		return 1;
	}else{
		printf("ERROR reg not in range 0 .. 255\n");
		return 0;
	}
}

int Ads5294::getReg(unsigned reg, unsigned& pattern)
{
	printf("getReg() %02x\n", reg);
	if (reg > 0 && reg < 255){
		Reg& _reg = regs->regs[reg];
		pattern = _reg;
		return 0;
	}else{
		printf("ERROR reg not in range 0 .. 255\n");
		return 0;
	}
}



Ads5294::MapLut Ads5294::maplut[] = {
 { "MAP_CH1234_TO_OUT1A", Ads5294Regs::RA_MAP_50, 0, { 1, 2, 3, 4} },
 { "MAP_CH1234_TO_OUT1B", Ads5294Regs::RA_MAP_50, 4, { 1, 2, 3, 4} },
 { "MAP_CH1234_TO_OUT2A", Ads5294Regs::RA_MAP_50, 8, { 1, 2, 3, 4} },
 { "MAP_CH1234_TO_OUT2B", Ads5294Regs::RA_MAP_51, 0, { 1, 2, 3, 4} },
 { "MAP_CH1234_TO_OUT3A", Ads5294Regs::RA_MAP_51, 4, { 1, 2, 3, 4} },
 { "MAP_CH1234_TO_OUT3B", Ads5294Regs::RA_MAP_51, 8, { 1, 2, 3, 4} },
 { "MAP_CH1234_TO_OUT4A", Ads5294Regs::RA_MAP_52, 0, { 1, 2, 3, 4} },
 { "MAP_CH1234_TO_OUT4B", Ads5294Regs::RA_MAP_52, 4, { 1, 2, 3, 4} },
 { "MAP_CH5678_TO_OUT5B", Ads5294Regs::RA_MAP_53, 0, { 5, 6, 7, 8} },
 { "MAP_CH5678_TO_OUT5A", Ads5294Regs::RA_MAP_53, 4, { 5, 6, 7, 8} },
 { "MAP_CH5678_TO_OUT6B", Ads5294Regs::RA_MAP_53, 8, { 5, 6, 7, 8} },
 { "MAP_CH5678_TO_OUT6A", Ads5294Regs::RA_MAP_54, 0, { 5, 6, 7, 8} },
 { "MAP_CH5678_TO_OUT7B", Ads5294Regs::RA_MAP_54, 4, { 5, 6, 7, 8} },
 { "MAP_CH5678_TO_OUT7A", Ads5294Regs::RA_MAP_54, 8, { 5, 6, 7, 8} },
 { "MAP_CH5678_TO_OUT8B", Ads5294Regs::RA_MAP_55, 0, { 5, 6, 7, 8} },
 { "MAP_CH5678_TO_OUT8A", Ads5294Regs::RA_MAP_55, 4, { 5, 6, 7, 8} }
};

#define NMAPLUT	(sizeof(maplut)/sizeof(MapLut))

const Ads5294::MapLut& Ads5294::lookupMap(const char* key){
	for (int ik = 0; ik < NMAPLUT; ++ik){
		if (strcmp(key, maplut[ik].key) == 0){
			return maplut[ik];
		}
	}

	fprintf(stderr, "ERROR: key \"%s\" NOT VALID\n", key);
	exit(1);
	return maplut[0];
}

void Ads5294::printMapHelp(const char* pfx)
{
	for (int ik = 0; ik < NMAPLUT; ++ik){
		MapLut& map = maplut[ik];
		printf("%s %s %d %d %d %d\n",
			pfx, map.key,
			map.wxyz[0], map.wxyz[1],
			map.wxyz[2], map.wxyz[3]);
	}
	printf("%s ALL 0 0 0 0\n", pfx);
	printf("%s ALL1234 1 2 3 4\n", pfx);
	printf("%s ALL5678 5 6 7 8\n", pfx);
}
void Ads5294::printMap(const Ads5294::MapLut& map)
{
	Reg& _reg = regs->regs[map.reg];
	unsigned chx = (_reg >> map.shl) & 0x0f;
	printf("%s ", map.key);
	for (int ix = 0; ix < 4; ++ix){
		printf("%d ", (chx&(1<<ix))? map.wxyz[ix]: 0);
	}
	printf("\n");
}

void Ads5294::printMap(int imap)
{
	printMap(maplut[imap]);
}

bool Ads5294::isValidChx(const Ads5294::MapLut& map, int chx)
{
	if (chx == 0) return true;

	for (int ix = 0; ix < 4; ++ix){
		if (chx == map.wxyz[ix]){
			return true;
		}
	}

	printf("WARNING: \"%s\" and %d not a valid combo\n", map.key, chx);
	return false;
}

int Ads5294::setMap(const char* mapping, int chW, int chX, int chY, int chZ)
{

	if (mapping == MAP_ALL || strncmp(mapping, "ALL", 3) == 0 ){
		int im0 = 0;
		int im1 = NMAPLUT;
		if (strcmp(mapping, "ALL1234") == 0){
			im1 = NMAPLUT/2;
		}else if (strcmp(mapping, "ALL5678") == 0){
			im0 = NMAPLUT/2;
		}
		// else all regs .. value for 0,0,0,0 only

		for (int imap = im0; imap < im1; ++imap){
			setMap(maplut[imap].key, chW, chX, chY, chZ);
		}
		return 1;
	}else{
		const MapLut& map = lookupMap(mapping);
		unsigned ccc = 0;

		if (chW > 0 && isValidChx(map, chW)) ccc |= 1 << ((chW-1)%4);
		if (chX > 0 && isValidChx(map, chX)) ccc |= 1 << ((chX-1)%4);
		if (chY > 0 && isValidChx(map, chY)) ccc |= 1 << ((chY-1)%4);
		if (chZ > 0 && isValidChx(map, chZ)) ccc |= 1 << ((chZ-1)%4);

		Reg& _reg = regs->regs[map.reg];

		_reg &= ~ (0xf << map.shl);
		_reg |= ccc << map.shl;
		_reg |= 1 << MAP_EN_BIT;
		return 1;
	}
}
int Ads5294::getMap(const char* mapping)
{
	if (mapping == MAP_ALL){
		for (int imap = 0; imap < NMAPLUT; ++imap){
			printMap(imap);
		}
	}else{
		printMap(lookupMap(mapping));
	}
	return 0;
}
