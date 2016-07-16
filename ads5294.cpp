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
		reg = regs->regs[Ads5294Regs::RA_GAIN_14];
		reg >>= RA_GAIN_14_CH(chan);
	}else{
		reg = regs->regs[Ads5294Regs::RA_GAIN_58];
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
			dst[tap] = 0;
		}
	}
	return 0;
}

short* Ads5294::getCustomCoefficients(Chan chan)
{
	if (!isValidChan(chan)) die("channel not valid");

	return reinterpret_cast<short*>(&regs->regs[RA_CUSTOM_COEFF(chan, 0)]);
}

int Ads5294::_setDecimationFilter(
	Reg& freg, bool enable, bool odd_tap, FilterCoeffSelect fcs, FilterRate rate)
{
	unsigned genbit = 1<<RA_GLOBAL_EN_FILTER_EN_BIT;
	if (enable){
		regs->regs[Ads5294Regs::RA_GLOBAL_EN_FILTER] |= genbit;
		freg |= (1<<RA_FILTER_ENABLE);
	}else{
		regs->regs[Ads5294Regs::RA_GLOBAL_EN_FILTER] &= ~genbit;
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

	return (regs->regs[Ads5294Regs::RA_INVERT_CH]&CHAN2BIT(chan)) != 0;
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

	return (regs->regs[Ads5294Regs::RA_LFNS]&CHAN2BIT(chan)) != 0;
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
	Reg& frame = regs->regs[Ads5294Regs::RA_EN_FRAME];

	reg &= ~0x03;
	if (enable){
		reg |= 0x01;

		frame = (1<<RA_EN_FRAME_BIT)|0x1555;
	}else{
		frame = 0;
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

#define WIRE_MODE_2WIRE	(1<<0)
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
	for (unsigned ikey = 0; ikey < NKEYS; ++ikey){
		const char* k = keyLut[ikey].key;
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
			for (unsigned ikey = 0; ikey < NKEYS; ++ikey){
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

	for (unsigned ikey = 0; ikey < NKEYS; ++ikey){
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
	return 0;
}
int Ads5294::setReg(unsigned reg, unsigned pattern)
{
	printf("setReg() %02x %04x\n", reg, pattern);
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
 { "MAP_CH1234_TO_OUT1A", Ads5294Regs::RA_MAP_50, 0, false },
 { "MAP_CH1234_TO_OUT1B", Ads5294Regs::RA_MAP_50, 4, false },
 { "MAP_CH1234_TO_OUT2A", Ads5294Regs::RA_MAP_50, 8, false },
 { "MAP_CH1234_TO_OUT2B", Ads5294Regs::RA_MAP_51, 0, false },
 { "MAP_CH1234_TO_OUT3A", Ads5294Regs::RA_MAP_51, 4, false },
 { "MAP_CH1234_TO_OUT3B", Ads5294Regs::RA_MAP_51, 8, false },
 { "MAP_CH1234_TO_OUT4A", Ads5294Regs::RA_MAP_52, 0, false },
 { "MAP_CH1234_TO_OUT4B", Ads5294Regs::RA_MAP_52, 4, false },

 { "MAP_CH5678_TO_OUT5A", Ads5294Regs::RA_MAP_53, 4, true },
 { "MAP_CH5678_TO_OUT5B", Ads5294Regs::RA_MAP_53, 0, true },

 { "MAP_CH5678_TO_OUT6A", Ads5294Regs::RA_MAP_54, 0, true },
 { "MAP_CH5678_TO_OUT6B", Ads5294Regs::RA_MAP_53, 8, true },

 { "MAP_CH5678_TO_OUT7A", Ads5294Regs::RA_MAP_54, 8, true },
 { "MAP_CH5678_TO_OUT7B", Ads5294Regs::RA_MAP_54, 4, true },

 { "MAP_CH5678_TO_OUT8A", Ads5294Regs::RA_MAP_55, 4, true },
 { "MAP_CH5678_TO_OUT8B", Ads5294Regs::RA_MAP_55, 0, true }
};
#define NMAPLUT	(sizeof(maplut)/sizeof(MapLut))


struct ChannelMapEncoding {
	int chan;
	int two_bit;
	unsigned pattern;
};
ChannelMapEncoding cme[] = {
	{ 0, 0, 0x8 },			// power down
	{ 1, 0, 0x0 }, { 1, 2, 0x1 },	// channel 1
	{ 2, 0, 0x2 }, { 2, 2, 0x3 },
	{ 3, 0, 0x4 }, { 3, 2, 0x5 },
	{ 4, 0, 0x6 }, { 4, 2, 0x7 },

	{ 5, 0, 0x6 }, { 5, 2, 0x7 },
	{ 6, 0, 0x4 }, { 6, 2, 0x5 },
	{ 7, 0, 0x2 }, { 7, 2, 0x3 },
	{ 8, 0, 0x0 }, { 8, 2, 0x1 }
};
#define NCME	(sizeof(cme)/sizeof(ChannelMapEncoding))

const Ads5294::MapLut& Ads5294::lookupMap(const char* key){
	for (unsigned ik = 0; ik < NMAPLUT; ++ik){
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
	for (unsigned ik = 0; ik < NMAPLUT; ++ik){
		MapLut& map = maplut[ik];
		printf("%s %s %s %s # %s %s\n",
			pfx, map.key,
			map.hi_chan? "5": "1",
			"0",
			map.hi_chan? "5678": "1234",
			"02");
	}
	printf("%s ALL 0\n", pfx);
}

void Ads5294::printMap(const Ads5294::MapLut& map)
{
	Reg& _reg = regs->regs[map.reg];
	unsigned pat = (_reg >> map.shl) & 0x0f;
	printf("%s ", map.key);
	for (unsigned ix = 0; ix < NCME; ++ix){
		if (pat == cme[ix].pattern && isValidChx(map, cme[ix].chan)){
			if (cme[ix].chan!=0){
				printf("%d", cme[ix].chan);
				if (cme[ix].chan!=0 && cme[ix].two_bit==2){
					printf(" 2");
				}
			}
			printf("\n");
			return;
		}
	}
	printf("ERROR: illegal pattern %x\n", pat);
}

void Ads5294::printMap(int imap)
{
	printMap(maplut[imap]);
}

bool Ads5294::isValidChx(const Ads5294::MapLut& map, int chx, bool warn)
{
	if (chx == 0) return true;

	bool ok = false;

	if (map.hi_chan){
		ok = chx >= 5 && chx <= 8;
	}else{
		ok = chx >= 1 && chx <= 4;
	}

	if (!ok && warn){
		printf("WARNING: \"%s\" and %d not a valid combo\n", map.key, chx);
	}
	return ok;
}


int Ads5294::setMap(const char* mapping, int chx, int two_bits)
{

	if (strcmp(mapping, MAP_ALL) == 0 || strncmp(mapping, "ALL", 3) == 0 ){
		int im0 = 0;
		int im1 = NMAPLUT;

		// else all regs .. value for 0,0,0,0 only

		for (int imap = im0; imap < im1; ++imap){
			setMap(maplut[imap].key, chx, two_bits);
		}
		return 1;
	}else{
		const MapLut& map = lookupMap(mapping);

		if (!isValidChx(map, chx, true)){
			return 0;
		}

		for (unsigned ix = 0; ix < NCME; ++ix){
			if (chx == cme[ix].chan && two_bits == cme[ix].two_bit){
				Reg& _reg = regs->regs[map.reg];
				_reg &= ~ (0xf << map.shl);
				_reg |= cme[ix].pattern << map.shl;
				_reg |= 1 << MAP_EN_BIT;
				return 1;
			}
		}

		printf("WARNING: unable to match %s %d %d\n", mapping, chx, two_bits);
		return 0;
	}
}


int Ads5294::getMap(const char* mapping)
{
	if (strcmp(mapping, MAP_ALL) == 0){
		for (unsigned imap = 0; imap < NMAPLUT; ++imap){
			printMap(imap);
		}
	}else{
		printMap(lookupMap(mapping));
	}
	return 0;
}



Ads5294::PllRange Ads5294::pllRanges[4][4] = {
	/* Decimation 1 */
	{
		{ 28, 9999, 	0x0240 },
		{ 18, 42,	0x0140 },
		{  9, 24, 	0x00C0 },
		{  0, 12, 	0x0040 },
	},
	/* Decimation 2 */
	{
		{ 56, 9999, 	0x0240 },
		{ 36, 80,	0x0140 },
		{ 18, 48, 	0x00C0 },
		{  0, 24, 	0x0040 },
	},
	/* Decimation 4 */
	{
		{ 72, 9999,	0x0140 },
		{ 36, 8, 	0x00C0 },
		{  0, 48, 	0x0040 },
	},
	/* Decimation 8 */
	{
		{ 72, 80, 	0x00C0 },
		{  0, 80, 	0x0040 },
	},
};

int Ads5294::setPLL(struct PllRange range[], int Fs)
{
	for (int ii = 0; ; ++ii){
		if (Fs >= range[ii].fmin && Fs <= range[ii].fmax){
			Reg& _reg = regs->regs[Ads5294Regs::RA_PLL];
			_reg = range[ii].pat;
			return 1;
		}
		if (range[ii].fmin == 0){
			break;
		}
	}

	printf("ERROR: range match NOT FOUND\n");
	return 0;
}

int Ads5294::setPLL(int Fs, int decimation)
{
	switch(decimation){
	case 8:
		return setPLL(pllRanges[3], Fs);
	case 4:
		return setPLL(pllRanges[2], Fs);
	case 2:
		return setPLL(pllRanges[1], Fs);
	case 1:
	default:
		return setPLL(pllRanges[0], Fs);
	}

}
int Ads5294::getPLL()
{
	Reg& _reg = regs->regs[Ads5294Regs::RA_PLL];
	printf("getPLL() %02x = %04x\n", Ads5294Regs::RA_PLL, _reg);
	return 0;
}
int Ads5294::setTwoWireMode(bool enable)
{
	Reg& _bitorder = regs->regs[Ads5294Regs::RA_BITORDER];
	Reg& _wiremode = regs->regs[Ads5294Regs::RA_WIRE_MODE];

	if (enable){
		_bitorder = 0x8000;
		_wiremode |= WIRE_MODE_2WIRE;
	} else {
		_bitorder = 0x0000;
		_wiremode &= ~WIRE_MODE_2WIRE;
	}
	return 0;
}

