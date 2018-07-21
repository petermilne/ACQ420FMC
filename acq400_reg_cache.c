
#include "acq400.h"

#define REG_CACHE_BITS		32
#define REG_CACHE_BIT_MASK	0xFFFFFFFF
#define REG_CACHE_MAX		(REG_CACHE_MAP_REGS*REG_CACHE_BITS)

#define reg2map(reg, ix, bit) do { 	\
	ix = (reg)/REG_CACHE_BITS; 	\
	bit = (reg)&REG_CACHE_BIT_MASK; \
	} while(0)

#define map2reg(ix, bit) ((ix)*REG_CACHE_BITS + (1<<(bit)))




void acq400_rc_register(struct acq400_dev *adev, int reg_bytes)
{
	unsigned ix, bit;
	unsigned reg = reg_bytes/sizeof(unsigned);

	reg2map(reg, ix, bit);
	adev->reg_cache.map[ix] |= 1 << bit;
}

void acq400_rc_init(struct acq400_dev *adev)
{
	int ix, bit;
	int last = 0;

	for (ix = 0; ix < REG_CACHE_MAP_REGS; ++ix){
		for (bit = 0; bit < REG_CACHE_BITS; ++bit){
			if (adev->reg_cache.map[ix]&(1<<bit)){
				last = map2reg(ix, bit);
			}
		}
	}
	adev->reg_cache.data = kzalloc(last*sizeof(unsigned), GFP_KERNEL);
}

void acq400_rc_update(struct acq400_dev *adev)
{
	int ix, bit;

	for (ix = 0; ix < REG_CACHE_MAP_REGS; ++ix){
		for (bit = 0; bit < REG_CACHE_BITS; ++bit){
			unsigned reg = map2reg(ix, bit);
			adev->reg_cache.data[reg] =
				acq400rd32(adev, reg*sizeof(unsigned));
		}
	}
}


