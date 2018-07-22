
#include "acq400.h"


int cache_sites;
module_param(cache_sites, int, 0444);
MODULE_PARM_DESC(cache_sites, "number of sites using reg cache");

int cache_sites_mask = 0xffff;
module_param(cache_sites_mask, int, 0644);
MODULE_PARM_DESC(cache_sites_mask, "inhibit update on these sites");


#define REG_CACHE_BITS		32
#define REG_CACHE_BIT_MASK	0x1F
#define REG_CACHE_MAX		(REG_CACHE_MAP_REGS*REG_CACHE_BITS)

#define reg2map(reg, ix, bit) do { 	\
	ix = (reg)/REG_CACHE_BITS; 	\
	bit = (reg)&REG_CACHE_BIT_MASK; \
	} while(0)

#define map2reg(ix, bit) ((ix)*REG_CACHE_BITS + (bit))




int dev_rc_register(struct device* dev, struct RegCache* reg_cache, int reg_bytes)
{
	unsigned ix, bit;
	unsigned reg = reg_bytes/sizeof(unsigned);
	if (reg < reg_cache->max_reg){
		reg2map(reg, ix, bit);
		reg_cache->map[ix] |= 1 << bit;
		return 0;
	}else{
		return -1;
	}
}

int dev_rc_init(struct device* dev,
		struct RegCache* reg_cache, void* va, int id, int reg_max_bytes)
{
	int max_reg = reg_max_bytes/sizeof(unsigned);
	if (max_reg > REG_CACHE_MAX) max_reg = REG_CACHE_MAP_REGS;

	cache_sites |= 1<<id;
	reg_cache->id = id;
	reg_cache->dev = dev;
	reg_cache->va = va;
	reg_cache->data = kzalloc(max_reg*sizeof(unsigned), GFP_KERNEL);
	reg_cache->max_reg = max_reg;
	return max_reg * sizeof(unsigned);
}

void dev_rc_update(struct device *dev, struct RegCache* reg_cache, unsigned* va)
{
	int ix, bit;

	for (ix = 0; ix < REG_CACHE_MAP_REGS; ++ix){
		if (reg_cache->map[ix] != 0){
			for (bit = 0; bit < REG_CACHE_BITS; ++bit){
				if (reg_cache->map[ix]&(1<<bit)){
					unsigned reg = map2reg(ix, bit);
					reg_cache->data[reg] = ioread32(va + reg);
				}
			}
		}
	}
}

enum hrtimer_restart dev_rc_timer_update(struct hrtimer* hrt)
{
	struct RegCache* reg_cache =
			container_of(hrt, struct RegCache, timer);
	if ((cache_sites_mask&(1<<reg_cache->id)) == 0){
		dev_rc_update(reg_cache->dev, reg_cache, reg_cache->va);
	}
	hrtimer_forward_now(hrt, ktime_set(1, 0));
	return HRTIMER_RESTART;
}

int dev_rc_finalize(struct device *dev, struct RegCache* reg_cache, int id)
{
	int ix, bit;
	int last = 0;

	for (ix = 0; ix < REG_CACHE_MAP_REGS; ++ix){
		for (bit = 0; bit < REG_CACHE_BITS; ++bit){
			if (reg_cache->map[ix]&(1<<bit)){
				last = map2reg(ix, bit);
			}
		}
	}
	dev_info(dev, "%s site:%d max:%d last:%d map:\n%08x\n%08x\n%08x\n%08x\n",
			__FUNCTION__, id, reg_cache->max_reg, last,
			reg_cache->map[0], reg_cache->map[1],
			reg_cache->map[2], reg_cache->map[3]);


	hrtimer_init(&reg_cache->timer, CLOCK_REALTIME, HRTIMER_MODE_REL);
	reg_cache->timer.function = dev_rc_timer_update;
	hrtimer_start(&reg_cache->timer, ktime_set(0, id*10*NSEC_PER_MSEC), HRTIMER_MODE_REL);

	dev_info(dev, "%s last %d", __FUNCTION__, last);
	return last;
}




