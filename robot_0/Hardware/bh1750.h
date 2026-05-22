#ifndef __BH1750_H
#define __BH1750_H

#include "stm32f10x.h"

/* BH1750 ?? */
#define BH1750_POWER_ON             0x01
#define BH1750_POWER_RESET          0x07
#define BH1750_CONTINUOUS_HIGH_RES  0x10
#define BH1750_ONE_TIME_HIGH_RES    0x20

void  bh1750_init(void);
float bh1750_get_lux(void);

#endif
