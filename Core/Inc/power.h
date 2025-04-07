#ifndef __POWER_H
#define __POWER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void power_setup(void);
void power_adc_callback(void);
void power_ctl_callback(void);

#ifdef __cplusplus
}
#endif

#endif /* __POWER_H */