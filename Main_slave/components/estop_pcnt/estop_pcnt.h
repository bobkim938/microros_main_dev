#ifndef _ESTOP_PCNT_
#define _ESTOP_PCNT_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "esp_system.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"

void initPCNT (const uint8_t input_a, const uint8_t input_b);

#ifdef __cplusplus
}
#endif

#endif //_ESTOP_PCNT_