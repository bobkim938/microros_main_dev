#ifndef _KINCO_CAN_
#define _KINCO_CAN_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
#include "esp_system.h"
#include "driver/twai.h"

void initTwai (const uint8_t tx, const uint8_t rx);
void setModesOfOperation (const uint8_t id, const int8_t msg); //6060
void setTargetVelocity (const uint8_t id, const int32_t msg); //60FF
void setProfileAcceleration (const uint8_t, const uint32_t msg); //6083
void setProfileDeceleration (const uint8_t, const uint32_t msg); //6084
void setTargetPosition (const uint8_t id, const int32_t msg); //607A
void setProfileSpeed (const uint8_t id, const uint32_t msg); //6081
void setControlWord (const uint8_t id, const uint16_t msg); //6040 
uint16_t getStatusWord (const uint8_t id); //6041
void setDin2Function (const uint8_t id, const uint16_t msg); //2010:04
void setDinSimulate (const uint8_t id, const uint16_t msg); //2010:02

#ifdef __cplusplus
}
#endif

#endif //_KINCO_CAN_