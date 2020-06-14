#ifndef BLT_MAIN_LOOP_H_
#define BLT_MAIN_LOOP_H_

#include "stm32f0xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void main_loop(SPI_HandleTypeDef* hspi);

#ifdef __cplusplus
}
#endif

#endif /* BLT_MAIN_LOOP_H_ */
