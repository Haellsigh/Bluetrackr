#pragma once

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void main_loop(SPI_HandleTypeDef* hspi, TIM_HandleTypeDef* htim);

#ifdef __cplusplus
}
#endif