#pragma once

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void main_loop(ADC_HandleTypeDef* hadc,
               TIM_HandleTypeDef* htim,
               SPI_HandleTypeDef* hspi1,
               SPI_HandleTypeDef* hspi2);

#ifdef __cplusplus
}
#endif