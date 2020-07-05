#pragma once

#if defined(STM32F0)
#include "stm32f0xx_hal.h"
#elif defined(STM32F3)
#include "stm32f3xx_hal.h"
#else
#error "Unsupported STM32 family"
#endif