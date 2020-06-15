#pragma once

#if F0
#include "stm32f0xx_hal.h"
#elif F3
#include "stm32f3xx_hal.h"
#else
#error "Unsupported STM32 family"
#endif