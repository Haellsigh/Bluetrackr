#ifndef BLT_DELAY_H_
#define BLT_DELAY_H_

#include "stm32f1xx_hal.h"

namespace blt {

void delayInit();
void delay(uint32_t us);

} /* namespace blt */

#endif /* BLT_DELAY_H_ */
