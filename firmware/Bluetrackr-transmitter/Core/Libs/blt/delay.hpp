#ifndef BLT_DELAY_H_
#define BLT_DELAY_H_

#include "stm32f1xx_hal.h"

#include <ratio>

namespace blt {

class delay {
 public:
  /**
   * \brief Initializes the delay functions.
   */
  static void init() {
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
      CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
      DWT->CYCCNT = 0;
      DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
  }

  /**
   * \brief Delays for us microseconds.
   * \param t The delay in microseconds.
   */
  static const void us(uint32_t t) {
    uint32_t startTick = DWT->CYCCNT;
    uint32_t delayTicks = t * (SystemCoreClock / 1000000);

    while (DWT->CYCCNT - startTick < delayTicks)
      ;
  }

  /**
   * \brief Delays for ms milliseconds.
   * \param t The delay in milliseconds.
   */
  static const void ms(uint32_t t) { us(t * 1000); }
};

}  // namespace blt

#endif /* BLT_DELAY_H_ */
