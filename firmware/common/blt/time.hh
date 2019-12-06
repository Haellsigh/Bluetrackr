#ifndef BLT_TIME_H_
#define BLT_TIME_H_

#include <blt/hal_include.hh>

#include <ratio>

namespace blt {

/**
 * \brief Provides various delay functionnalities.
 */
namespace delay {
/**
 * \brief Initializes the delay functions.
 */
static void init() {
#ifdef DWT
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }
#endif
}

/**
 * \brief Delays for t microseconds.
 * \param t The delay in microseconds.
 */
static void us(uint32_t t) {
#ifdef DWT
  uint32_t startTick  = DWT->CYCCNT;
  uint32_t delayTicks = t * (SystemCoreClock / 1000000);

  while (DWT->CYCCNT - startTick < delayTicks)
    ;
#else
  SysTick->LOAD = t * (SystemCoreClock / 8000000);
  SysTick->VAL  = 0;
  SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;
  // COUNTFLAG is a bit that is set to 1 when counter reaches 0.
  // It's automatically cleared when read.
  while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0)
    ;
  SysTick->CTRL = 0;
#endif
}

/**
 * \brief Delays for t milliseconds.
 * \param t The delay in milliseconds.
 */
static inline void ms(uint32_t t) {
  us(t * 1000);
}
};  // namespace delay

namespace time {}

}  // namespace blt

#endif  // BLT_TIME_H_
