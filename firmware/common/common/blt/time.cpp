#include "time.hh"

#include <blt/hal_include.hh>

/**
 * Provides functionality around time: software timers, delays, ...
 */
namespace blt::time {

/**
 * Literals for durations
 */
namespace literals {

constexpr Microseconds operator"" us(uint64_t us) {
  return Microseconds{us};
}
constexpr Microseconds operator"" ms(uint64_t ms) {
  return Microseconds{1000 * ms};
}

}  // namespace literals

/**
 * Initializes the time functions.
 */
void init() {
#ifdef DWT
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }
#else
  // Nothing to initialize because the SysTick is initialized by the HAL.
#endif
}

/**
 * Pauses execution for a certain amount of time
 */
void delay(Microseconds t) {
#ifdef DWT
  uint32_t startTick  = DWT->CYCCNT;
  uint32_t delayTicks = t.us * (SystemCoreClock / 1000000);

  while (DWT->CYCCNT - startTick < delayTicks)
    ;
#else
  SysTick->LOAD = t.us * (SystemCoreClock / 8000000);
  SysTick->VAL  = 0;
  SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;
  // COUNTFLAG is a bit that is set to 1 when counter reaches 0.
  // It's automatically cleared when read.
  while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0)
    ;
  SysTick->CTRL = 0;
#endif
}

}  // namespace blt::time