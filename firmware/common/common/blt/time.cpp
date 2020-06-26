#include <blt/time.hh>

#include <blt/hal_include.hh>

/**
 * Provides functionality around time: software timers, delays, ...
 */
namespace blt::time {

/**
 * Initializes the time functions.
 */
void init()
{
#ifdef DWT
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
#if F7
    DWT->LAR = 0xC5ACCE55;
#endif
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
void delay(const Microseconds& t)
{
#ifdef DWT
  const uint32_t startTick  = DWT->CYCCNT;
  const uint32_t delayTicks = t.us * (SystemCoreClock / 1000000);

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