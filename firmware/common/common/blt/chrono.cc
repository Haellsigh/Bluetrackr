#include <blt/chrono.hh>

/**
 * Provides functionality around time: software timers, delays, ...
 */
namespace blt::chrono {

/*
clock::time_point clock::now() noexcept
{
#ifdef DWT
  return time_point{duration{(static_cast<uint64_t>(DWT->CYCCNT) * 1000000) / SystemCoreClock}};
#else
  return time_point{duration{__HAL_TIM_GET_COUNTER(htim_)}};
#endif
}
*/

/*
void init(TIM_HandleTypeDef* htim) noexcept
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
  htim_ = htim;
  HAL_TIM_Base_Start(htim_);
#endif
}
*/

}  // namespace blt::chrono