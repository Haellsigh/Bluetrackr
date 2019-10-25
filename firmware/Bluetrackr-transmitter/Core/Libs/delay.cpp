#include <delay.h>

namespace blt {

void delayInit() {
	if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		DWT->CYCCNT = 0;
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	}
}

/*!
 * \brief Delays for us microseconds.
 * @param us The delay in microseconds.
 */
void delay(uint32_t us) {
	uint32_t startTick = DWT->CYCCNT;
	uint32_t delayTicks = us * (SystemCoreClock / 1000000);

	while (DWT->CYCCNT - startTick < delayTicks)
		;
}

} /* namespace blt */
