target_sources(Bluetrackr-receiver.elf PRIVATE
  main.c
  main_loop.cpp
  nrf24_hal.c
  stm32f0xx_hal_msp.c
  stm32f0xx_it.c
)
