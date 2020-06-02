target_sources(Bluetrackr-transmitter.elf PRIVATE
  main.c
  main_loop.cpp
  mpu.c
  nrf24_hal.c
  stm32f1xx_hal_msp.c
  stm32f1xx_it.c
)
