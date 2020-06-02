target_include_directories(Bluetrackr-transmitter.elf PRIVATE /)
target_sources(Bluetrackr-transmitter.elf PRIVATE
  main.h
  error_handler.hh
  main_loop.hh
  mpu.h
  nrf24_hal.h
  stm32f1xx_hal_conf.h
  stm32f1xx_it.h
)
