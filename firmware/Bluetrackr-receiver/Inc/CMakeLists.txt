target_include_directories(Bluetrackr-receiver PRIVATE /)
target_sources(Bluetrackr-receiver PRIVATE
  main.h
  error_handler.hh
  main_loop.hh
  nrf24_hal.h
  stm32f0xx_hal_conf.h
  stm32f0xx_it.h
)
