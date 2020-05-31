cmake_minimum_required(VERSION 3.17.3)

add_library(Bluetrackr-common INTERFACE)

target_compile_options(Bluetrackr-common INTERFACE -Wall -Wextra)

target_include_directories(Bluetrackr-common INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_sources(Bluetrackr-common INTERFACE
  mpu9250.h
  mpu9250.cpp
  blt/bit.hh
  blt/gpio.hh
  blt/hal_include.hh
  blt/time.hh
  blt/utils.hh
  nrf24/nrf24.c
  nrf24/nrf24.h
  nrf24_custom/nrf24.hh
  nrf24_custom/nrf24_impl.hpp
  spi/spi.hh
)
