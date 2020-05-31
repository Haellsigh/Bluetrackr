cmake_minimum_required(VERSION 3.17.3)

add_executable(Bluetrackr-transmitter)

# C++20
target_compile_features(Bluetrackr-transmitter PUBLIC cxx_std_20)
set_target_properties(Bluetrackr-transmitter PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(Bluetrackr-transmitter PRIVATE -Wall -Wextra)

add_subdirectory(Inc)
add_subdirectory(Src)

target_link_libraries(Bluetrackr-transmitter PRIVATE Bluetrackr-common)

target_link_libraries(Bluetrackr-transmitter PRIVATE
  HAL::STM32::F1::ADC
  HAL::STM32::F1::SPI
  HAL::STM32::F1::RCCEx
  HAL::STM32::F1::GPIO
  HAL::STM32::F1::CORTEX
  CMSIS::STM32::F103C8
  STM32::NoSys
)

