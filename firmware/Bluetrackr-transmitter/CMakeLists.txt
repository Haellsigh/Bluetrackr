cmake_minimum_required(VERSION 3.17.3)

add_executable(Bluetrackr-transmitter.elf)

# C++20
target_compile_features(Bluetrackr-transmitter.elf PUBLIC cxx_std_20)
set_target_properties(Bluetrackr-transmitter.elf PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(Bluetrackr-transmitter.elf PRIVATE -Wall -Wextra)

add_subdirectory(Inc)
add_subdirectory(Src)

target_link_libraries(Bluetrackr-transmitter.elf PRIVATE Bluetrackr-common)

target_link_libraries(Bluetrackr-transmitter.elf PRIVATE
  HAL::STM32::F3::UART
  HAL::STM32::F3::SPI
  HAL::STM32::F3::RCCEx
  HAL::STM32::F3::GPIO
  HAL::STM32::F3::CORTEX
  CMSIS::STM32::F303K8
  STM32::NoSys
)

