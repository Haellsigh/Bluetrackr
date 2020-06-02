cmake_minimum_required(VERSION 3.17.3)

add_executable(Bluetrackr-receiver.elf)

# C++20
target_compile_features(Bluetrackr-receiver.elf PUBLIC cxx_std_20)
set_target_properties(Bluetrackr-receiver.elf PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(Bluetrackr-receiver.elf PRIVATE -Wall -Wextra)

add_subdirectory(Inc)
add_subdirectory(Src)

target_link_libraries(Bluetrackr-receiver.elf PRIVATE Bluetrackr-common)

target_link_libraries(Bluetrackr-receiver.elf PRIVATE
  HAL::STM32::F0::SPI
  HAL::STM32::F0::RCC
  HAL::STM32::F0::GPIO
  HAL::STM32::F0::CORTEX
  CMSIS::STM32::F072RB
  STM32::NoSys
)

