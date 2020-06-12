cmake_minimum_required(VERSION 3.17.3)

project(Bluetrackr C CXX ASM)

find_package(CMSIS COMPONENTS STM32F0 STM32F1 REQUIRED)
find_package(HAL COMPONENTS STM32F0 STM32F1 REQUIRED)

add_subdirectory(Bluetrackr-common)
add_subdirectory(Bluetrackr-receiver)
add_subdirectory(Bluetrackr-transmitter)
