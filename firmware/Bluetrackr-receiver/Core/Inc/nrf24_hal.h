#ifndef __NRF24_HAL_H
#define __NRF24_HAL_H

// Hardware abstraction layer for NRF24L01+ transceiver (hardware depended functions)
// GPIO pins definition
// GPIO pins initialization and control functions
// SPI transmit functions

// Peripheral libraries & user variables
#include "main.h"

// SPI port peripheral
#define nRF24_SPI_PORT SPI2

// CE (chip enable) pin
#define nRF24_CE_PORT NRF_CE_GPIO_Port
#define nRF24_CE_PIN NRF_CE_Pin
#define nRF24_CE_L HAL_GPIO_WritePin(nRF24_CE_PORT, nRF24_CE_PIN, GPIO_PIN_RESET)
#define nRF24_CE_H HAL_GPIO_WritePin(nRF24_CE_PORT, nRF24_CE_PIN, GPIO_PIN_SET)

// CSN (chip select negative) pin
#define nRF24_CSN_PORT NRF_CSN_GPIO_Port
#define nRF24_CSN_PIN NRF_CSN_Pin
#define nRF24_CSN_L HAL_GPIO_WritePin(nRF24_CSN_PORT, nRF24_CSN_PIN, GPIO_PIN_RESET)
#define nRF24_CSN_H HAL_GPIO_WritePin(nRF24_CSN_PORT, nRF24_CSN_PIN, GPIO_PIN_SET)

// IRQ pin (PB10)
#define nRF24_IRQ_PORT NRF_INT_GPIO_Port
#define nRF24_IRQ_PIN NRF_INT_Pin

// Macros for the RX on/off
#define nRF24_RX_ON nRF24_CE_H
#define nRF24_RX_OFF nRF24_CE_L

// Function prototypes
void    nRF24_LL_INIT(SPI_HandleTypeDef* hspi);
uint8_t nRF24_LL_RW(uint8_t data);

#endif  // __NRF24_HAL_H
