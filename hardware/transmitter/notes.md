# Bluetrackr-transmitter notes

## Components



- [x] STM32F103
- [x] nRF24L01+
- [x] TP4056 Battery charger
- [x] MPU9250
- [x] Programming header (4x 2.54mm)
- [x] Vbat Sensing
- [x] Pairing button
- [x] Status LED (blue)
- [x] Power LED (red) (blinks on low power)

## LEDs

 - Status LED (blue) is blinking when the device is connected. Off if it is not connected.
 - Power LED is slowly blinking when the power is low.

## Pairing process

Press the pairing button on both devices at the same time.
 - The **receiver** begins transmitting on the last channel. It broadcasts the (fixed) channel it's going to take in a few seconds.
 - The **transmitter** switches to the channel and pairing is done.


| Component | Schematic | PCB |
|-----------|:----------|:---:|
| STM32F103 | yes       | no |
| nRF24L01+ | yes       | no |
| TP4056 | yes       | no |
| MPU9250 | yes       | no |
| Programming header | yes       | no |
| Vbat sensing | yes       | no |
| Pairing button | yes       | no |
| Status LED (blue) | no       | no |
| Power LED (red) | no       | no |