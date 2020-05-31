# Bluetrackr notes

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

## Power cycling

#### Power management

2 choices:
 - [ ] A switch to switch the power on or off
 - [x] Automatically switch off power on accelerometer idle

Setup an interrupt on the MPU9250 to trigger when the device is moved, somehow.

#### Charging

TP4056 cannot charge the battery while it is under load. (Discovered after the first prototype)

Solution: Use the +Vin pins to detect USB power, and use it instead of the battery power. The regulator can handle it. Circuit with diodes.

## Programming

#### Power

Don't use power from the reset connector

#### Reset

2 choices:
  - [x] Use the button
  - [x] Reset button on NRST pin

Use both.


## Interrupts

MPU_INT is active high. There is no pull down internally.
On rising edge it means it's triggered.

NRF_INT is active low. There is a pull up resistor internaly.
On falling edge it means it's triggered.

## LEDs

 - Status LED (blue) blinks when the device first connect. Off in normal state.
 - Maybe add a shake function to show status? On shake detection, it blinks to show the connection status.
 - Power LED blinks fast when there is an error
 - Power LED is slowly blinking when the power is low.



## Pairing process

Press the pairing button on both devices at the same time.
 - The **receiver** begins transmitting on the last channel. It broadcasts the (fixed) channel it's going to take in a few seconds.
 - The **transmitter** switches to the channel and pairing is done.

The channels paired are stored in the backup registers of the µc. (See [reference](https://www.st.com/content/ccc/resource/technical/document/reference_manual/59/b9/ba/7f/11/af/43/d5/CD00171190.pdf/files/CD00171190.pdf/jcr:content/translations/en.CD00171190.pdf) page 83.)