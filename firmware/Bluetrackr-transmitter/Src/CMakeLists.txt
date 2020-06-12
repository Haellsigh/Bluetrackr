target_sources(Bluetrackr-transmitter.elf PRIVATE
  main.c
  main_loop.cpp
  mpu.c
  nrf24_hal.c
  stm32f1xx_hal_msp.c
  stm32f1xx_it.c
  usb_device.c
  usbd_cdc_if.c
  usbd_conf.c
  usbd_desc.c
)

add_subdirectory(USB)