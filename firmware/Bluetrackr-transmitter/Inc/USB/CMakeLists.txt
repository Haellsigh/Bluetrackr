target_include_directories(Bluetrackr-transmitter.elf PRIVATE /)
target_sources(Bluetrackr-transmitter.elf PRIVATE
  usbd_cdc.h
  usbd_core.h
  usbd_ctlreq.h
  usbd_def.h
  usbd_ioreq.h
)
