#include <nrf24/nrf24.hpp>

namespace blt {

namespace nrf24 {

template <typename spidevice, typename csn, typename ce>
uint8_t device<spidevice, csn, ce>::readRegister(uint8_t reg) {
  uint8_t result;

  cs::write(true);

  spirw(reg & RegisterMask::kMap);
  result = spirw(Instruction::kNop);

  cs::write(false);

  return result;
}

template <typename spidevice, typename csn, typename ce>
void device<spidevice, csn, ce>::readRegister(uint8_t reg,
                                              uint8_t* buf,
                                              uint8_t len) {
  cs::write(true);

  spirw(reg);
  while (len--) {
    *buf++ = spirw(Instruction::kNop);
  }

  cs::write(false);
}

template <typename spidevice, typename csn, typename ce>
void device<spidevice, csn, ce>::writeRegister(uint8_t reg, uint8_t val) {
  cs::write(true);

  // Register access
  if (reg < Instruction::kWriteRegister) {
    spirw(Instruction::kWriteRegister | (reg & RegisterMask::kMap));
    spirw(val);
  }
  // Singly byte command or future command/register
  else {
    spirw(reg);
    if ((reg != Instruction::kFlushTx) && (reg != Instruction::kFlushRx) &&
        (reg != Instruction::kReuseTxPayload) && (reg != Instruction::kNop)) {
      spirw(val);  // Send register value
    }
  }

  cs::write(false);
}

template <typename spidevice, typename csn, typename ce>
void device<spidevice, csn, ce>::writeRegister(uint8_t reg,
                                               uint8_t* buf,
                                               uint8_t len) {
  cs::write(true);

  spirw(reg);
  while (len--) {
    spirw(*buf++);
  }

  cs::write(false);
}

template <typename spidevice, typename csn, typename ce>
void device<spidevice, csn, ce>::flushTx() {
  writeRegister(Instruction::kFlushTx, Instruction::kNop);
}

template <typename spidevice, typename csn, typename ce>
void device<spidevice, csn, ce>::flushRx() {
  writeRegister(Instruction::kFlushRx, Instruction::kNop);
}

template <typename spidevice, typename csn, typename ce>
void device<spidevice, csn, ce>::clearIRQFlags() {
  uint8_t reg;

  reg = readRegister(Register::kStatus);
  reg |= RegisterMask::kStatusIRQ;
  writeRegister(Register::kStatus, reg);
}

template <typename spidevice, typename csn, typename ce>
uint8_t device<spidevice, csn, ce>::getAddressWidth() {
  return readRegister(Register::kSetupAw) + 2u;
}

template <typename spidevice, typename csn, typename ce>
void device<spidevice, csn, ce>::setAddress(uint8_t pipe, const uint8_t* addr) {
  uint8_t addr_width;

  switch (pipe) {
    case Pipe::kTx:
    case Pipe::kRx0:
    case Pipe::kRx1:
      addr_width = getAddressWidth();
      writeRegister(Instruction::kWriteRegister | nRF24_ADDR_REGS[pipe],
                    (uint8_t*)addr, addr_width);
      break;

    case Pipe::kRx2:
    case Pipe::kRx3:
    case Pipe::kRx4:
    case Pipe::kRx5:
      writeRegister(nRF24_ADDR_REGS[pipe], *addr);
      break;
    default:
      // Incorrect pipe, do nothing
      break;
  }
}

template <typename spidevice, typename csn, typename ce>
void device<spidevice, csn, ce>::init() {
  ce::write(false);
  cs::write(false);

  // Power up delay
  blt::delay::ms(5);

  /// Arbitrary default values for the registers
  // Reset config & enable 16-bit CRC
  writeRegister(Register::kConfig, 0x0C);
  // The minimum viable delay after testing on all speeds.
  enableAutoRetransmit(RetransmitDelay::k1500us, 15);
  setDataRate(DataRate::k2Mbps);
  //
  //
  //
  //
  //
  //
  //
  //
  //

  writeRegister(Register::kEnableAutoAck, 0x3F);
  writeRegister(Register::kEnableRxAddresses, 0x03);
  writeRegister(Register::kSetupAw, 0x03);
  writeRegister(Register::kRfChannel, 0x02);
  writeRegister(Register::kRfSetup, 0x0E);
  writeRegister(Register::kStatus, 0x00);
  writeRegister(Register::kRxPwP0, 0x00);
  writeRegister(Register::kRxPwP1, 0x00);
  writeRegister(Register::kRxPwP2, 0x00);
  writeRegister(Register::kRxPwP3, 0x00);
  writeRegister(Register::kRxPwP4, 0x00);
  writeRegister(Register::kRxPwP5, 0x00);
  writeRegister(Register::kDynPd, 0x00);
  writeRegister(Register::kFeature, 0x00);

  // Configure the default RX/TX address values
  uint8_t addr[5];
  uint8_t idx;
  for (idx = 0U; idx < sizeof(addr); idx++) {
    addr[idx] = 0xE7;
  }
  setAddress(Pipe::kTx, addr);
  setAddress(Pipe::kRx0, addr);
  for (idx = 0u; idx < sizeof(addr); idx++) {
    addr[idx] = 0xC2;
  }
  setAddress(Pipe::kRx1, addr);
  for (idx = 2u; idx < 6u; idx++) {
    addr[0] = idx + 0xC1;
    setAddress(idx, addr);
  }

  // Clear the FIFO's
  flushRx();
  flushTx();

  // Clear any pending interrupt flags
  clearIRQFlags();

  // Deassert CSN pin (chip release)
  cs::write(false);
}

template <typename spidevice, typename csn, typename ce>
bool device<spidevice, csn, ce>::test() {
  uint8_t rxbuf[5];
  uint8_t txbuf[5] = {'n', 'R', 'F', '2', '4'};
  constexpr uint8_t bufsize = 5;

  // Write the test address to the TxAddr register
  writeRegister(Instruction::kWriteRegister | Register::kTxAddr, txbuf,
                bufsize);

  // Read it back
  readRegister(Instruction::kReadRegister | Register::kTxAddr, rxbuf, bufsize);

  // Compare transmitted and received data
  for (uint8_t i = 0; i < bufsize; i++) {
    if (rxbuf[i] != txbuf[i])
      // Transmitter is absent or there is a problem.
      return false;
  }

  // Transmitter is present & working.
  return true;
}

/**
 * \brief Sets the automatic retransmission parameters.
 *
 * \param ard Delay, one of RetransmitDelay
 * \param arc Count, in range 0 - 15
 */
template <typename spidevice, typename csn, typename ce>
void device<spidevice, csn, ce>::enableAutoRetransmit(uint8_t ard,
                                                      uint8_t arc) {
  writeRegister(Register::kSetupRetries, ard << 4 | arc);
}

template <typename spidevice, typename csn, typename ce>
void device<spidevice, csn, ce>::disableAutoRetransmit() {
  writeRegister(Register::kSetupRetries, 0);
}

template <typename spidevice, typename csn, typename ce>
bool device<spidevice, csn, ce>::setDataRate(uint8_t dataRate) {
  uint8_t reg;

  // Read the current register value
  reg = readRegister(Register::kRfSetup);
  // Reset the datarate bits to 0
  reg &= ~RegisterMask::kDatarate;
  // Write the datarate to those bits
  reg |= dataRate;

  writeRegister(Register::kRfSetup, reg);

  // Verify our register was saved.
  return readRegister(Register::kRfSetup) == reg;
}

}  // namespace nrf24

}  // namespace blt
