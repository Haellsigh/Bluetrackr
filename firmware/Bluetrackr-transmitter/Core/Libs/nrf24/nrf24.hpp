#ifndef BLT_LIBS_NRF24_H_
#define BLT_LIBS_NRF24_H_

#include <cstdint>

#include <blt/gpio.hpp>

namespace blt {

namespace nrf24 {

#define nRF24_TEST_ADDR "nRF24"

namespace Instruction {
enum : uint8_t {
  kReadRegister = 0x00,
  kWriteRegister = 0x20,
  kReadRxPayload = 0x61,
  kWriteTxPayload = 0xA0,
  kFlushTx = 0xE1,
  kFlushRx = 0xE2,
  kReuseTxPayload = 0xE3,
  kLockUnlock = 0x50,
  kNop = 0xFF
};
}

namespace Register {
enum : uint8_t {
  kConfig = 0x00,
  kEnableAutoAck = 0x01,
  kEnableRxAddresses = 0x02,
  kSetupAw = 0x03,
  kSetupRetries = 0x04,
  kRfChannel = 0x05,
  kRfSetup = 0x06,
  kStatus = 0x07,
  kObserveTx = 0x08,
  kRPD = 0x09,
  kRxAddrP0 = 0x0A,
  kRxAddrP1 = 0x0B,
  kRxAddrP2 = 0x0C,
  kRxAddrP3 = 0x0D,
  kRxAddrP4 = 0x0E,
  kRxAddrP5 = 0x0F,
  kTxAddr = 0x10,
  kRxPwP0 = 0x11,
  kRxPwP1 = 0x12,
  kRxPwP2 = 0x13,
  kRxPwP3 = 0x14,
  kRxPwP4 = 0x15,
  kRxPwP5 = 0x16,
  kFifoStatus = 0x17,
  kDynPd = 0x1C,
  kFeature = 0x1D
};
}

namespace RegisterMask {
enum : uint8_t {
  kMap = 0x1F,
  kCrc = 0x0C,
  kStatusIRQ = 0x70,
  kRfPower = 0x06,
  kRxPayloadNumber = 0x0E,
  kDatarate = 0x28,
};
}

// Enumeration of RX pipe addresses and TX address
namespace Pipe {
enum : uint8_t {
  kRx0 = 0x00,
  kRx1 = 0x01,
  kRx2 = 0x02,
  kRx3 = 0x03,
  kRx4 = 0x04,
  kRx5 = 0x05,
  kTx = 0x06
};
}

// Addresses of the address registers
static const uint8_t nRF24_ADDR_REGS[7] = {
    Register::kRxAddrP0, Register::kRxAddrP1, Register::kRxAddrP2,
    Register::kRxAddrP3, Register::kRxAddrP4, Register::kRxAddrP5,
    Register::kTxAddr};

namespace RetransmitDelay {
enum : uint8_t {
  kNone = 0x00,
  k250us = 0x00,
  k500us = 0x01,
  k750us = 0x02,
  k1000us = 0x03,
  k1250us = 0x04,
  k1500us = 0x05,
  k1750us = 0x06,
  k2000us = 0x07,
  k2250us = 0x08,
  k2500us = 0x09,
  k2750us = 0x0A,
  k3000us = 0x0B,
  k3250us = 0x0C,
  k3500us = 0x0D,
  k3750us = 0x0E,
  k4000us = 0x0F
};
}

namespace DataRate {
enum : uint8_t { k250kbps = 0x20, k1Mbps = 0x00, k2Mbps = 0x08 };
}

namespace TransmitPower {
enum : int8_t {
  kZero_dB = 0,
  kMinus_6_dB = -6,
  kMinus_12_dB = -12,
  kMinus_18_dB = -18
};
}

/**
 * \brief nRF24L01 driver class.
 *
 * \tparam spidevice The spi device driver.
 * \tparam csn       The chip select negative (csn) gpio pin.
 * \tparam ce        The chip enable gpio pin.
 */
template <typename spidevice, typename csn, typename ce>
class device {
  // Invert and setup a settling time of 100 µs
  using cs = gpio::settle<gpio::invert<csn>, 100>;

 public:
  device(SPI_HandleTypeDef* hSpi) : m_hSpi(hSpi){};

  void init();
  bool test();

  void enableAutoRetransmit(uint8_t delay, uint8_t count);
  void disableAutoRetransmit();

  bool setDataRate(uint8_t dataRate);

 private:
  /**
   * Helper function to write then read data from the spi device.
   */
  uint8_t inline spirw(uint8_t data) { return spidevice::rw(m_hSpi, data); }

  uint8_t readRegister(uint8_t reg);
  void readRegister(uint8_t reg, uint8_t* buf, uint8_t len);

  void writeRegister(uint8_t reg, uint8_t val);
  void writeRegister(uint8_t reg, uint8_t* buf, uint8_t len);

  void flushTx();
  void flushRx();

  void clearIRQFlags();

  uint8_t getAddressWidth();
  void setAddress(uint8_t pipe, const uint8_t* addr);

 private:
  SPI_HandleTypeDef* m_hSpi;
};

}  // namespace nrf24

}  // namespace blt

#include "nrf24_impl.hpp"

#endif  // BLT_LIBS_NRF24_H_
