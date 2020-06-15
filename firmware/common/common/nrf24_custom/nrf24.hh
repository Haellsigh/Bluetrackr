/*!
 * \file
 * \brief nRF24L01+ library header
 */

#ifndef BLT_LIBS_NRF24_H_
#define BLT_LIBS_NRF24_H_

#include <cstdint>

#include <blt/bit.hh>
#include <blt/gpio.hh>

namespace blt {

namespace nrf24 {

#define nRF24_TEST_ADDR "nRF24"

namespace Command {
/*!
 * \name Command
 * \brief SPI commands.
 */
enum Command : uint8_t {
  kNone                = 0b00000000,
  kDisable             = 0b00000000,
  kReadRegister        = 0b00000000,
  kWriteRegister       = 0b00100000,
  kReadRxPayload       = 0b01100001,
  kWriteTxPayload      = 0b10100000,
  kWriteTxPayloadNoAck = 0b10110000,
  kFlushRx             = 0b11100010,
  kFlushTx             = 0b11100001,
  kReuseTxPayload      = 0b11100011,
  kNop                 = 0b11111111
  // kLockUnlock     = 0x50, // What is this, what is it used for?
};
}  // namespace Command

static constexpr uint8_t kReadWriteMask = 0b00011111;

namespace CommandMask {
/*!
 * \name CommandMask
 * \brief SPI commands masks.
 */
enum CommandMask : uint8_t {
  kRegisterReadWrite = 0b00011111,  //!< kRegisterReadWrite
};
}  // namespace CommandMask

namespace Register {
/*!
 * \name Register
 */
enum Register : uint8_t {
  kConfig                = 0x00,
  kEnableAutoAck         = 0x01,
  kEnabledRxAddresses    = 0x02,
  kSetupAddressWidths    = 0x03,
  kSetupAutoRet          = 0x04,
  kRfChannel             = 0x05,
  kRfSetup               = 0x06,
  kStatus                = 0x07,
  kObserveTx             = 0x08,
  kReceivedPowerDetector = 0x09,
  kRxAddressP0           = 0x0A,
  kRxAddressP1           = 0x0B,
  kRxAddressP2           = 0x0C,
  kRxAddressP3           = 0x0D,
  kRxAddressP4           = 0x0E,
  kRxAddressP5           = 0x0F,
  kTxAddress             = 0x10,
  kRxPayloadWidthP0      = 0x11,
  kRxPayloadWidthP1      = 0x12,
  kRxPayloadWidthP2      = 0x13,
  kRxPayloadWidthP3      = 0x14,
  kRxPayloadWidthP4      = 0x15,
  kRxPayloadWidthP5      = 0x16,
  kFifoStatus            = 0x17,
  kDynamicPayload        = 0x1C,
  kFeature               = 0x1D
};
}  // namespace Register

namespace RegisterFieldBit {
/*!
 * \name RegisterFieldBit
 * \brief Describes the field position in the register.
 */
enum RegisterFieldBit : uint8_t {
  kConfigMaskRxDataReady  = 6,       //!< kConfig
  kConfigMaskTxDataSent   = 5,       //!< kConfig
  kConfigMaskMaxRet       = 4,       //!< kConfig
  kConfigEnableCrc        = 3,       //!< kConfig
  kConfigCrcScheme        = 2,       //!< kConfig
  kConfigPower            = 1,       //!< kConfig
  kConfigPrimary          = 0,       //!< kConfig
  kSetupAddressWidths     = 0,       //!< kSetupAddressWidths
  kSetupAutoRetArd        = 4,       //!< kSetupAutoRet
  kSetupAutoRetArc        = 0,       //!< kSetupAutoRet
  kRfChannel              = 0,       //!< kRfChannel
  kRfSetupContWave        = 7,       //!< kRfSetup, Testing only.
  kRfSetupRfDatarateLow   = 5,       //!< kRfSetup
  kRfSetupPllLock         = 4,       //!< kRfSetup, Testing only.
  kRfSetupRfDatarateHigh  = 3,       //!< kRfSetup
  kRfSetupRfPower         = 1,       //!< kRfSetup
  kStatusRxDataReady      = 6,       //!< kStatus
  kStatusTxDataSent       = 5,       //!< kStatus
  kStatusMaxRetransmits   = 4,       //!< kStatus
  kStatusRxPayloadPipeN   = 1,       //!< kStatus
  kStatusTxFull           = 0,       //!< kStatus
  kObserveTxPacketLossCnt = 4,       //!< kObserveTx
                                     //!< Overflows at 15. Reset by writing to
                                     //!< kRfChannel.
  kObserveTxAutoRetransmitCnt  = 0,  //!< kObserveTx, Reset by a new packet.
  kReceivedPowerDetector       = 0,  //!< kReceivedPowerDetector
  kRxPayloadWidth              = 0,  //!< kRxPayloadWidthP0 to P5
  kFifoStatusTxReuse           = 6,  //!< kFifoStatus
  kFifoStatusTxFull            = 5,  //!< kFifoStatus
  kFifoStatusTxEmpty           = 4,  //!< kFifoStatus
  kFifoStatusRxFull            = 1,  //!< kFifoStatus
  kFifoStatusRxEmpty           = 0,  //!< kFifoStatus
  kFeatureEnableDynamicPayload = 2,  //!< kFeature
  kFeatureEnableAckPayload     = 1,  //!< kFeature
  kFeatureEnableDynamicAck     = 0   //!< kFeature
};
}  // namespace RegisterFieldBit

namespace RegisterFieldMask {
using namespace blt::bit;
/*!
 * \name RegisterFieldMask
 */
enum RegisterFieldMask : uint8_t {
  kConfigMaskRxDataReady  = maskBits<6>(),           //!< kConfig
  kConfigMaskTxDataSent   = maskBits<5>(),           //!< kConfig
  kConfigMaskMaxRet       = maskBits<4>(),           //!< kConfig
  kConfigEnableCrc        = maskBits<3>(),           //!< kConfig
  kConfigCrcScheme        = maskBits<2>(),           //!< kConfig
  kConfigPower            = maskBits<1>(),           //!< kConfig
  kConfigPrimary          = maskBits<0>(),           //!< kConfig
  kSetupAddressWidths     = maskRange<1, 0>(),       //!< kSetupAddressWidths
  kSetupAutoRetArd        = maskRange<7, 4>(),       //!< kSetupAutoRet
  kSetupAutoRetArc        = maskRange<3, 0>(),       //!< kSetupAutoRet
  kRfChannel              = maskRange<6, 0>(),       //!< kRfChannel
  kRfSetupRfContWave      = maskBits<7>(),           //!< kRfSetup, Testing only.
  kRfSetupRfDatarateLow   = maskBits<5>(),           //!< kRfSetup
  kRfSetupRfPllLock       = maskBits<4>(),           //!< kRfSetup, Testing only.
  kRfSetupRfDatarateHigh  = maskBits<3>(),           //!< kRfSetup
  kRfSetupRfPower         = maskRange<2, 1>(),       //!< kRfSetup
  kStatusRxDataReady      = maskBits<6>(),           //!< kStatus
  kStatusTxDataSent       = maskBits<5>(),           //!< kStatus
  kStatusMaxRetransmits   = maskBits<4>(),           //!< kStatus
  kStatusRxPayloadPipeN   = maskRange<1, 3>(),       //!< kStatus
  kStatusTxFull           = maskBits<0>(),           //!< kStatus
  kObserveTxPacketLossCnt = maskRange<7, 4>(),       //!< Overflows at 15.
                                                     //!< Reset by writing to
                                                     //!< kRfChannel.
  kObserveTxAutoRetransmitCnt  = maskRange<3, 0>(),  //!< Reset by a new packet.
  kReceivedPowerDetector       = maskBits<0>(),      //!< kReceivedPowerDetector
  kRxPayloadWidth              = maskRange<5, 0>(),  //!< kRxPayloadWidthP0 to P5
  kFifoStatusTxReuse           = maskBits<6>(),      //!< kFifoStatus
  kFifoStatusTxFull            = maskBits<5>(),      //!< kFifoStatus
  kFifoStatusTxEmpty           = maskBits<4>(),      //!< kFifoStatus
  kFifoStatusRxFull            = maskBits<1>(),      //!< kFifoStatus
  kFifoStatusRxEmpty           = maskBits<0>(),      //!< kFifoStatus
  kFeatureEnableDynamicPayload = maskBits<2>(),      //!< kFeature
  kFeatureEnableAckPayload     = maskBits<1>(),      //!< kFeature
  kFeatureEnableDynamicAck     = maskBits<0>()       //!< kFeature
};
}  // namespace RegisterFieldMask

namespace RegisterMask {
using namespace blt::bit;
/*!
 * \name RegisterMask
 * \brief Register
 */
enum RegisterMask : uint8_t {
  kRegister        = 0x1F,  //!< kRegister
  kCrc             = 0x0C,  //!< kCrc
  kStatusIRQ       = 0x70,  //!< kStatusIRQ
  kRfPower         = 0x06,  //!< kRfPower
  kRxPayloadNumber = 0x0E,  //!< kRxPayloadNumber
  kDatarate        = maskBits<RegisterFieldBit::kRfSetupRfDatarateLow,
                       RegisterFieldBit::kRfSetupRfDatarateHigh>(),  //!< kDatarate
};
}  // namespace RegisterMask

namespace RegisterValue {
/*!
 * \name RegisterValue
 * \brief Predefined values for the register (whole register of field).
 */
enum RegisterValue : uint8_t {
  kStatusIRQClear = RegisterFieldMask::kStatusRxDataReady |
                    RegisterFieldMask::kStatusTxDataSent |
                    RegisterFieldMask::kStatusMaxRetransmits
};
}  // namespace RegisterValue

// Enumeration of RX pipe addresses and TX address
namespace Pipe {
enum Pipe : uint8_t {
  kRx0 = 0x00,
  kRx1 = 0x01,
  kRx2 = 0x02,
  kRx3 = 0x03,
  kRx4 = 0x04,
  kRx5 = 0x05,
  kTx  = 0x06,
};
}

// Enumeration of pipe flags
namespace PipeFlag {
enum PipeFlag : uint8_t {
  kRx0   = bit::maskBits<Pipe::kRx0>(),
  kRx1   = bit::maskBits<Pipe::kRx1>(),
  kRx2   = bit::maskBits<Pipe::kRx2>(),
  kRx3   = bit::maskBits<Pipe::kRx3>(),
  kRx4   = bit::maskBits<Pipe::kRx4>(),
  kRx5   = bit::maskBits<Pipe::kRx5>(),
  kTx    = bit::maskBits<Pipe::kTx>(),
  kRxAll = bit::maskRange<Pipe::kRx0, Pipe::kRx5>()
};
}

// Addresses of the address registers
/*
static const uint8_t kPipeAddress[7] = {Register ::kRxAddressP0, Register ::kRxAddressP1,
                                        Register ::kRxAddressP2, Register ::kRxAddressP3,
                                        Register ::kRxAddressP4, Register ::kRxAddressP5,
                                        Register ::kTxAddress};*/

enum class AutoRetransmitDelay : uint8_t {
  kNone   = 0x00,
  k250us  = 0x00,
  k500us  = 0x01,
  k750us  = 0x02,
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

namespace DataRate {
using namespace blt::bit;
enum DataRate : uint8_t {
  k250kbps = maskBits<5>(),
  k1Mbps   = 0x00,
  k2Mbps   = maskBits<3>()
};
}  // namespace DataRate

enum class PowerAmplifier : uint8_t {
  kMinus_18_dB = 0b00,
  kMinus_12_dB = 0b01,
  kMinus_6_dB  = 0b10,
  kZero_dB     = 0b11,
  kMinimum     = kMinus_18_dB
};

/**
 * \brief nRF24L01 driver class.
 *
 * \tparam spi The spi peripheral driver.
 * \tparam csn The chip select negative (csn) gpio pin.
 * \tparam ce  The chip enable gpio pin.
 */
template <typename spi, typename csn, typename ce>
class device {
  // cs is csn inverted, with a 100 us settling time
  using cs = gpio::settle<gpio::invert<csn>, 100>;

 public:
  bool init();
  bool test();

  void setAutoAck(bool enable = true);
  void setAutoAck(bool enable, uint8_t pipe);
  bool setDataRate(uint8_t dataRate);
  void setDynamicPayload(bool enable = true);
  void setAutoRetransmit(AutoRetransmitDelay ard = AutoRetransmitDelay::kNone,
                         uint8_t             arc = 0);
  void setPayloadSize(uint8_t size);
  void setPALevel(PowerAmplifier level);

  void enableAckPayload();

  void openReadingPipe(uint8_t pipe, uint64_t address);
  void openWritingPipe(uint64_t address);
  void openWritingPipe(const uint8_t* address);

  void startListening();
  void stopListening();

  bool available();
  bool available(uint8_t& pipe);

  void powerUp();
  void powerDown();

  bool write(const uint8_t* buf, uint8_t len, const bool multicast);

 private:
  /**
   * Helper function to write then read data from the spi device.
   */
  uint8_t inline spirw(uint8_t data);
  uint8_t spiTransfer(uint8_t cmd);

  static constexpr inline void begin();
  static constexpr inline void end();

  /// Register read access
  uint8_t readRegister(uint8_t reg);
  uint8_t readRegister(uint8_t reg, uint8_t* buf, uint8_t len);

  /// Register write access
  uint8_t writeRegister(uint8_t reg, uint8_t val);
  uint8_t writeRegister(uint8_t reg, const uint8_t* buf, uint8_t len);

  uint8_t getStatus();

  uint8_t flushTx();
  uint8_t flushRx();

  void clearIRQFlags();

  void    setChannel(uint8_t channel);
  uint8_t getChannel();

  uint8_t getAddressWidth();
  void    setAddressWidth(uint8_t width);

  // void setAddress(uint8_t pipe, const uint8_t* addr);

  void toggleFeatures();

  /// Commands
  uint8_t readPayload(uint8_t* buf, uint8_t len);
  uint8_t writePayload(const uint8_t* buf, uint8_t len, const uint8_t type);

  void startWriteFast(const uint8_t* buf,
                      uint8_t        len,
                      const bool     multicast,
                      bool           startTx = true);

 private:
  bool m_pVariant = false;

  uint8_t m_addressWidth   = 5;
  uint8_t m_P0RxAddress[5] = {0};

  uint8_t m_payloadSize     = 32;
  bool    m_dynamicPayloads = false;
};

}  // namespace nrf24

}  // namespace blt

#include <nrf24_custom/nrf24_impl.hpp>

#endif  // BLT_LIBS_NRF24_H_
