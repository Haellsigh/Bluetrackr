/**
 * \file nrf24.hh
 */
#pragma once

#include <cstdint>

#include <blt/bit.hh>
#include <blt/gpio.hh>
#include <blt/memory.hh>

namespace nrf24 {

enum class Command : uint8_t {
  kNone                = 0b00000000,
  kReadRegister        = 0b000 << 5,
  kWriteRegister       = 0b001 << 5,
  kReadRxPayload       = 0b01100001,
  kWriteTxPayload      = 0b10100000,
  kWriteTxPayloadNoAck = 0b10110000,
  kFlushRx             = 0b11100010,
  kFlushTx             = 0b11100001,
  kReuseTxPayload      = 0b11100011,
  kNop                 = 0b11111111
};

inline constexpr uint8_t kReadWriteCommandMask = 0b00011111;

namespace Register {
using namespace blt;
using Config                  = memory::Register<0x00>;
using EnableAutoAck           = memory::Register<0x01>;
using EnabledRxAddresses      = memory::Register<0x02>;
using SetupAddressWidths      = memory::Register<0x03>;
using SetupAutoRetransmission = memory::Register<0x04>;
using RfChannel               = memory::Register<0x05>;
using RfSetup                 = memory::Register<0x06>;
using Status                  = memory::Register<0x07>;
using TxObserve               = memory::Register<0x08>;
using RxPowerDetector         = memory::Register<0x09>;
using RxAddressP0             = memory::Register<0x0A>;
using RxAddressP1             = memory::Register<0x0B>;
using RxAddressP2             = memory::Register<0x0C>;
using RxAddressP3             = memory::Register<0x0D>;
using RxAddressP4             = memory::Register<0x0E>;
using RxAddressP5             = memory::Register<0x0F>;
using TxAddress               = memory::Register<0x10>;
using RxPayloadWidthP0        = memory::Register<0x11>;
using RxPayloadWidthP1        = memory::Register<0x12>;
using RxPayloadWidthP2        = memory::Register<0x13>;
using RxPayloadWidthP3        = memory::Register<0x14>;
using RxPayloadWidthP4        = memory::Register<0x15>;
using RxPayloadWidthP5        = memory::Register<0x16>;
using FifoStatus              = memory::Register<0x17>;
using DynamicPayload          = memory::Register<0x1C>;
using Feature                 = memory::Register<0x1D>;
}  // namespace Register

namespace Field {
using namespace blt::memory;
using namespace blt::memory::access_policy;
// Config
using MaskRxDataReady   = Register::Config::Field<bits<6>>;
using MaskTxDataSent    = Register::Config::Field<bits<5>>;
using MaskMaxRetransmit = Register::Config::Field<bits<4>>;
using EnableCrc         = Register::Config::Field<bits<3>>;
using CrcEncoding       = Register::Config::Field<bits<2>>;
using PowerUp           = Register::Config::Field<bits<1>>;
using Primary           = Register::Config::Field<bits<0>>;
// EnableAutoAck
using AutoAckP5    = Register::EnableAutoAck::Field<bits<5>>;
using AutoAckP4    = Register::EnableAutoAck::Field<bits<4>>;
using AutoAckP3    = Register::EnableAutoAck::Field<bits<3>>;
using AutoAckP2    = Register::EnableAutoAck::Field<bits<2>>;
using AutoAckP1    = Register::EnableAutoAck::Field<bits<1>>;
using AutoAckP0    = Register::EnableAutoAck::Field<bits<0>>;
using AutoAckPipes = Register::EnableAutoAck::Field<range<0, 5>>;
// EnabledRxAddresses
using RxP5    = Register::EnabledRxAddresses::Field<bits<5>>;
using RxP4    = Register::EnabledRxAddresses::Field<bits<4>>;
using RxP3    = Register::EnabledRxAddresses::Field<bits<3>>;
using RxP2    = Register::EnabledRxAddresses::Field<bits<2>>;
using RxP1    = Register::EnabledRxAddresses::Field<bits<1>>;
using RxP0    = Register::EnabledRxAddresses::Field<bits<0>>;
using RxPipes = Register::EnabledRxAddresses::Field<range<0, 5>>;
// SetupAddressWidths
using AddressWidths = Register::SetupAddressWidths::Field<range<0, 1>>;
// SetupAutoRet
using SetupAutoRetransmitDelay = Register::SetupAutoRetransmission::Field<range<4, 7>>;
using SetupAutoRetransmitCount = Register::SetupAutoRetransmission::Field<range<0, 3>>;
// RfChannel
using RfChannelFreq = Register::RfChannel::Field<range<0, 6>>;
// RfSetup
using ContinuousWave = Register::RfSetup::Field<bits<7>>;
using PLLLock        = Register::RfSetup::Field<bits<4>>;
using RfDatarate     = Register::RfSetup::Field<bits<5, 3>>;
using RfPower        = Register::RfSetup::Field<range<1, 2>>;
using RfLnaGain      = Register::RfSetup::Field<bits<0>>;
// Status
using RxFifoDataReady   = Register::Status::Field<bits<6>>;
using TxFifoDataSent    = Register::Status::Field<bits<5>>;
using MaxRetransmit     = Register::Status::Field<bits<4>>;
using RxFifoPayloadPipe = Register::Status::Field<range<1, 3>, read_only>;
using TxFull            = Register::Status::Field<bits<0>, read_only>;
// TxObserve
using PacketLossCount     = Register::TxObserve::Field<range<4, 7>, read_only>;
using AutoRetransmitCount = Register::TxObserve::Field<range<0, 3>, read_only>;
// RxPowerDetector
using RxPowerDetected = Register::RxPowerDetector::Field<bits<0>, read_only>;
// RxAddressP0 \todo 39 bits
// RxAddressP1 \todo 39 bits
// RxAddressP2 \todo 7 bits
// RxAddressP3 \todo 7 bits
// RxAddressP4 \todo 7 bits
// RxAddressP5 \todo 7 bits
// TxAddress   \todo 39 bits
// RxPayloadWidthP0
using RxPayloadWidthP0 = Register::RxPayloadWidthP0::Field<range<0, 5>>;
// RxPayloadWidthP1
using RxPayloadWidthP1 = Register::RxPayloadWidthP1::Field<range<0, 5>>;
// RxPayloadWidthP2
using RxPayloadWidthP2 = Register::RxPayloadWidthP2::Field<range<0, 5>>;
// RxPayloadWidthP3
using RxPayloadWidthP3 = Register::RxPayloadWidthP3::Field<range<0, 5>>;
// RxPayloadWidthP4
using RxPayloadWidthP4 = Register::RxPayloadWidthP4::Field<range<0, 5>>;
// RxPayloadWidthP5
using RxPayloadWidthP5 = Register::RxPayloadWidthP5::Field<range<0, 5>>;
// FifoStatus
using UsingTxReuse = Register::FifoStatus::Field<bits<6>, read_only>;
using TxFifoFull   = Register::FifoStatus::Field<bits<5>, read_only>;
using TxFifoEmpty  = Register::FifoStatus::Field<bits<4>, read_only>;
using RxFifoFull   = Register::FifoStatus::Field<bits<1>, read_only>;
using RxFifoEmpty  = Register::FifoStatus::Field<bits<0>, read_only>;
// DynamicPayload
using DynamicPayloadP5    = Register::DynamicPayload::Field<bits<5>>;
using DynamicPayloadP4    = Register::DynamicPayload::Field<bits<4>>;
using DynamicPayloadP3    = Register::DynamicPayload::Field<bits<3>>;
using DynamicPayloadP2    = Register::DynamicPayload::Field<bits<2>>;
using DynamicPayloadP1    = Register::DynamicPayload::Field<bits<1>>;
using DynamicPayloadP0    = Register::DynamicPayload::Field<bits<0>>;
using DynamicPayloadPipes = Register::DynamicPayload::Field<range<0, 5>>;
// Feature
using DynamicPayloadEnabled = Register::Feature::Field<bits<2>>;
using AckPayloadEnabled     = Register::Feature::Field<bits<1>>;
using DynamicAckEnabled     = Register::Feature::Field<bits<0>>;
}  // namespace Field

namespace Value {
// MaskRxDataReady
using EnableIntRxDataReady  = Field::MaskRxDataReady::value<0>;
using DisableIntRxDataReady = Field::MaskRxDataReady::value<1>;
// MaskTxDataSent
using EnableIntTxDataSent  = Field::MaskTxDataSent::value<0>;
using DisableIntTxDataSent = Field::MaskTxDataSent::value<1>;
// MaskMaxRetransmit
using EnableIntMaxRetransmit  = Field::MaskMaxRetransmit::value<0>;
using DisableIntMaxRetransmit = Field::MaskMaxRetransmit::value<1>;
// CrcEncoding
using Crc1byte  = Field::CrcEncoding::value<0>;
using Crc2bytes = Field::CrcEncoding::value<1>;
// Power
using PowerUp   = Field::PowerUp::value<1>;
using PowerDown = Field::PowerUp::value<0>;
// Primary
using PrimaryRx = Field::Primary::value<1>;
using PrimaryTx = Field::Primary::value<0>;
// AutoAckP5
// AutoAckP4
// AutoAckP3
// AutoAckP2
// AutoAckP1
// AutoAckP0
// AutoAckPipes
using EnableAutoAckAllPipes  = Field::AutoAckPipes::value<0b111111>;
using DisableAutoAckAllPipes = Field::AutoAckPipes::value<0b111111>;
// RxP5
// RxP4
// RxP3
// RxP2
// RxP1
// RxP0
// RxPipes
// AddressWidths
using AddressWidth3byte = Field::AddressWidths::value<0, 1>;
using AddressWidth4byte = Field::AddressWidths::value<1, 0>;
using AddressWidth5byte = Field::AddressWidths::value<1, 1>;
// SetupAutoRetransmitDelay
using AutoRetransmitDelay250us  = Field::SetupAutoRetransmitDelay::value<0>;
using AutoRetransmitDelay500us  = Field::SetupAutoRetransmitDelay::value<1>;
using AutoRetransmitDelay750us  = Field::SetupAutoRetransmitDelay::value<2>;
using AutoRetransmitDelay1000us = Field::SetupAutoRetransmitDelay::value<3>;
using AutoRetransmitDelay1250us = Field::SetupAutoRetransmitDelay::value<4>;
using AutoRetransmitDelay1500us = Field::SetupAutoRetransmitDelay::value<5>;
using AutoRetransmitDelay1750us = Field::SetupAutoRetransmitDelay::value<6>;
using AutoRetransmitDelay2000us = Field::SetupAutoRetransmitDelay::value<7>;
using AutoRetransmitDelay2250us = Field::SetupAutoRetransmitDelay::value<8>;
using AutoRetransmitDelay2500us = Field::SetupAutoRetransmitDelay::value<9>;
using AutoRetransmitDelay3000us = Field::SetupAutoRetransmitDelay::value<10>;
using AutoRetransmitDelay3250us = Field::SetupAutoRetransmitDelay::value<11>;
using AutoRetransmitDelay2750us = Field::SetupAutoRetransmitDelay::value<12>;
using AutoRetransmitDelay3500us = Field::SetupAutoRetransmitDelay::value<13>;
using AutoRetransmitDelay3750us = Field::SetupAutoRetransmitDelay::value<14>;
using AutoRetransmitDelay4000us = Field::SetupAutoRetransmitDelay::value<15>;
// SetupAutoRetransmitCount
using AutoRetransmitCountDisabled = Field::SetupAutoRetransmitCount::value<0>;
// RfChannelFreq
// ContinuousWave
using EnableContinuousWave  = Field::ContinuousWave::value<1>;
using DisableContinuousWave = Field::ContinuousWave::value<0>;
// PLLLock
using EnablePLLLock  = Field::ContinuousWave::value<1>;
using DisablePLLLock = Field::ContinuousWave::value<0>;
// RfDatarate
using RfDatarate250kbps = Field::RfDatarate::value<1, 0>;
using RfDatarate1Mbps   = Field::RfDatarate::value<0, 0>;
using RfDatarate2Mbps   = Field::RfDatarate::value<0, 1>;
// RfPower
using RfPowerMinus18dBm = Field::RfPower::value<0b00>;
using RfPowerMinimum    = RfPowerMinus18dBm;
using RfPowerMinus12dBm = Field::RfPower::value<0b01>;
using RfPowerMinus16dBm = Field::RfPower::value<0b10>;
using RfPower0dBm       = Field::RfPower::value<0b11>;
// RxFifoDataReady
using ClearRxDataReady = Field::RxFifoDataReady::value<1>;
// TxFifoDataSent
using ClearTxDataSent = Field::TxFifoDataSent::value<1>;
// MaxRetransmit
using ClearMaxRetransmit = Field::MaxRetransmit::value<1>;
// RxFifoPayloadPipe
using RxFifoEmpty = Field::RxFifoPayloadPipe::value<6>;
using RxFifoP0    = Field::RxFifoPayloadPipe::value<0>;
using RxFifoP1    = Field::RxFifoPayloadPipe::value<1>;
using RxFifoP2    = Field::RxFifoPayloadPipe::value<2>;
using RxFifoP3    = Field::RxFifoPayloadPipe::value<3>;
using RxFifoP4    = Field::RxFifoPayloadPipe::value<4>;
using RxFifoP5    = Field::RxFifoPayloadPipe::value<5>;
// TxFull
using TxFifoFull = Field::TxFull::value<1>;
// PacketLossCount
// AutoRetransmitCount
// ReceivedPowerDetected
// RxPayloadWidthP0
// RxPayloadWidthP1
// RxPayloadWidthP2
// RxPayloadWidthP3
// RxPayloadWidthP4
// RxPayloadWidthP5
// UsingTxReuse
// TxFifoFull
// TxFifoEmpty
// RxFifoFull
// RxFifoEmpty
// DynamicPayloadP5
// DynamicPayloadP4
// DynamicPayloadP3
// DynamicPayloadP2
// DynamicPayloadP1
// DynamicPayloadP0
// DynamicPayloadPipes
using EnableDynamicPayloadAllPipes = Field::DynamicPayloadPipes::value<0b111111>;
// DynamicPayloadEnabled
// AckPayloadEnabled
// DynamicAckEnabled
}  // namespace Value

/**
 * \brief nRF24L01 driver class.
 *
 * \tparam spi The spi peripheral driver.
 * \tparam csn The chip select negative (csn) gpio pin.
 * \tparam ce  The chip enable gpio pin.
 */
/*requires requires
{
  spi::rw(0);
}*/
template <typename spi, typename csn, typename ce>
class device : public blt::utils::noncopyable {
  // cs is csn inverted, with a 100 us settling time
  // using cs = gpio::settle<gpio::invert<csn>, 100>;
  using cs = blt::gpio::invert<csn>;

 public:
  bool init();
  bool test();

  void setAutoAck(bool enable = true);
  void setAutoAck(bool enable, uint8_t pipe);
  template <typename DataRate = Value::RfDatarate1Mbps>
  bool setDataRate();
  void setDynamicPayload(bool enable = true);

  template <typename AutoRetransmitDelay = Value::AutoRetransmitDelay250us,
            uint8_t AutoRetransmitCount  = 0>
  void setupAutoRetransmit();
  void setPayloadSize(uint8_t size);
  template <typename RfPower>
  void setRfPowerLevel();

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
  uint8_t spiCommand(Command cmd);

  static constexpr inline void begin();
  static constexpr inline void end();

  /// Register read access
  template <typename register_t>
  register_t readRegister();
  template <typename register_t>
  Register::Status readRegister(uint8_t* buf, uint8_t len);

  /// Register write access
  template <typename register_t>
  inline Register::Status writeRegister(register_t reg);
  template <typename register_t>
  inline Register::Status writeRegisterOffset(register_t reg, uint8_t address_offset);
  template <typename register_t>
  Register::Status writeRegister(const uint8_t* buf,
                                 uint8_t        len,
                                 uint8_t        address_offset = 0);

  // Spi commands
  Register::Status getStatus();
  uint8_t          flushTx();
  uint8_t          flushRx();

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
  bool mPVariant = false;

  uint8_t mAddressWidth   = 5;
  uint8_t mP0RxAddress[5] = {0};

  uint8_t mPayloadSize    = 32;
  bool    mDynamicPayload = false;
};

}  // namespace nrf24

#include "nrf24_impl.hpp"
