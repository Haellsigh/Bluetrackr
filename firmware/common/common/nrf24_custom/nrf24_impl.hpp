/**
 * \file nrf24_impl.hpp
 */
#include <blt/utils.hh>
#include <nrf24_custom/nrf24.hh>

#include <algorithm>  // min/max
#include <array>
#include <cstring>  // memcpy

namespace nrf24 {

/**
 * \enum Command
 * \brief Available SPI commands
 *
 * \note The command *bits* are sent from msbit to lsbit.
 * \note The data *bytes* are sent from lsbyte to msbyte, then msbit to lsbit.
 *
 * \note The device ouputs the Status register on MISO while writing a command on MOSI.
 *
 * \var ReadRegister
 * \brief Read a register
 * \note Last 5 bits are the register address
 *
 * \var WriteRegister
 * \brief Write a register.
 * \note Last 5 bits are the register address
 * \note Executable in Power down or standby modes only
 *
 * \var ReadRxPayload
 * \brief Read receive payload (1-32 bytes)
 * \note Payload is deleted from FIFO after it is read
 *
 * \var WriteTxPayload
 * \brief Write transmit payload (1-32 bytes)
 *
 * \var FlushRx
 * \brief Flush receive FIFO, used in receive mode
 * \note Should not be executed during transmission of acknowledge, that is,
 *       acknowledge package will not be completed.
 *
 * \var FlushTx
 * \brief Flush transmit FIFO, used in transmit mode
 *
 * \var ReuseTxPayload
 * \brief Reuse last transmitted payload
 * \note Used for a PTX device
 * \note Transmit payload reuse is active until WriteTxPayload or FlushTx is
 * executed.
 * \note Transmit payload reuse must not be activated or deactivated during
 * package transmission.
 *
 * \var Nop
 * \brief No operation
 * \note Can be used to read the Register::kStatus register
 */

/**
 * \var ReadWriteCommandMask
 * \brief Bit-mask for the ReadRegister and WriteRegister commands
 */

/**
 * \brief Register table of the device
 */
namespace Register {
/**
 * \var Config
 * \brief Configuration register
 *
 * \var EnableAutoAck
 * \brief 'Auto Acknowledgment' feature setup register
 * \note Only available on p-variant.
 *       Disable this feature to be compatible with nRF24L01.
 *
 * \var EnabledRxAddresses
 * \brief Enabled RX pipes.
 *
 * \var SetupAddressWidths
 * \brief Address widths setup register
 * \note Address width is common for all pipes.
 *
 * \var SetupAutoRet
 * \brief Automatic retransmission setup register
 *
 * \var RfChannel
 * \brief RF Channel setup register
 *
 * \var RfSetup
 * \brief RF setup register
 *
 * \var Status
 * \brief Status register
 *
 * \var TxObserve
 * \brief Transmit observe register
 *
 * \var RxPowerDetector
 * \brief Received power detector register.
 * \note This register is called CD (Carrier Detect) on non p-variant because they have
 *       different thresholds.
 *
 * \var RxAddressP0
 * \brief Receive address pipe 0 register.
 *
 * \var RxAddressP1
 * \brief Receive address pipe 1 register.
 *
 * \var RxAddressP2
 * \brief Receive address pipe 2 register.
 *
 * \var RxAddressP3
 * \brief Receive address pipe 3 register.
 *
 * \var RxAddressP4
 * \brief Receive address pipe 4 register.
 *
 * \var RxAddressP5
 * \brief Receive address pipe 5 register.
 *
 * \var TxAddress
 * \brief Transmit address register.
 * \note Used for a PTX device only.
 * \note Set RxAddressP0 equal to this address to handle automatic acknowledge
 * if this is a PTX device with Enhanced ShockBurst enabled.
 *
 * \var RxPayloadWidthP0
 * \brief Number of bytes of receive payload in pipe 0.
 *
 * \var RxPayloadWidthP1
 * \brief Number of bytes of receive payload in pipe 1.
 *
 * \var RxPayloadWidthP2
 * \brief Number of bytes of receive payload in pipe 2.
 *
 * \var RxPayloadWidthP3
 * \brief Number of bytes of receive payload in pipe 3.
 *
 * \var RxPayloadWidthP4
 * \brief Number of bytes of receive payload in pipe 4.
 *
 * \var RxPayloadWidthP5
 * \brief Number of bytes of receive payload in pipe 5.
 *
 * \var FifoStatus
 * \brief FIFO status register.
 *
 * \var DynamicPayload
 * \brief Dynamic payload setup register.
 *
 * \var Feature
 * \brief Feature (dynamic payload, payload in ack, no ack on packet) register.
 * \note The 'no ack on packet' function is not implemented.
 */
}  // namespace Register

namespace RegisterField {
/**
 * \var MaskRxDataReady
 * \brief
 *
 * \var PacketLossCount
 * \brief Packet loss counter.
 * \note Counts up to 15 then stops.
 * \note Can be reset by writing to RfChannelFreq.
 *
 * \todo Finish this part of the documentation
 */
}

static constexpr uint8_t kMaxChannel     = 125;
static constexpr uint8_t kMaxPayloadSize = 32;

template <typename spi, typename csn, typename ce>
uint8_t inline device<spi, csn, ce>::spirw(uint8_t data)
{
  return spi::rw(data);
}

template <typename spi, typename csn, typename ce>
constexpr inline void device<spi, csn, ce>::begin()
{
  cs::set();
}

template <typename spi, typename csn, typename ce>
constexpr inline void device<spi, csn, ce>::end()
{
  cs::clear();
}

template <typename spi, typename csn, typename ce>
uint8_t device<spi, csn, ce>::spiCommand(Command cmd)
{
  uint8_t status;
  begin();

  // Execute command
  status = spirw(static_cast<uint8_t>(cmd));

  end();
  return status;
}

/**
 * \brief
 * \param reg The register to read.
 * \return The register value.
 */
template <typename spi, typename csn, typename ce>
template <typename register_t>
register_t device<spi, csn, ce>::readRegister()
{
  begin();

  // Register read access
  /*const uint8/_t status = */ spirw(
      static_cast<uint8_t>(Command::kReadRegister) |
      (kReadWriteCommandMask & register_t::register_address));
  const uint8_t reg = spirw(static_cast<uint8_t>(Command::kNop));

  end();
  return register_t{reg};
}

/**
 * \param reg The register to read.
 * \param buf The register value(s).
 * \param len The length of buf.
 * \return The status of the operation.
 */
template <typename spi, typename csn, typename ce>
template <typename register_t>
Register::Status device<spi, csn, ce>::readRegister(uint8_t* buf, uint8_t len)
{
  begin();

  // Register read access
  const uint8_t status = spirw(static_cast<uint8_t>(Command::kReadRegister) |
                               (kReadWriteCommandMask & register_t::register_address));
  while (len--) {
    *buf++ = spirw(static_cast<uint8_t>(Command::kNop));
  }

  end();
  return Register::Status{status};
}

template <typename spi, typename csn, typename ce>
template <typename register_t>
Register::Status device<spi, csn, ce>::writeRegister(register_t reg)
{
  begin();

  const uint8_t status =
      spirw(static_cast<uint8_t>(Command::kWriteRegister) | register_t::register_address);

  spirw(reg.value());

  end();
  return Register::Status{status};
}

template <typename spi, typename csn, typename ce>
template <typename register_t>
Register::Status device<spi, csn, ce>::writeRegisterOffset(register_t reg,
                                                           uint8_t    address_offset)
{
  begin();

  const uint8_t status = spirw(static_cast<uint8_t>(Command::kWriteRegister) |
                               (register_t::register_address + address_offset));

  spirw(reg.value());

  end();
  return Register::Status{status};
}

/**
 *
 * @param reg
 * @param buf
 * @param len
 * @return The status of the operation.
 */
template <typename spi, typename csn, typename ce>
template <typename register_t>
Register::Status device<spi, csn, ce>::writeRegister(const uint8_t* buf,
                                                     uint8_t        len,
                                                     uint8_t        address_offset)
{
  begin();

  const uint8_t status = spirw(static_cast<uint8_t>(Command::kWriteRegister) |
                               (register_t::register_address + address_offset));
  while (len--) {
    spirw(*buf++);
  }

  end();
  return Register::Status{status};
}

template <typename spi, typename csn, typename ce>
Register::Status device<spi, csn, ce>::getStatus()
{
  return Register::Status{spiCommand(Command::kNop)};
}

template <typename spi, typename csn, typename ce>
uint8_t device<spi, csn, ce>::flushTx()
{
  return spiCommand(Command::kFlushTx);
}

template <typename spi, typename csn, typename ce>
uint8_t device<spi, csn, ce>::flushRx()
{
  return spiCommand(Command::kFlushRx);
}

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::clearIRQFlags()
{
  using namespace Value;
  Register::Status status = readRegister<Register::Status>();

  status.write<ClearRxDataReady>();
  status.write<ClearTxDataSent>();
  status.write<ClearMaxRetransmit>();

  writeRegister(status);
}

/**
 * \brief Sets the radio channel, between 0 and 125 included.
 */
template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::setChannel(uint8_t channel)
{
  using namespace Field;
  Register::RfChannel rfchannel;

  rfchannel.field<RfChannelFreq>() = std::min(channel, kMaxChannel);

  writeRegister(rfchannel);
}

template <typename spi, typename csn, typename ce>
uint8_t device<spi, csn, ce>::getChannel()
{
  Register::RfChannel rfchannel = readRegister<Register::RfChannel>();
  return rfchannel.field<Field::RfChannelFreq>();
}

template <typename spi, typename csn, typename ce>
uint8_t device<spi, csn, ce>::getAddressWidth()
{
  using namespace blt::utils::literals;
  Register::SetupAddressWidths aw = readRegister();
  return static_cast<uint8_t>(aw.field<Field::AddressWidths>()) + 2_u8;
}

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::setAddressWidth(uint8_t width)
{
  width = std::clamp<uint8_t>(width, 3, 5);
  Register::SetupAddressWidths aw;
  aw.field<Field::AddressWidths>() = width - 2;

  writeRegister(aw);
}

/*
template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::setAddress(uint8_t pipe, const uint8_t* addr) {
  uint8_t addr_width;

  switch (pipe) {
    case Pipe::kTx:
    case Pipe::kRx0:
    case Pipe::kRx1:
      addr_width = getAddressWidth();
      writeRegister(Command::kWriteRegister | nRF24_ADDR_REGS[pipe], (uint8_t*)addr,
                    addr_width);
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
}*/

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::toggleFeatures()
{
  // From the nRF24L01 Rev2.0 datasheet, not present in nRF24L01+.
  // This does nothing on the nRF24L01+.
  begin();
  spirw(0x50);  // Activate
  spirw(0x73);  // ??
  end();
}

template <typename spi, typename csn, typename ce>
uint8_t device<spi, csn, ce>::readPayload(uint8_t* buf, uint8_t len)
{
  uint8_t status;

  len               = std::min(len, mPayloadSize);
  uint8_t blank_len = mDynamicPayload ? 0 : mPayloadSize - len;

  begin();

  status = spirw(Command::kReadRxPayload);
  while (len--) {
    *buf++ = spirw(Command::kNop);
  }
  while (blank_len--) {
    spirw(Command::kNop);
  }

  end();
  return status;
}

template <typename spi, typename csn, typename ce>
uint8_t device<spi, csn, ce>::writePayload(const uint8_t* buf,
                                           uint8_t        len,
                                           const uint8_t  type)
{
  uint8_t status;

  len               = std::min(len, mPayloadSize);
  uint8_t blank_len = mDynamicPayload ? 0 : mPayloadSize - len;

  begin();

  status = spirw(type);
  while (len--) {
    spirw(*buf++);
  }
  while (blank_len--) {
    spirw(Command::kNone);
  }

  end();

  return status;
}

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::startWriteFast(const uint8_t* buf,
                                          uint8_t        len,
                                          const bool     multicast,
                                          bool           startTx)
{
  writePayload(buf, len,
               multicast ? Command::kWriteTxPayloadNoAck : Command::kWriteTxPayload);
  if (startTx)
    ce::set();
}

/// Public functions

/**
 * Initializes the radio with default parameters
 */
template <typename spi, typename csn, typename ce>
bool device<spi, csn, ce>::init()
{
  using namespace blt;
  using namespace blt::time::literals;

  uint8_t setup = 0;
  ce::clear();
  cs::clear();

  // Section 5.7: Power on Reset
  time::delay(100_ms);

  // Power up delay
  time::delay(5_ms);

  /// Arbitrary default values for the registers
  // Reset config & enable 16-bit CRC
  auto config                      = Register::Config{};
  config.field<Field::EnableCrc>() = true;
  config.write<Value::Crc2bytes>();
  writeRegister(config);

  // The minimum viable delay after testing on all speeds
  setupAutoRetransmit<Value::AutoRetransmitDelay1500us, 15>();

  // This is a p variant if 250 kbps is supported
  if (setDataRate<Value::RfDatarate250kbps>()) {
    mPVariant = true;
  }

  auto rfsetup = readRegister<Register::RfSetup>();

  // Slowest common supported speed on both non-p and p variants
  setDataRate<Value::RfDatarate1Mbps>();

  // Disable dynamic payloads
  toggleFeatures();
  setDynamicPayload(false);
  mDynamicPayload = false;

  // Reset status (clears IRQ)
  clearIRQFlags();
  // writeRegister(Register::kStatus, RegisterMask::kStatusIRQ);

  // Setup default channel configuration
  // Channel 76 should be mostly interference free
  setChannel(76);

  flushRx();
  flushTx();

  powerUp();

  // Enable PTX & keep CE low so the radio will remain in standby I mode
  // 22µa consumption
  config = readRegister<Register::Config>();
  config.write<Value::PrimaryTx>();
  writeRegister(config);

  return (rfsetup.value() != 0x00) && (rfsetup.value() != 0xFF);
}

template <typename spi, typename csn, typename ce>
bool device<spi, csn, ce>::test()
{
  constexpr uint8_t            bufsize = 5;
  std::array<uint8_t, bufsize> rxbuf;
  std::array<uint8_t, bufsize> txbuf = {'n', 'R', 'F', '2', '4'};

  // Write the test address to the TxAddr register
  writeRegister<Register::TxAddress>(txbuf.data(), bufsize);

  // Read it back
  readRegister<Register::TxAddress>(rxbuf.data(), bufsize);

  // Compare transmitted and received data
  for (uint8_t i = 0; i < bufsize; i++) {
    if (rxbuf[i] != txbuf[i])
      // Transmitter is absent or there is a problem.
      return false;
  }

  // Transmitter is present & working.
  return true;
}

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::setAutoAck(bool enable)
{
  Register::EnableAutoAck en_aa;
  if (enable) {
    en_aa.write<Value::EnableAutoAckAllPipes>();
  } else {
    en_aa.write<Value::DisableAutoAckAllPipes>();
  }
  writeRegister(en_aa);
}

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::setAutoAck(bool enable, uint8_t pipe)
{
  if (pipe <= 5) {
    Register::EnableAutoAck en_aa = readRegister();

    if (enable)
      en_aa.setValue(en_aa.value() | (1U << pipe));
    else
      en_aa.setValue(en_aa.value() & ~(1U << pipe));

    writeRegister(en_aa);
  }
}

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::enableAckPayload()
{
  // Enable ack payload (& dynamic payload)
  Register::Feature feature                     = readRegister();
  feature.field<Field::AckPayloadEnabled>()     = true;
  feature.field<Field::DynamicPayloadEnabled>() = true;
  writeRegister(feature);

  // Enable dynamic payload on all RX pipes
  Register::DynamicPayload dynpd = readRegister();
  dynpd.write<Value::EnableDynamicPayloadAllPipes>();
  writeRegister(dynpd);

  /*
  // Original code, this is weird because it says 'dynpd on only certain pipes is
  not really used', but then it does this:
  // Enable dynamic payload on pipes 0 & 1
  uint8_t dynpd = readRegister(Register::kDynamicPayload);
  dynpd |= bit::maskBits<Pipe::kRx0, Pipe::kRx1>();
  writeRegister(Register::kDynamicPayload, dynpd);
  */

  mDynamicPayload = true;
}

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::setDynamicPayload(bool enable)
{
  // Enable dynamic payload
  if (enable) {
    // Enable dynamic payload feature
    Register::Feature feature                     = readRegister<Register::Feature>();
    feature.field<Field::DynamicPayloadEnabled>() = true;
    writeRegister(feature);

    // Enable dynamic payload on all RX pipes
    Register::DynamicPayload dynpd = readRegister<Register::DynamicPayload>();
    dynpd.write<Value::EnableDynamicPayloadAllPipes>();
    writeRegister(dynpd);

    mDynamicPayload = true;
  }
  // Disable dynamic payload
  else {
    // Disable dynamic payload feature
    writeRegister(Register::Feature{});

    // Disable dynamic payload on all pipes
    writeRegister(Register::DynamicPayload{});

    mDynamicPayload = false;
  }
}

/**
 * \brief Sets the automatic retransmission parameters.
 * @param ard The auto-retransmit delay.
 * @param arc The auto-retransmit count.
 */
template <typename spi, typename csn, typename ce>
template <typename AutoRetransmitDelay, uint8_t AutoRetransmitCount>
void device<spi, csn, ce>::setupAutoRetransmit()
{
  Register::SetupAutoRetransmission setup;

  setup.write<AutoRetransmitDelay>();
  setup.field<Field::SetupAutoRetransmitCount>() = AutoRetransmitCount;

  writeRegister(setup);
}

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::setPayloadSize(uint8_t size)
{
  mPayloadSize = std::min(size, kMaxPayloadSize);
  // Payload size is set when opening a pipe
}

/*!
 * \brief Sets the power amplifier gain level.
 */
template <typename spi, typename csn, typename ce>
template <typename RfPower>
void device<spi, csn, ce>::setRfPowerLevel()
{
  // Get current RF configuration
  Register::RfSetup rfsetup = readRegister<Register::RfSetup>();
  // Write power level
  rfsetup.write<RfPower>();
  if (!mPVariant)
    rfsetup.field<Field::RfLnaGain>() = true;  // Sets the LNA gain for non p-variants

  writeRegister(rfsetup);
}

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::openReadingPipe(uint8_t pipe, uint64_t address)
{
  // Special case: cache the address for startListening().
  if (pipe == 0) {
    std::memcpy(mP0RxAddress, &address, mAddressWidth);
    return;
  }
  // Write the address width (1 for pipe 2-5 or m_addressWidth for pipe 0-1).
  writeRegister<Register::RxAddressP0>(reinterpret_cast<const uint8_t*>(&address),
                                       pipe >= 2 ? 1 : mAddressWidth, pipe);
  // Set the payload size for the pipe
  writeRegisterOffset<Register::RxPayloadWidthP0>(mPayloadSize, pipe);
  // Enable the address for receiving
  Register::EnabledRxAddresses en_rx = readRegister();
  en_rx.setValue(en_rx.value() | (1 << pipe));
  writeRegister(en_rx);
}

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::openWritingPipe(uint64_t address)
{
  // The radio expects this value LSB first
  // STM32 are usually little endian which is compatible
  openWritingPipe(reinterpret_cast<uint8_t*>(&address));
}

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::openWritingPipe(const uint8_t* address)
{
  // The radio expects this value LSB first
  // STM32 are usually little endian which is compatible
  writeRegister<Register::RxAddressP0>(address, mAddressWidth);
  writeRegister<Register::TxAddress>(address, mAddressWidth);

  Register::RxPayloadWidthP0 rxpayload;
  rxpayload.field<Field::RxPayloadWidthP0>() = mPayloadSize;
  writeRegister(rxpayload);
}

template <typename spi, typename csn, typename ce>
template <typename DataRate>
bool device<spi, csn, ce>::setDataRate()
{
  uint8_t reg;

  Register::RfSetup rfsetup = readRegister<Register::RfSetup>();
  rfsetup.write<DataRate>();
  writeRegister(rfsetup);

  // Verify our register was saved.
  auto set_rfsetup = readRegister<Register::RfSetup>();
  return set_rfsetup.value() == rfsetup.value();
}

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::startListening()
{
  Register::Config config        = readRegister<Register::Config>();
  config.field<Field::Primary>() = true;
  writeRegister(config);

  Register::Status status;
  status.field<Field::RxFifoDataReady>() = true;
  status.field<Field::TxFifoDataSent>()  = true;
  status.field<Field::MaxRetransmit>()   = true;
  writeRegister(status);

  ce::set();

  if (mP0RxAddress[0] > 0) {
    // Restore pipe 0 address if needed
    writeRegister<Register::RxAddressP0>(mP0RxAddress, mAddressWidth);
  } else {
    // Close pipe 0
    Register::EnabledRxAddresses en_rx_pipes =
        readRegister<Register::EnabledRxAddresses>();
    en_rx_pipes.field<Field::RxP0>() = 0;
    writeRegister(en_rx_pipes);
  }

  if (Register::Feature feature = readRegister<Register::Feature>();
      feature.field<Field::AckPayloadEnabled>()) {
    flushTx();
  }
}  // namespace nrf24

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::stopListening()
{
  using namespace blt;
  using namespace blt::time::literals;

  ce::clear();
  // \todo: 450 us is the maximum delay. It can be lower on 1Mbps and 2Mbps datarates
  time::delay(450_us);

  if (Register::Feature feature = readRegister();
      feature.field<Field::AckPayloadEnabled>()) {
    time::delay(450_us);
    flushTx();
  }

  // Reset primary
  Register::Config config        = readRegister();
  config.field<Field::Primary>() = 0;
  writeRegister(config);

  // Re-enable pipe 0
  Register::EnableAutoAck en_rx_pipes   = readRegister();
  en_rx_pipes.field<Field::AutoAckP0>() = true;
  writeRegister(en_rx_pipes);
}

template <typename spi, typename csn, typename ce>
bool device<spi, csn, ce>::available()
{
  Register::FifoStatus fifoStatus = readRegister();
  return !fifoStatus.field<Field::RxFifoEmpty>();
}

template <typename spi, typename csn, typename ce>
bool device<spi, csn, ce>::available(uint8_t& pipe)
{
  Register::Status status       = getStatus();
  uint8_t          rx_fifo_pipe = status.field<Field::RxFifoPayloadPipe>();

  return rx_fifo_pipe == pipe;
}

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::powerUp()
{
  using namespace blt;
  using namespace blt::time::literals;

  Register::Config config = readRegister<Register::Config>();

  // If power is down
  if (!config.field<Field::PowerUp>()) {
    config.field<Field::PowerUp>() = true;
    writeRegister(config);
    // Wait for power up
    time::delay(5_ms);
  }
}

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::powerDown()
{
  ce::clear();

  Register::Config config        = readRegister();
  config.field<Field::PowerUp>() = false;
  writeRegister(config);
}

template <typename spi, typename csn, typename ce>
bool device<spi, csn, ce>::write(const uint8_t* buf, uint8_t len, const bool multicast)
{
  startWriteFast(buf, len, multicast);

  // Wait until finished or error
  Register::Status status = getStatus();
  while (status.field<Field::TxFifoDataSent>() | status.field<Field::MaxRetransmit>()) {
    status = getStatus();
    // \todo Handle a timeout here
  }

  ce::clear();

  status.field<Field::RxFifoDataReady>() = 1;
  status.field<Field::TxFifoDataSent>()  = 1;
  status.field<Field::MaxRetransmit>()   = 1;
  status                                 = writeRegister(status);

  // Maximum retries exceeded
  if (status.field<Field::MaxRetransmit>()) {
    flushTx();
    return false;
  }

  return true;
}  // namespace nrf24

}  // namespace nrf24
