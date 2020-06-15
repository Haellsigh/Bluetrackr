#include <algorithm>  // min/max
#include <cstring>    // memcpy
#include <nrf24_custom/nrf24.hh>

namespace blt::nrf24 {

namespace Command {
/*!
 * \enum Command
 * \note The command word is sent from msbit to lsbit.
 * \note The data bytes are sent from lsbyte to msbyte.
 */
/*!
 * \var kNone
 * \brief No command.
 *
 * \var kDisable
 * \brief Disable feature.
 * \todo Move kDisable to another enum.
 *
 * \var kReadRegister
 * \brief Read command and status registers.
 * \note Last 5 bits are the register map address.
 *
 * \var kWriteRegister
 * \brief Write command and status registers.
 * \note Last 5 bits are the register map address.
 * \note Executable in Power down or standby modes only.
 *
 * \var kReadRxPayload
 * \brief Read receive payload (1-32 bytes).
 * \note Payload is deleted from FIFO after it is read.
 *
 * \var kWriteTxPayload
 * \brief Write transmit payload (1-32 bytes).
 *
 * \var kFlushRx
 * \brief Flush receive FIFO, used in receive mode.
 * \note Should not be executed during transmission of acknowledge, that is,
 *       acknowledge package will not be completed.
 *
 * \var kFlushTx
 * \brief Flush transmit FIFO, used in transmit mode.
 *
 * \var kReuseTxPayload
 * \brief Reuse last transmitted payload.
 * \note Used for a PTX device.
 * \note Transmit payload reuse is active until kWriteTxPayload or kFlushTx is
 * executed.
 * \note Transmit payload reuse must not be activated or deactivated during
 * package transmission.
 *
 * \var kNop
 * \brief No operation.
 * \brief Can be used to read the Register::kStatus register.
 */
}  // namespace Command

namespace CommandMask {
/*!
 * \enum CommandMask
 */
/*!
 * \var kRegisterReadWrite
 * \brief Command mask to access the register read/write command data.
 */
}  // namespace CommandMask

namespace Register {
/*!
 * \enum Register
 * \brief Register addresses
 * \note The associated bits for each register is described in the RegisterBit
 *       enumeration.
 */
/*!
 * \var kConfig
 * \brief Configuration register.
 *
 * \var kEnableAutoAck
 * \brief 'Auto Acknowledgment' setup enable on selected pipes.
 * \note Disable this feature to be compatible with nRF24L01.
 *
 * \var kEnabledRxAddresses
 * \brief Enabled RX pipes setup register.
 *
 * \var kSetupAddressWidths
 * \brief Address widths (common for all pipes) setup register.
 *
 * \var kSetupAutoRet
 * \brief Automatic retransmission setup register.
 *
 * \var kRfChannel
 * \brief RF Channel setup register.
 *
 * \var kRfSetup
 * \brief RF setup register.
 *
 * \var kStatus
 * \brief Status register.
 *
 * \var kObserveTx
 * \brief Transmit observe register.
 *
 * \var kReceivedPowerDetector
 * \brief Received power detector register.
 * \note This register is called Carrier Detect on nRF24L01.
 *
 * \var kRxAddressP0
 * \brief Receive address pipe 0 register.
 *
 * \var kRxAddressP1
 * \brief Receive address pipe 1 register.
 *
 * \var kRxAddressP2
 * \brief Receive address pipe 2 register.
 *
 * \var kRxAddressP3
 * \brief Receive address pipe 3 register.
 *
 * \var kRxAddressP4
 * \brief Receive address pipe 4 register.
 *
 * \var kRxAddressP5
 * \brief Receive address pipe 5 register.
 *
 * \var kTxAddress
 * \brief Transmit address register.
 * \note Used for a PTX device only.
 * \note Set kRxAddressP0 equal to this address to handle automatic acknowledge
 * if this is a PTX device with Enhanced ShockBurst enabled.
 *
 * \var kRxPayloadWidthP0
 * \brief Number of bytes of receive payload in pipe 0.
 *
 * \var kRxPayloadWidthP1
 * \brief Number of bytes of receive payload in pipe 1.
 *
 * \var kRxPayloadWidthP2
 * \brief Number of bytes of receive payload in pipe 2.
 *
 * \var kRxPayloadWidthP3
 * \brief Number of bytes of receive payload in pipe 3.
 *
 * \var kRxPayloadWidthP4
 * \brief Number of bytes of receive payload in pipe 4.
 *
 * \var kRxPayloadWidthP5
 * \brief Number of bytes of receive payload in pipe 5.
 *
 * \var kFifoStatus
 * \brief FIFO status register.
 *
 * \var kDynamicPayload
 * \brief Dynamic payload setup register.
 *
 * \var kFeature
 * \brief Feature (dynamic payload, payload in ack, no ack on packet) register.
 * \note The 'no ack on packet' function is not implemented.
 */
}  // namespace Register

namespace RegisterFieldMask {
/*!
 * \enum RegisterFieldMask
 * \brief Describes the bit mask of each bit of the registers.
 *
 * I'm actually too lazy to document each bit one by one, for now.
 * Just go to the
 * [documentation](https://infocenter.nordicsemi.com/pdf/nRF24L01P_PS_v1.0.pdf)
 * on section 9.1. Register map table.
 */
}  // namespace RegisterFieldMask

static constexpr uint8_t kMaxChannel     = 125;
static constexpr uint8_t kMaxPayloadSize = 32;

template <typename spi, typename csn, typename ce>
uint8_t inline device<spi, csn, ce>::spirw(uint8_t data) {
  return spi::rw(data);
}

template <typename spi, typename csn, typename ce>
constexpr inline void device<spi, csn, ce>::begin() {
  cs::set();
}

template <typename spi, typename csn, typename ce>
constexpr inline void device<spi, csn, ce>::end() {
  cs::clear();
}

template <typename spi, typename csn, typename ce>
uint8_t device<spi, csn, ce>::spiTransfer(uint8_t cmd) {
  uint8_t status;
  begin();

  // Execute command
  status = spirw(cmd);

  end();
  return status;
}

/**
 * \brief
 * \param reg The register to read.
 * \return The register value.
 */
template <typename spi, typename csn, typename ce>
uint8_t device<spi, csn, ce>::readRegister(uint8_t reg) {
  uint8_t result;
  begin();

  // Register read access
  spirw(Command::kReadRegister | (CommandMask::kRegisterReadWrite & reg));
  result = spirw(Command::kNop);

  end();
  return result;
}

/**
 * \param reg The register to read.
 * \param buf The register value(s).
 * \param len The length of buf.
 * \return The status of the operation.
 */
template <typename spi, typename csn, typename ce>
uint8_t device<spi, csn, ce>::readRegister(uint8_t reg, uint8_t* buf, uint8_t len) {
  uint8_t status;
  begin();

  // Register read access
  status = spirw(Command::kReadRegister | (CommandMask::kRegisterReadWrite & reg));
  while (len--) {
    *buf++ = spirw(Command::kNop);
  }

  end();
  return status;
}

/**
 *
 * @param reg
 * @param val
 * @return The status of the operation.
 */
template <typename spi, typename csn, typename ce>
uint8_t device<spi, csn, ce>::writeRegister(uint8_t reg, uint8_t val) {
  uint8_t status;
  begin();

  // Register write access
  status = spirw(Command::kWriteRegister | (CommandMask::kRegisterReadWrite & reg));
  spirw(val);

  end();
  return status;
}

/**
 *
 * @param reg
 * @param buf
 * @param len
 * @return The status of the operation.
 */
template <typename spi, typename csn, typename ce>
uint8_t device<spi, csn, ce>::writeRegister(uint8_t        reg,
                                            const uint8_t* buf,
                                            uint8_t        len) {
  uint8_t status;
  begin();

  status = spirw(Command::kWriteRegister | (CommandMask::kRegisterReadWrite & reg));
  while (len--) {
    spirw(*buf++);
  }

  end();
  return status;
}

template <typename spi, typename csn, typename ce>
uint8_t device<spi, csn, ce>::getStatus() {
  return spiTransfer(Command::kNop);
}

template <typename spi, typename csn, typename ce>
uint8_t device<spi, csn, ce>::flushTx() {
  return spiTransfer(Command::kFlushTx);
}

template <typename spi, typename csn, typename ce>
uint8_t device<spi, csn, ce>::flushRx() {
  return spiTransfer(Command::kFlushRx);
}

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::clearIRQFlags() {
  uint8_t reg;

  reg = readRegister(Register::kStatus);
  reg |= RegisterValue::kStatusIRQClear;
  writeRegister(Register::kStatus, reg);
}

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::setChannel(uint8_t channel) {
  writeRegister(Register::kRfChannel, std::min(channel, kMaxChannel));
}

template <typename spi, typename csn, typename ce>
uint8_t device<spi, csn, ce>::getChannel() {
  return readRegister(Register::kRfChannel);
}

template <typename spi, typename csn, typename ce>
uint8_t device<spi, csn, ce>::getAddressWidth() {
  return readRegister(Register::kSetupAddressWidths) + 2u;
}

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::setAddressWidth(uint8_t width) {
  if (width -= 2) {
    writeRegister(Register::kSetupAddressWidths, width % 4);
    m_addressWidth = (width % 4) + 2;
  } else {
    writeRegister(Register::kSetupAddressWidths, Command::kNone);
    m_addressWidth = 2;
  }
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
void device<spi, csn, ce>::toggleFeatures() {
  // From the nRF24L01 Rev2.0 datasheet, not present in nRF24L01+.
  // This does nothing on the nRF24L01+.
  begin();
  spirw(0x50);  // Activate
  spirw(0x73);  // ??
  end();
}

template <typename spi, typename csn, typename ce>
uint8_t device<spi, csn, ce>::readPayload(uint8_t* buf, uint8_t len) {
  uint8_t status;

  len               = std::min(len, m_payloadSize);
  uint8_t blank_len = m_dynamicPayloads ? 0 : m_payloadSize - len;

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
                                           const uint8_t  type) {
  uint8_t status;

  len               = std::min(len, m_payloadSize);
  uint8_t blank_len = m_dynamicPayloads ? 0 : m_payloadSize - len;

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
                                          bool           startTx) {
  writePayload(buf, len,
               multicast ? Command::kWriteTxPayloadNoAck : Command::kWriteTxPayload);
  if (startTx)
    ce::set();
}

/// Public functions

template <typename spi, typename csn, typename ce>
bool device<spi, csn, ce>::init() {
  using namespace time::literals;

  uint8_t setup = 0;
  ce::clear();
  cs::clear();
  time::delay(100_ms);

  // Power up delay
  time::delay(5_ms);

  /// Arbitrary default values for the registers
  // Reset config & enable 16-bit CRC
  writeRegister(Register::kConfig, 0x0C);

  // The minimum viable delay after testing on all speeds
  setAutoRetransmit(AutoRetransmitDelay::k1500us, 15);

  // This is a p variant if 250 kbps is supported
  if (setDataRate(DataRate::k250kbps)) {
    m_pVariant = true;
  }

  setup = readRegister(Register::kRfSetup);

  // Slowest common supported speed on both non-p and p variants
  setDataRate(DataRate::k1Mbps);

  // Disable dynamic payloads
  toggleFeatures();
  writeRegister(Register::kFeature, Command::kDisable);
  writeRegister(Register::kDynamicPayload, Command::kDisable);
  m_dynamicPayloads = false;

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
  writeRegister(Register::kConfig,
                readRegister(Register::kConfig) & ~RegisterFieldMask::kConfigPrimary);

  return (setup != 0x00) && (setup != 0xFF);
}

template <typename spi, typename csn, typename ce>
bool device<spi, csn, ce>::test() {
    uint8_t           rxbuf[5];
  uint8_t           txbuf[5] = {'n', 'R', 'F', '2', '4'};
  constexpr uint8_t bufsize  = 5;

  // Write the test address to the TxAddr register
  writeRegister(Register::kTxAddress, txbuf, bufsize);

  // Read it back
  readRegister(Register::kTxAddress, rxbuf, bufsize);

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
void device<spi, csn, ce>::setAutoAck(bool enable) {
  // Enable all registers
  if (enable)
    writeRegister(Register::kEnableAutoAck,
                  bit::maskBits<Pipe::kRx0, Pipe::kRx1, Pipe::kRx2, Pipe::kRx3,
                                Pipe::kRx4, Pipe::kRx5>());
  else
    writeRegister(Register::kEnableAutoAck, Command::kNone);
}

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::setAutoAck(bool enable, uint8_t pipe) {
  if (pipe <= Pipe::kTx) {
    uint8_t aa_state = readRegister(Register::kEnableAutoAck);

    if (enable)
      bit::set(aa_state, pipe);
    else
      bit::clear(aa_state, pipe);

    writeRegister(Register::kEnableAutoAck, aa_state);
  }
}

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::enableAckPayload() {
  // Enable ack payload (& dynamic payload)
  uint8_t feature = readRegister(Register::kFeature);
  feature |= RegisterFieldMask::kFeatureEnableAckPayload |
             RegisterFieldMask::kFeatureEnableDynamicPayload;
  writeRegister(Register::kFeature, feature);

  // Enable dynamic payload on all RX pipes
  uint8_t dynpd = readRegister(Register::kDynamicPayload);
  dynpd |= PipeFlag::kRxAll;
  writeRegister(Register::kDynamicPayload, dynpd);

  /*
  // Original code, this is weird because it says 'dynpd on only certain pipes is
  not really used', but then it does this:
  // Enable dynamic payload on pipes 0 & 1
  uint8_t dynpd = readRegister(Register::kDynamicPayload);
  dynpd |= bit::maskBits<Pipe::kRx0, Pipe::kRx1>();
  writeRegister(Register::kDynamicPayload, dynpd);
  */

  m_dynamicPayloads = true;
}

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::setDynamicPayload(bool enable) {
  // Enable dynamic payload
  if (enable) {
    // Enable dynamic payload feature
    uint8_t feature = readRegister(Register::kFeature);
    feature |= RegisterFieldMask::kFeatureEnableDynamicPayload;
    writeRegister(Register::kFeature, feature);

    // Enable dynamic payload on all RX pipes
    uint8_t dynpd = readRegister(Register::kDynamicPayload);
    dynpd |= PipeFlag::kRxAll;
    writeRegister(Register::kDynamicPayload, dynpd);

    m_dynamicPayloads = true;
  }
  // Disable dynamic payload
  else {
    // Disable dynamic payload feature
    writeRegister(Register::kFeature, Command::kNone);

    // Disable dynamic payload on all pipes
    writeRegister(Register::kDynamicPayload, Command::kNone);

    m_dynamicPayloads = false;
  }
}

/**
 * \brief Sets the automatic retransmission parameters.
 * @param enabled
 * @param ard
 * @param arc
 */
template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::setAutoRetransmit(AutoRetransmitDelay ard, uint8_t arc) {
  writeRegister(Register::kSetupAutoRet,
                (static_cast<uint8_t>(ard) << RegisterFieldBit::kSetupAutoRetArd) | arc);
}

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::setPayloadSize(uint8_t size) {
  m_payloadSize = std::min(size, kMaxPayloadSize);
}

/*!
 * \brief Sets the power amplifier gain level.
 */
template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::setPALevel(PowerAmplifier level) {
  // Get current RF configuration
  uint8_t setup = readRegister(Register::kRfSetup);
  // Clear bits 0-1-2
  setup &= ~bit::maskRange<0, 2>();
  // Write power level
  setup |= bit::shift<1>(static_cast<uint8_t>(level));
  if (!m_pVariant)
    setup |= bit::maskBits<0>();  // Sets the LNA gain for non p-variants

  writeRegister(Register::kRfSetup, setup);
}

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::openReadingPipe(uint8_t pipe, uint64_t address) {
  // Special case: cache the address for startListening().
  if (pipe == 0) {
    std::memcpy(m_P0RxAddress, &address, m_addressWidth);
    return;
  }
  // Write the address width (1 for pipe 2-5 or m_addressWidth for pipe 0-1).
  if (pipe >= 2) {
    writeRegister(Register::kRxAddressP0 + pipe,
                  reinterpret_cast<const uint8_t*>(&address), 1);
  } else {
    writeRegister(Register::kRxAddressP0 + pipe,
                  reinterpret_cast<const uint8_t*>(&address), m_addressWidth);
  }
  // Set the payload size for the pipe
  writeRegister(Register::kRxPayloadWidthP0 + pipe, m_payloadSize);
  // Enable the address for receiving
  uint8_t reg = readRegister(Register::kEnabledRxAddresses);
  bit::set(reg, static_cast<uint8_t>(Pipe::kRx0 + pipe));
  writeRegister(Register::kEnabledRxAddresses, reg);
}

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::openWritingPipe(uint64_t address) {
  // The radio expects this value LSB first
  // STM32 are usually little endian which is compatible

  writeRegister(Register::kRxAddressP0, reinterpret_cast<uint8_t*>(&address),
                m_addressWidth);
  writeRegister(Register::kTxAddress, reinterpret_cast<uint8_t*>(&address),
                m_addressWidth);

  writeRegister(Register::kRxPayloadWidthP0, m_payloadSize);
}

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::openWritingPipe(const uint8_t* address) {
  // The radio expects this value LSB first
  // STM32 are usually little endian which is compatible

  writeRegister(Register::kRxAddressP0, address, m_addressWidth);
  writeRegister(Register::kTxAddress, address, m_addressWidth);

  writeRegister(Register::kRxPayloadWidthP0, m_payloadSize);
}

template <typename spi, typename csn, typename ce>
bool device<spi, csn, ce>::setDataRate(uint8_t dataRate) {
  uint8_t reg;

  // Read the current setup register
  reg = readRegister(Register::kRfSetup);
  // Clears the datarate bits
  reg &= ~RegisterMask::kDatarate;
  // Write the datarate to those bits
  reg |= static_cast<uint8_t>(dataRate);

  writeRegister(Register::kRfSetup, reg);

  // Verify our register was saved.
  return readRegister(Register::kRfSetup) == reg;
}

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::startListening() {
  namespace bits = RegisterFieldBit;

  uint8_t cfg = readRegister(Register::kConfig);
  bit::set<bits::kConfigPrimary>(cfg);
  writeRegister(Register::kConfig, cfg);

  uint8_t status = 0;
  bit::set<bits::kStatusRxDataReady, bits::kStatusTxDataSent,
           bits::kStatusMaxRetransmits>(status);
  writeRegister(Register::kStatus, status);

  ce::set();

  if (m_P0RxAddress[0] > 0) {
    // Restore pipe 0 address if needed
    writeRegister(Register::kRxAddressP0, m_P0RxAddress, m_addressWidth);
  } else {
    // Close pipe 0
    uint8_t en_pipes = readRegister(Register::kEnabledRxAddresses);
    bit::clear<Pipe::kRx0>(en_pipes);
    writeRegister(Register::kEnabledRxAddresses, en_pipes);
  }

  if (readRegister(Register::kFeature) & RegisterFieldMask::kFeatureEnableAckPayload) {
    flushTx();
  }
}

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::stopListening() {
  using namespace time::literals;
  namespace bits = RegisterFieldBit;

  ce::clear();
  // \todo: 450 us is the maximum delay. It can be lower on 1Mbps and 2Mbps datarates
  time::delay(450_us);

  if (readRegister(Register::kFeature) & RegisterFieldMask::kFeatureEnableAckPayload) {
    time::delay(450_us);
    flushTx();
  }

  // Reset primary
  uint8_t cfg = readRegister(Register::kConfig);
  bit::clear<bits::kConfigPrimary>(cfg);
  writeRegister(Register::kConfig, cfg);

  // Re-enable pipe 0
  uint8_t en_pipes = readRegister(Register::kEnabledRxAddresses);
  bit::set<Pipe::kRx0>(en_pipes);
  writeRegister(Register::kEnabledRxAddresses, en_pipes);
}

template <typename spi, typename csn, typename ce>
bool device<spi, csn, ce>::available() {
  if (!(readRegister(Register::kFifoStatus) & RegisterFieldMask::kFifoStatusRxEmpty)) {
    return true;
  }
  return false;
}

template <typename spi, typename csn, typename ce>
bool device<spi, csn, ce>::available(uint8_t& pipe) {
  namespace mask = RegisterFieldMask;
  namespace bits = RegisterFieldBit;

  if (available()) {
    uint8_t status = getStatus();
    pipe = (status & mask::kStatusRxPayloadPipeN) >> bits::kStatusRxPayloadPipeN;
    return true;
  }
  return false;
}

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::powerUp() {
  using namespace time::literals;

  uint8_t cfg = readRegister(Register::kConfig);

  // If we're powered down
  if (!(cfg & RegisterFieldMask::kConfigPower)) {
    // Power up
    writeRegister(Register::kConfig, cfg | RegisterFieldMask::kConfigPower);
    // Wait for the radio
    time::delay(5_ms);
  }
}

template <typename spi, typename csn, typename ce>
void device<spi, csn, ce>::powerDown() {
  namespace b = RegisterFieldBit;

  ce::clear();

  uint8_t cfg = readRegister(Register::kConfig);
  // Clear the power up bit
  bit::clear<b::kConfigPower>(cfg);

  writeRegister(Register::kConfig, cfg);
}

template <typename spi, typename csn, typename ce>
bool device<spi, csn, ce>::write(const uint8_t* buf, uint8_t len, const bool multicast) {
  namespace m = RegisterFieldMask;
  namespace b = RegisterFieldBit;

  startWriteFast(buf, len, multicast);

  // Wait until finished or error
  while (!(getStatus() & (m::kConfigMaskTxDataSent | m::kConfigMaskMaxRet))) {
    // \todo Handle a timeout here
  }

  ce::clear();

  uint8_t status =
      writeRegister(Register::kStatus, m::kConfigMaskMaxRet | m::kConfigMaskTxDataSent |
                                           m::kConfigMaskRxDataReady);

  // Maximum retries exceeded
  if (status & m::kConfigMaskMaxRet) {
    flushTx();
    return false;
  }

  return true;
}

}  // namespace blt::nrf24