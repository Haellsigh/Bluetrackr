#include "main_loop.hh"

#include "error_handler.hh"

#include <cmath>

#include <blt/devices/lsm9ds1/lsm9ds1.hh>
#include <blt/devices/nrf24/nrf24.hh>
#include <blt/gpio.hh>
#include <blt/messages.hh>
#include <blt/time.hh>
#include <blt/uart.hh>
#include <spi/spi.hh>

SPI_HandleTypeDef* g_hspi1 = nullptr;
auto               fhSpi1()
{
  return g_hspi1;
}

SPI_HandleTypeDef* g_hspi2 = nullptr;
auto               fhSpi2()
{
  return g_hspi2;
}

const uint8_t nrf24_addresses[][6] = {"1Node", "2Node"};

void main_loop(SPI_HandleTypeDef* hspi1, SPI_HandleTypeDef* hspi2)
{
  using namespace blt;
  using namespace gpio;
  using namespace time::literals;

  using led_status = gpio::pin_out<PB, 8>;
  // using led_power  = gpio::pin_out<PB, 9>;
  using leds = gpio::pin_out<PB, 8 /*, 9*/>;

  using btn_pair = gpio::pin_in<PB, 6>;

  using csn_nrf = gpio::pin_out<PA, 4>;
  using ce_nrf  = gpio::pin_out<PA, 3>;
  using cs_ag   = gpio::pin_out<PB, 12>;
  using cs_m    = gpio::pin_out<PB, 11>;

  // Allows accessing the bootloading before any bullshit :)
  if (btn_pair::read()) {
    dfu_run_bootloader();
  }

  g_hspi1 = hspi1;
  g_hspi2 = hspi2;

  using spi1 = spi::device<fhSpi1>;
  using spi2 = spi::device<fhSpi2>;

  time::init();
  leds::clear();

  nrf24::device<spi1, csn_nrf, ce_nrf> radio;
  lsm9ds1::device<spi2, cs_ag, cs_m>   imu;
  blt::message::motion::type           data;

  led_status::set();
  if (!radio.init()) {
    error_handler();
  }

  if (!radio.test()) {
    error_handler();
  }

  if (!imu.init()) {
    error_handler();
  }

  radio.setChannel(40);
  radio.setRfPowerLevel<nrf24::Value::RfPowerMinimum>();
  radio.setDataRate<nrf24::Value::RfDatarate250kbps>();
  radio.setAutoAck(true);
  radio.setupAutoRetransmit<nrf24::Value::AutoRetransmitDelay1500us, 15>();
  radio.setAddressWidth(5);
  radio.openWritingPipe(nrf24_addresses[1]);
  radio.openReadingPipe(1, nrf24_addresses[0]);

  radio.startListening();
  radio.stopListening();

  radio.powerUp();

  led_status::clear();

  while (true) {
    if (btn_pair::read()) {
      dfu_run_bootloader();
    }

    const auto [ax, ay, az] = imu.read_accel();
    const auto [gx, gy, gz] = imu.read_gyro();

    data.x  = ax;
    data.y  = ay;
    data.z  = az;
    data.rx = gx;
    data.ry = gy;
    data.rz = gz;

    led_status::toggle();
    radio.writeFast(reinterpret_cast<uint8_t*>(&data), blt::message::motion::size);
  }

  error_handler();
}
