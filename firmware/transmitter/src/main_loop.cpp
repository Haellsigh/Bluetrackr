#include "main_loop.hh"

#include "error_handler.hh"

#include <math.h>

#include <blt/algorithms/madgwick.hh>
#include <blt/chrono.hh>
#include <blt/devices/lsm9ds1/lsm9ds1.hh>
#include <blt/devices/nrf24/nrf24.hh>
#include <blt/gpio.hh>
#include <blt/messages.hh>
#include <blt/uart.hh>
#include <spi/spi.hh>

template <typename T>
T constexpr pi = std::acos(-T(1));

const uint8_t nrf24_addresses[][6] = {"1Node", "2Node"};

SPI_HandleTypeDef* g_hspi1 = nullptr;
SPI_HandleTypeDef* g_hspi2 = nullptr;
TIM_HandleTypeDef* g_htim  = nullptr;

auto fhSpi1()
{
  return g_hspi1;
}

auto fhSpi2()
{
  return g_hspi2;
}

void main_loop(ADC_HandleTypeDef* hadc,
               TIM_HandleTypeDef* htim,
               SPI_HandleTypeDef* hspi1,
               SPI_HandleTypeDef* hspi2)
{
  using namespace blt;
  using namespace gpio;
  using namespace chrono::literals;

  using led_status = gpio::pin_out<PB, 8>;
  // using led_power  = gpio::pin_out<PB, 9>;
  using leds = gpio::pin_out<PB, 8 /*, 9*/>;

  using btn_pair = gpio::pin_in<PB, 6>;

  using csn_nrf = gpio::pin_out<PA, 4>;
  using ce_nrf  = gpio::pin_out<PA, 3>;
  using cs_ag   = gpio::pin_out<PB, 12>;
  using cs_m    = gpio::pin_out<PB, 11>;

  g_hspi1 = hspi1;
  g_hspi2 = hspi2;
  g_htim  = htim;

  using spi1 = spi::device<fhSpi1>;
  using spi2 = spi::device<fhSpi2>;

  HAL_TIM_Base_Start(g_htim);
  chrono::init([]() noexcept {
    return chrono::clock::time_point{chrono::clock::duration{__HAL_TIM_GET_COUNTER(g_htim)}};
  });
  leds::clear();

  // Press and hold reset for 1s to access DFU
  if (btn_pair::read()) {
    bool dfu   = true;
    auto watch = chrono::stopwatch{};
    while (watch.elapsed() < 1s && dfu)
      dfu &= btn_pair::read();
    if (dfu)
      dfu_run_bootloader();
  }

  // Power up delay and whatever else is needed
  chrono::delay(10ms);

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
  radio.setRfPowerLevel<nrf24::Value::RfPower0dBm>();
  radio.setDataRate<nrf24::Value::RfDatarate1Mbps>();
  // radio.setAutoAck(true);
  // radio.setupAutoRetransmit<nrf24::Value::AutoRetransmitDelay500us, 15>();
  radio.setAddressWidth(5);
  radio.setPayloadSize(blt::message::motion::size);
  radio.openWritingPipe(nrf24_addresses[1]);
  radio.openReadingPipe(1, nrf24_addresses[0]);

  radio.stopListening();
  radio.powerUp();

  imu.calibrate_ag(0, 0, 1);
  imu.set_calibration_bias_m(-686, 2497, -2149);
  imu.set_calibration_matrix_m(0.97489937f, 1.01780377f, 1.0096359f, 0.0397189729f, -0.0047589753f,
                               -0.014580024f);

  led_status::clear();

  algorithms::madgwick filter;
  float                yaw_hp = 0, last_yaw = 0;

  chrono::stopwatch filter_watch;

  chrono::periodic_task sender{
      200_hz, [&](const auto& deltat, auto& radio, auto& data) {
        const auto [roll, pitch, yaw] = filter.get();

        const auto  dt    = std::chrono::duration_cast<chrono::fsec>(deltat).count();
        const float alpha = 50.f / (50.f + dt);
        yaw_hp            = alpha * yaw_hp + alpha * (yaw - last_yaw);
        data.roll         = 182.038888889 * roll;
        data.pitch        = 364.077777778 * pitch;
        data.yaw          = 182.038888889 * yaw_hp;

        last_yaw = yaw;

        led_status::toggle();
        radio.writeFast(reinterpret_cast<uint8_t*>(&data), blt::message::motion::size);
      }};

  filter_watch.restart();

  while (true) {
    if (btn_pair::read()) {
      NVIC_SystemReset();
    }

    if (imu.available_a()) {
      const auto [ay, ax, az] = imu.read_accel();
      const auto [gy, gx, gz] = imu.read_gyro();
      // const auto [nmy, mx, mz] = imu.read_mag();

      const auto dt = std::chrono::duration_cast<chrono::fsec>(filter_watch.restart()).count();

      // filter.update(dt, gy, gx, gz, ay, ax, az, mx, -nmy, mz);
      filter.update(dt, gx, gy, gz, ax, ay, az);
    }

    sender.update(radio, data);
  }

  error_handler();
}
