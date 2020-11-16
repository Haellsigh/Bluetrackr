#pragma once

#include <cstdint>

#include <blt/gpio.hh>
#include <blt/memory/manip.hh>
#include <blt/utils.hh>

#include "lsm9ds1_registers.hh"

namespace blt::lsm9ds1 {

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
class device : public blt::utils::noncopyable {
 public:
  bool                                  init();
  std::tuple<int16_t, int16_t, int16_t> read_accel_raw() const;
  std::tuple<int16_t, int16_t, int16_t> read_gyro_raw() const;
  std::tuple<int16_t, int16_t, int16_t> read_mag_raw() const;
  std::tuple<float, float, float>       read_accel() const;
  std::tuple<float, float, float>       read_gyro() const;
  std::tuple<float, float, float>       read_mag() const;
  float                                 read_temperature() const;

  void calibrate_ag(float expected_ax, float expected_ay, float expected_az);
  void set_calibration_bias_m(int16_t bias_mx, int16_t bias_my, int16_t bias_mz);
  void set_calibration_matrix_m(float a11, float a22, float a33, float a21, float a31, float a32);

  void        enable_fifo(bool enable = true) const;
  void        set_fifo(uint8_t mode, uint8_t level) const;
  std::size_t fifo_samples() const;
  void        set_gyro_orientation(GyroOrientation value) const;

  bool available_a() const;
  bool available_g() const;
  bool available_m() const;

 protected:
  template <typename register_t>
  requires(register_t::address >= 0) register_t read_ag() const;
  template <typename register_t>
  requires(register_t::address >= 0) register_t read_m() const;
  template <typename field_t>
  requires(field_t::offset >= 0) field_t::value_t read_ag() const;
  template <typename field_t>
  requires(field_t::offset >= 0) field_t::value_t read_m() const;

  template <typename register_t>
  void write_ag(register_t const& v) const;
  template <typename register_t>
  void write_m(register_t const& v) const;

 protected:
  // accelerometer calibration
  int16_t bias_ax_ = 0, bias_ay_ = 0, bias_az_ = 0;
  // gyroscope calibration
  int16_t bias_gx_ = 0, bias_gy_ = 0, bias_gz_ = 0;
  // magnetometer calibration
  int   bias_mx_ = 0, bias_my_ = 0, bias_mz_ = 0;
  float a11_, a21_, a31_, a22_, a32_, a33_;
  // conversion factors
  float a_res_ = 0, g_res_ = 0, m_res_ = 0;
};

}  // namespace blt::lsm9ds1

#include "lsm9ds1_impl.hh"