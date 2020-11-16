#pragma once

#include <utility>

#include "lsm9ds1_registers.hh"

namespace blt::lsm9ds1 {

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
template <typename register_t>
requires(register_t::address >= 0) register_t device<bus, cs_ag, cs_m>::read_ag() const
{
  using type           = register_t::value_t;
  constexpr auto count = register_t::size_bytes;
  type           value = type{};

  cs_ag::clear();
  bus::rw(0b10000000 | (register_t::address & 0b01111111));
  for (std::size_t i = 0; i < count; i++) {
    value |= (static_cast<type>(bus::rw(0)) << (8 * i));
  }
  cs_ag::set();

  return register_t{value};
}

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
template <typename register_t>
requires(register_t::address >= 0) register_t device<bus, cs_ag, cs_m>::read_m() const
{
  using type           = register_t::value_t;
  constexpr auto count = register_t::size_bytes;
  type           value = type{};

  cs_m::clear();
  bus::rw(0b11000000 | (register_t::address & 0b00111111));
  for (std::size_t i = 0; i < count; i++) {
    value |= (static_cast<type>(bus::rw(0)) << (8 * i));
  }
  cs_m::set();

  return register_t{value};
}

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
template <typename field_t>
requires(field_t::offset >= 0) field_t::value_t device<bus, cs_ag, cs_m>::read_ag() const
{
  using register_t = typename field_t::register_t;
  return read_ag<register_t>().template get<field_t>();
}

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
template <typename field_t>
requires(field_t::offset >= 0) field_t::value_t device<bus, cs_ag, cs_m>::read_m() const
{
  using register_t = typename field_t::register_t;
  return read_m<register_t>().template get<field_t>();
}

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
template <typename register_t>
void device<bus, cs_ag, cs_m>::write_ag(register_t const& v) const
{
  constexpr auto count = register_t::size_bytes;

  cs_ag::clear();
  bus::rw(register_t::address & 0b01111111);
  for (std::size_t i = 0; i < count; i++) {
    bus::rw(v.getByte(i));
  }
  cs_ag::set();
}

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
template <typename register_t>
void device<bus, cs_ag, cs_m>::write_m(register_t const& v) const
{
  constexpr auto count = register_t::size_bytes;

  cs_m::clear();
  bus::rw(register_t::address & 0b00111111);
  for (std::size_t i = 0; i < count; i++) {
    bus::rw(v.getByte(i));
  }
  cs_m::set();
}

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
bool device<bus, cs_ag, cs_m>::init()
{
  cs_ag::set();
  cs_m::set();

  const auto test_ag = read_ag<WhoAmI>().get();
  const auto test_m  = read_m<WhoAmI>().get();

  if (test_ag != 0b01101000 || test_m != 0b00111101) {
    return false;
  }

  // Init gyroscope
  {
    Control1AG ctrl1;
    ctrl1.set<Control1AG::Bandwidth>(Control1AG::Bandwidth::kSmall);
    ctrl1.set<Control1AG::FullScale>(Control1AG::FullScale::k2000dps);
    ctrl1.set<Control1AG::DataRate>(Control1AG::DataRate::k952Hz);
    write_ag(ctrl1);
    g_res_ = 70. / 1000.;

    Control2AG ctrl2;
    write_ag(ctrl2);

    Control3AG ctrl3;
    ctrl3.set<Control3AG::HighPassCutoffFreq>(0);
    ctrl3.set<Control3AG::HighPassEnable>(false);
    ctrl3.set<Control3AG::LowPowerEnable>(false);
    write_ag(ctrl3);

    Control4AG ctrl4;
    ctrl4.set<Control4AG::Int4D>(Control4AG::Int4D::kUse6D);
    ctrl4.set<Control4AG::LatchedInt>(true);
    ctrl4.set<Control4AG::XOutputEnable>(true);
    ctrl4.set<Control4AG::YOutputEnable>(true);
    ctrl4.set<Control4AG::ZOutputEnable>(true);
    write_ag(ctrl4);

    GyroOrientation orient;
    orient.set<GyroOrientation::UserOrientation>(GyroOrientation::UserOrientation::kXYZ);
    orient.set<GyroOrientation::FlipZ>(false);
    orient.set<GyroOrientation::FlipY>(false);
    orient.set<GyroOrientation::FlipX>(false);
    set_gyro_orientation(orient);
  }

  // Init accelerometer
  {
    Control5AG ctrl5;
    ctrl5.set<Control5AG::XOutputEnable>(true);
    ctrl5.set<Control5AG::YOutputEnable>(true);
    ctrl5.set<Control5AG::ZOutputEnable>(true);
    ctrl5.set<Control5AG::Decimation>(0);
    write_ag(ctrl5);

    Control6AG ctrl6;
    ctrl6.set<Control6AG::FilterBandwidth>(Control6AG::FilterBandwidth::k408Hz);
    ctrl6.set<Control6AG::Bandwidth>(Control6AG::Bandwidth::kFromODR);
    ctrl6.set<Control6AG::Scale>(Control6AG::Scale::k4g);
    ctrl6.set<Control6AG::OutputDataRate>(Control6AG::OutputDataRate::k952Hz);
    write_ag(ctrl6);
    a_res_ = 0.122 / 1000.;

    Control7AG ctrl7;
    ctrl7.set<Control7AG::HPIS1>(0);
    ctrl7.set<Control7AG::DisableFilter>(false);
    ctrl7.set<Control7AG::FilterCutoff>(Control7AG::FilterCutoff::kOdr_50);
    ctrl7.set<Control7AG::EnableHighResolution>(true);
    write_ag(ctrl7);
  }

  // Init magnetometer
  {
    Ctrl1M ctrl1;
    ctrl1.set<Ctrl1M::EnableSelfTest>(false);
    ctrl1.set<Ctrl1M::EnableTemperatureComp>(false);
    ctrl1.set<Ctrl1M::DataRate>(Ctrl1M::DataRate::k80Hz);
    ctrl1.set<Ctrl1M::EnableFastODR>(false);
    ctrl1.set<Ctrl1M::XYOperativeMode>(Ctrl1M::XYOperativeMode::kUltraHighPerf);
    write_m(ctrl1);

    Ctrl2M ctrl2;
    ctrl2.set<Ctrl2M::SoftReset>(false);
    ctrl2.set<Ctrl2M::RebootMemory>(false);
    ctrl2.set<Ctrl2M::ScaleSelection>(Ctrl2M::ScaleSelection::k4gauss);
    write_m(ctrl2);
    m_res_ = 0.14 / 1000.;

    Ctrl3M ctrl3;
    ctrl3.set<Ctrl3M::OperatingMode>(Ctrl3M::OperatingMode::kContinuous);
    ctrl3.set<Ctrl3M::SPIWrite>(Ctrl3M::SPIWrite::kWriteOnly);
    ctrl3.set<Ctrl3M::LowPower>(false);
    ctrl3.set<Ctrl3M::I2CDisable>(false);
    write_m(ctrl3);

    Ctrl4M ctrl4;
    ctrl4.set<Ctrl4M::Endianness>(Ctrl4M::Endianness::kLSBFirst);
    ctrl4.set<Ctrl4M::ZOperativeMode>(Ctrl4M::ZOperativeMode::kUltraHighPerf);
    write_m(ctrl4);

    Ctrl5M ctrl5;
    ctrl5.set<Ctrl5M::BlockDataUpdate>(false);
    ctrl5.set<Ctrl5M::FastRead>(false);
    write_m(ctrl5);
  }

  return test_m;
}

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
std::tuple<int16_t, int16_t, int16_t> device<bus, cs_ag, cs_m>::read_accel_raw() const
{
  const Acc a = read_ag<Acc>();
  return std::make_tuple(a.get<Acc::X>() - bias_ax_, a.get<Acc::Y>() - bias_ay_,
                         a.get<Acc::Z>() - bias_az_);
}

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
std::tuple<int16_t, int16_t, int16_t> device<bus, cs_ag, cs_m>::read_gyro_raw() const
{
  const Gyro g = read_ag<Gyro>();
  return std::make_tuple(g.get<Gyro::X>() - bias_gx_, g.get<Gyro::Y>() - bias_gy_,
                         g.get<Gyro::Z>() - bias_gz_);
}

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
std::tuple<int16_t, int16_t, int16_t> device<bus, cs_ag, cs_m>::read_mag_raw() const
{
  const Mag m = read_m<Mag>();
  return std::make_tuple(m.get<Mag::X>(), m.get<Mag::Y>(), m.get<Mag::Z>());
}

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
std::tuple<float, float, float> device<bus, cs_ag, cs_m>::read_accel() const
{
  const auto [ax, ay, az] = read_accel_raw();
  return {ax * a_res_, ay * a_res_, az * a_res_};
}

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
std::tuple<float, float, float> device<bus, cs_ag, cs_m>::read_gyro() const
{
  const auto [gx, gy, gz] = read_gyro_raw();
  return {gx * g_res_, gy * g_res_, gz * g_res_};
}

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
std::tuple<float, float, float> device<bus, cs_ag, cs_m>::read_mag() const
{
  // Uncorrected measurement
  const auto [mux, muy, muz] = read_mag_raw();

  // Correct hard-iron (offset)
  const auto mcx = mux - bias_mx_;
  const auto mcy = muy - bias_my_;
  const auto mcz = muz - bias_mz_;
  // Correct soft-iron (deformation)
  const auto mx = a11_ * mcx + a21_ * mcy + a31_ * mcz;
  const auto my = a21_ * mcx + a22_ * mcy + a32_ * mcz;
  const auto mz = a31_ * mcx + a32_ * mcy + a33_ * mcz;

  return {mx * m_res_, my * m_res_, mz * m_res_};
}

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
float device<bus, cs_ag, cs_m>::read_temperature() const
{
  const Temperature t = read_ag<Temperature>();

  // Scale:  16 LSB/°C <=> value >> 4
  // Offset: 0 LSB at 25°C
  return 25.f + (((t.get<Temperature::High>() << 8) | t.get<Temperature::Low>()) >> 4);
}

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
void device<bus, cs_ag, cs_m>::calibrate_ag(float expected_ax, float expected_ay, float expected_az)
{
  constexpr uint8_t samples = 31;
  constexpr uint8_t n       = 10;
  const int16_t     exp_ax  = expected_ax / a_res_;
  const int16_t     exp_ay  = expected_ay / a_res_;
  const int16_t     exp_az  = expected_az / a_res_;

  int32_t bias_ax = 0, bias_ay = 0, bias_az = 0;
  int32_t bias_gx = 0, bias_gy = 0, bias_gz = 0;

  enable_fifo();
  set_fifo(FifoControl::FifoModeSelection::kFifo, samples);

  for (uint8_t k = 0; k < n; k++) {
    while (fifo_samples() < samples)
      ;

    for (uint8_t i = 0; i < samples; i++) {
      const auto [gx, gy, gz] = read_gyro_raw();
      const auto [ax, ay, az] = read_accel_raw();
      bias_ax += ax - exp_ax;
      bias_ay += ay - exp_ay;
      bias_az += az - exp_az;
      bias_gx += gx;
      bias_gy += gy;
      bias_gz += gz;
    }
  }

  bias_gx_ = bias_gx / (samples * n);
  bias_gy_ = bias_gy / (samples * n);
  bias_gz_ = bias_gz / (samples * n);
  bias_ax_ = bias_ax / (samples * n);
  bias_ay_ = bias_ay / (samples * n);
  bias_az_ = bias_az / (samples * n);

  enable_fifo(false);
  set_fifo(FifoControl::FifoModeSelection::kOff, 0);
}

/**
 * \brief Corrects the hard-iron offset.
 *
 * \note The values are in raw units (LSB).
 *
 * \param bias_mx The bias on the x axis.
 * \param bias_my The bias on the y axis.
 * \param bias_mz The bias on the z axis.
 */
template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
void device<bus, cs_ag, cs_m>::set_calibration_bias_m(int16_t bias_mx,
                                                      int16_t bias_my,
                                                      int16_t bias_mz)
{
  bias_mx_ = bias_mx;
  bias_my_ = bias_my;
  bias_mz_ = bias_mz;
}

/**
 * \brief Coefficients of the symmetric matrix to correct the soft-iron deformation.
 *
 * \note The values apply to the raw units (LSB).
 *
 * \param a11 The first diagonal coefficient
 * \param a22 The second diagonal coefficient
 * \param a33 The third diagonal coefficient
 * \param a21 = a12
 * \param a31 = a13
 * \param a32 = a23
 */
template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
void device<bus, cs_ag, cs_m>::set_calibration_matrix_m(float a11,
                                                        float a22,
                                                        float a33,
                                                        float a21,
                                                        float a31,
                                                        float a32)
{
  a11_ = a11;
  a22_ = a22;
  a33_ = a33;
  a21_ = a21;
  a31_ = a31;
  a32_ = a32;
}

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
void device<bus, cs_ag, cs_m>::enable_fifo(bool enable) const
{
  Control9AG ctrl9 = read_ag<Control9AG>();
  ctrl9.set<Control9AG::FifoEnable>(enable);
  write_ag(ctrl9);
}

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
void device<bus, cs_ag, cs_m>::set_fifo(uint8_t mode, uint8_t level) const
{
  FifoControl fifo_ctrl = read_ag<FifoControl>();
  fifo_ctrl.set<FifoControl::FifoThresholdLevel>(level);
  fifo_ctrl.set<FifoControl::FifoModeSelection>(mode);
  write_ag(fifo_ctrl);
}

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
std::size_t device<bus, cs_ag, cs_m>::fifo_samples() const
{
  return read_ag<FifoStatusControl::SampleCount>();
}

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
void device<bus, cs_ag, cs_m>::set_gyro_orientation(GyroOrientation value) const
{
  write_ag(value);
}

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
bool device<bus, cs_ag, cs_m>::available_a() const
{
  //
  return read_ag<Status2AG>().template get<Status2AG::XLDA>();
}

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
bool device<bus, cs_ag, cs_m>::available_g() const
{
  // return read_ag<Status1AG>().template get<Status1AG::DataAvailableGyro>();
  return read_ag<Status2AG>().template get<Status2AG::GDA>();
}

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
bool device<bus, cs_ag, cs_m>::available_m() const
{
  const MStatus s = read_m<MStatus>();
  return s.get<MStatus::XDataReady>() | s.get<MStatus::YDataReady>() | s.get<MStatus::ZDataReady>();
}

}  // namespace blt::lsm9ds1