#pragma once

#include <utility>

#include "lsm9ds1_registers.hh"

namespace blt::lsm9ds1 {

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
template <typename register_t>
register_t device<bus, cs_ag, cs_m>::read_ag() const
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

  return {value};
}

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
template <typename register_t>
register_t device<bus, cs_ag, cs_m>::read_m() const
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

  return {value};
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

  cs_ag::clear();
  bus::rw(register_t::address & 0b00111111);
  for (std::size_t i = 0; i < count; i++) {
    bus::rw(v.getByte(i));
  }
  cs_ag::set();
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
    Control1Gyro ctrl1;
    ctrl1.set<Control1Gyro::Bandwidth>(Control1Gyro::Bandwidth::kSmall);
    ctrl1.set<Control1Gyro::FullScale>(Control1Gyro::FullScale::k2000dps);
    ctrl1.set<Control1Gyro::DataRate>(Control1Gyro::DataRate::k952Hz);
    write_ag(ctrl1);

    Control2Gyro ctrl2;
    write_ag(ctrl2);

    Control3Gyro ctrl3;
    ctrl3.set<Control3Gyro::HighPassCutoffFreq>(0);
    ctrl3.set<Control3Gyro::HighPassEnable>(false);
    ctrl3.set<Control3Gyro::LowPowerEnable>(false);
    write_ag(ctrl3);

    Control4Gyro ctrl4;
    ctrl4.set<Control4Gyro::Int4D>(Control4Gyro::Int4D::kUse6D);
    ctrl4.set<Control4Gyro::LatchedInt>(true);
    ctrl4.set<Control4Gyro::XOutputEnable>(true);
    ctrl4.set<Control4Gyro::YOutputEnable>(true);
    ctrl4.set<Control4Gyro::ZOutputEnable>(true);
    write_ag(ctrl4);

    GyroOrientation orient;
    orient.set<GyroOrientation::UserOrientation>(0);
    orient.set<GyroOrientation::FlipZ>(false);
    orient.set<GyroOrientation::FlipY>(false);
    orient.set<GyroOrientation::FlipX>(false);
    write_ag(orient);
  }

  // Init accelerometer
  {
    Control5Acc ctrl5;
    ctrl5.set<Control5Acc::XOutputEnable>(true);
    ctrl5.set<Control5Acc::YOutputEnable>(true);
    ctrl5.set<Control5Acc::ZOutputEnable>(true);
    ctrl5.set<Control5Acc::Decimation>(0);
    write_ag(ctrl5);

    Control6Acc ctrl6;
    ctrl6.set<Control6Acc::FilterBandwidth>(Control6Acc::FilterBandwidth::k50Hz);
    ctrl6.set<Control6Acc::Bandwidth>(Control6Acc::Bandwidth::kFromODR);
    ctrl6.set<Control6Acc::Scale>(Control6Acc::Scale::k16g);
    ctrl6.set<Control6Acc::OutputDataRate>(Control6Acc::OutputDataRate::k952Hz);
    write_ag(ctrl6);

    Control7Acc ctrl7;
    ctrl7.set<Control7Acc::HPIS1>(0);
    ctrl7.set<Control7Acc::FDS>(Control7Acc::FDS::kBypassInternalFilter);
    ctrl7.set<Control7Acc::LPFilterCutoff>(Control7Acc::LPFilterCutoff::kOdr_50);
    ctrl7.set<Control7Acc::HR>(0);
    write_ag(ctrl7);
  }

  return test_m;
}

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
std::tuple<float, float, float> device<bus, cs_ag, cs_m>::read_gyro() const
{
  const Gyro g = read_ag<Gyro>();
  return std::make_tuple(g.get<Gyro::X>(), g.get<Gyro::Y>(), g.get<Gyro::Z>());
}

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
std::tuple<float, float, float> device<bus, cs_ag, cs_m>::read_accel() const
{
  const Acc a = read_ag<Acc>();
  return std::make_tuple(a.get<Acc::X>(), a.get<Acc::Y>(), a.get<Acc::Z>());
}

}  // namespace blt::lsm9ds1