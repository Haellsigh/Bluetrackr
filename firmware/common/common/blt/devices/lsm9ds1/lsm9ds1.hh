#pragma once

#include <cstdint>

#include <blt/gpio.hh>
#include <blt/memory/manip.hh>
#include <blt/utils.hh>

namespace blt::lsm9ds1 {

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
class device : public blt::utils::noncopyable {
 public:
  bool                            init();
  std::tuple<float, float, float> read_gyro() const;
  std::tuple<float, float, float> read_accel() const;
  // std::tuple<float, float, float> read_mag();

 protected:
  template <typename register_t>
  register_t read_ag() const;
  template <typename register_t>
  register_t read_m() const;

  template <typename register_t>
  void write_ag(register_t const& v) const;
  template <typename register_t>
  void write_m(register_t const& v) const;
};

}  // namespace blt::lsm9ds1

#include "lsm9ds1_impl.hh"