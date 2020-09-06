#pragma once

#include <cstdint>

#include <blt/gpio.hh>
#include <blt/utils.hh>

namespace blt::lsm9ds1 {

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
class device : public blt::utils::noncopyable {
 public:
  void                            init();
  std::tuple<float, float, float> readGyro();
  std::tuple<float, float, float> readAccel();
  // std::tuple<float, float, float> readMag();

 protected:
}

}  // namespace blt::lsm9ds1

#include "lsm9ds1_impl.hh"