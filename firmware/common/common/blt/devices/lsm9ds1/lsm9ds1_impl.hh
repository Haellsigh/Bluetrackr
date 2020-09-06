#pragma once

#include "lsm9ds1_registers.hh"

namespace blt::lsm9ds1 {

template <typename bus, gpio::pin cs_ag, gpio::pin cs_m>
void device<bus, cs_ag, cs_m>::init()
{}

}  // namespace blt::lsm9ds1