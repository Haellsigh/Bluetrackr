#pragma once

#include <cstdint>

namespace blt::time {

struct Microseconds {
  uint32_t us;
};

namespace literals {

constexpr Microseconds operator"" _us(uint64_t us) {
  return Microseconds{us};
}
constexpr Microseconds operator"" _ms(uint64_t ms) {
  return Microseconds{1000 * ms};
}

}  // namespace literals

void init();
void delay(Microseconds t);

}  // namespace blt::time