#pragma once

#include <cstdint>

namespace blt::time {

struct Microseconds {
  uint32_t us;
};

namespace literals {

constexpr Microseconds operator"" _us(uint64_t us)
{
  return Microseconds{static_cast<uint32_t>(us)};
}
constexpr Microseconds operator"" _ms(uint64_t ms)
{
  return Microseconds{1000 * static_cast<uint32_t>(ms)};
}

}  // namespace literals

void init();
void delay(Microseconds t);

}  // namespace blt::time