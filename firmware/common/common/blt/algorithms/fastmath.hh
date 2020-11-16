#pragma once

namespace blt::fastmath {

inline constexpr float inverse_sqrt(float x)
{
  float halfx = 0.5f * x;
  float y     = x;
  long  i     = *(long*)&y;
  i           = 0x5f3759df - (i >> 1);
  y           = *(float*)&i;
  y           = y * (1.5f - (halfx * y * y));
  y           = y * (1.5f - (halfx * y * y));
  return y;
}

}  // namespace blt::fastmath
