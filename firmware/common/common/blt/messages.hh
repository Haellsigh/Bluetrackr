#pragma once

#include <cstdint>

namespace blt::message {

namespace motion {

struct type {
  int16_t x = 0, y = 0, z = 0;
  int16_t rx = 0, ry = 0, rz = 0;
};
constexpr uint8_t size = sizeof(type);

}  // namespace motion

}  // namespace blt::message