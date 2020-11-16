#pragma once

#include <cstdint>

namespace blt::message {

namespace motion {

struct type {
  int16_t x = 0, y = 0, z = 0;
  int16_t roll = 0, pitch = 0, yaw = 0;
};
constexpr int size = sizeof(type);

}  // namespace motion

}  // namespace blt::message