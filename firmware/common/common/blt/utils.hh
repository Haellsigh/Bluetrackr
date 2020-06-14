#pragma once

#include <algorithm>

namespace blt::utils {

/**
 * \brief Non copyable class
 */
class noncopyable {
 public:
  constexpr noncopyable()         = default;
  noncopyable(const noncopyable&) = delete;
  noncopyable& operator=(const noncopyable&) = delete;
  // ~noncopyable() {} // This apparently uses the heap.
};

/**
 * \name Disjunction
 */
template <uint16_t... values>
constexpr uint16_t disjunction() {
  return (values | ...);
}

/**
 * \name Flag disjunction
 */
template <uint16_t... values>
constexpr uint16_t disjunction_flag() {
  static_assert(std::max({values...}) < 16, "maximum value is 15");
  return ((1 << values) | ...);
}

}  // namespace blt::utils