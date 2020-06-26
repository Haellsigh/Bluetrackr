#pragma once

#include <algorithm>
#include <type_traits>

namespace blt::utils {

namespace literals {

inline uint8_t operator"" _u8(uint64_t value)
{
  return static_cast<uint8_t>(value);
}

}  // namespace literals

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
 * Converts an enum to it's underlying type (without expliciting the type).
 */
template <typename E>
constexpr auto to_underlying(E e) noexcept
{
  return static_cast<std::underlying_type_t<E>>(e);
}

/**
 * \name Disjunction
 */
template <uint16_t... values>
constexpr uint16_t disjunction()
{
  return (values | ...);
}

/**
 * \name Flag disjunction
 */
template <uint16_t... values>
constexpr uint16_t disjunction_flag()
{
  static_assert(std::max({values...}) < 16, "maximum value is 15");
  return ((1 << values) | ...);
}

}  // namespace blt::utils