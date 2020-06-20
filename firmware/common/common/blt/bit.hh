#pragma once

#include <algorithm>
#include <cstdint>

namespace blt::bit {

/**
 * \brief Creates a bit mask from bits.
 */
template <uint8_t... positions>
requires((std::max({positions...}) < 8) && (sizeof...(positions) < 8)) struct mask {
  static constexpr uint8_t value = ((1 << positions) | ...);
  static constexpr uint8_t start = std::min({positions...});
};

/**
 * \brief Creates a bit mask from the range [begin, end] or [end, begin].
 */
template <uint8_t begin, uint8_t end>
requires(begin <= end) struct mask_range {
  static constexpr uint8_t value = (((1 << begin) - 1) ^ ((1 << (end + 1)) - 1));
  static constexpr uint8_t start = begin;
};

/**
 * \brief Joins two or more bit masks.
 * \{
 */
template <typename...>
struct join;

template <uint8_t... positionsA, uint8_t... positionsB>
struct join<mask<positionsA...>, mask<positionsB...>> {
  static constexpr uint8_t value = mask<positionsA..., positionsB...>::value;
};
/// \}

template <typename mask_t>
inline constexpr void invert(uint8_t& output)
{
  output ^= mask_t::value;
}

inline constexpr void invert(uint8_t& output)
{
  output = ~output;
}

template <typename mask_t>
inline constexpr uint8_t get(uint8_t& output)
{
  return (output & mask_t::value) >> mask_t::start;
}

template <typename mask_t>
inline constexpr void set(uint8_t& output)
{
  output |= mask_t::value;
}

template <typename mask_t>
inline constexpr void set(uint8_t& output, uint8_t value)
{
  output |= (value & mask_t::value);
}

template <typename mask_t>
inline constexpr void clear(uint8_t& output)
{
  output &= ~mask_t::value;
}

template <typename mask_t>
inline constexpr void setclear(uint8_t& output, uint8_t input)
{
  clear<mask_t>(output);
  set<mask_t>(output, input << mask_t::start);
}

template <uint8_t... positions>
inline constexpr void setclear(uint8_t& output,
                               uint8_t  input,
                               uint8_t  begin,
                               uint8_t  index)
{
  constexpr std::array<uint8_t, sizeof...(positions)> pos = {{positions...}};

  const uint8_t position = pos[index] + begin;
  const uint8_t shift    = (1 << position);
  output &= ~shift;
  output |= ((input << position) & shift);
}

}  // namespace blt::bit
