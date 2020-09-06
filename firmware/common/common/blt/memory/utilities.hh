#pragma once

#include <cstdint>
#include <type_traits>

namespace blt::memory::utilities {

namespace details {

template <typename T>
struct return_t {
  typedef T type;
};

template <uint64_t bits>
constexpr auto bytecount = (bits + 7) >> 3;

template <uint64_t bytes>
struct bytetype : return_t<uint64_t> {};

template <>
struct bytetype<4> : return_t<uint32_t> {};

template <>
struct bytetype<3> : return_t<uint32_t> {};

template <>
struct bytetype<2> : return_t<uint16_t> {};

template <>
struct bytetype<1> : return_t<uint8_t> {};

}  // namespace details

template <uint64_t bits>
using smallestint_t = typename details::bytetype<details::bytecount<bits>>::type;

}  // namespace blt::memory::utilities
