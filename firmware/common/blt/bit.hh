#ifndef BLT_BIT_HH_
#define BLT_BIT_HH_

#include <cstdint>

namespace blt::bit {

/**
 * \brief Returns a bit mask for the bit(s).
 * Thank god for C++17.
 */
template <uint8_t... bits>
static constexpr uint8_t maskBits() {
  return ((1 << bits) | ... | 0);
}

/**
 * \brief Returns a bit mask for the bits in range [l, h].
 */
template <uint8_t first, uint8_t last>
static constexpr uint8_t maskRange() {
  if constexpr (first > last)
    return (((1 << last) - 1) ^ ((1 << (first + 1)) - 1));
  else
    return (((1 << first) - 1) ^ ((1 << (last + 1)) - 1));
}

/*!
 * \brief Shifts in by n bits.
 */
template <uint8_t n, typename Tout, typename Tin>
static constexpr Tout shift(Tin in = 1) {
  return static_cast<Tout>(in) << n;
}

/*!
 * \brief Shifts in by n bits.
 * \note Runtime version.
 */
template <typename Tin, typename Tout>
static constexpr Tout shift(Tin in, Tin n) {
  return static_cast<Tout>(in) << n;
}

/*!
 * \brief Sets the bits from the input in.
 */
template <uint8_t... bits>
static constexpr void set(uint8_t& in) {
  in |= maskBits<bits...>();
}

/*!
 * \brief Sets the bit n from the input in.
 */
template <uint8_t n, typename Tin>
static constexpr void set(Tin& in) {
  in |= shift<n, Tin>(1);
}

/*!
 * \brief Gets the bits from the input in.
 */
template <uint8_t... bits>
static constexpr uint8_t get(uint8_t& in) {
  return in & maskBits<bits...>();
}

/*!
 * \brief Gets the bit n from the input in.
 */
template <uint8_t n, typename Tin>
static constexpr uint8_t get(Tin& in) {
  return in & maskBits<n>();
}

/*!
 * \brief Sets the bit n from the input in.
 * \note Runtime version.
 */
template <typename Tin>
static constexpr void set(Tin& in, Tin n) {
  in |= shift<Tin, Tin>(1, n);
}

/*!
 * \brief Clears the bit n from the input in.
 */
template <uint8_t n, typename Tin>
static constexpr void clear(Tin& in) {
  in &= ~shift<n, Tin>(1);
}

/*!
 * \brief Clears the bit n from the input in.
 * \note Runtime version.
 */
template <typename Tin>
static constexpr void clear(Tin& in, Tin n) {
  in &= ~shift<Tin, Tin>(1, n);
}

}  // namespace blt::bit

#endif  // BLT_BIT_HH_
