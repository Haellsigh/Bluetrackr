#pragma once

#include <blt/bit.hh>

#include <array>
#include <type_traits>

namespace blt::memory {

namespace detail {

template <uint8_t start, uint8_t... Is>
constexpr std::array<uint8_t, sizeof...(Is)> make_range_impl(
    std::integer_sequence<uint8_t, Is...>)
{
  return {{(Is + start)...}};  // generates [start, start + 1, ..., end]
}

template <uint8_t begin, uint8_t end>  //< Expects begin <= end
constexpr auto make_range()
{
  constexpr uint8_t size = end - begin + 1;
  return make_range_impl<begin>(std::make_integer_sequence<uint8_t, size>{});
}

struct pattern {};
struct value {};

}  // namespace detail

/**
 * Defines read only, write only or readwrite access policy for each field of a register.
 */
namespace access_policy {
struct read_only {};
struct write_only {};
struct read_write : read_only, write_only {};
}  // namespace access_policy

template <class AccessPolicyT>
inline constexpr bool can_write =
    std::is_base_of_v<access_policy::write_only, AccessPolicyT>;

template <class AccessPolicyT>
inline constexpr bool can_read =
    std::is_base_of_v<access_policy::read_only, AccessPolicyT>;

// bits::value implementation
template <uint8_t... positions>
struct bits : detail::pattern {
  using mask_t = bit::mask<positions...>;

  template <typename AccessPolicyT>
  static constexpr uint8_t get(uint8_t input) requires can_read<AccessPolicyT>
  {
    return bit::get<mask_t>(input);
  }

  template <typename AccessPolicyT, uint8_t... values>
  static constexpr void apply(uint8_t& output) requires can_write<AccessPolicyT>
  {
    (bit::setclear<mask_t>(output, values), ...);
  }

  template <typename AccessPolicyT>
  static constexpr void apply(uint8_t& output,
                              uint8_t  input) requires can_write<AccessPolicyT>
  {
    bit::setclear<mask_t>(output, input);
  }

  template <typename AccessPolicyT, uint8_t... values>
      requires(sizeof...(values) == sizeof...(positions)) &&
      (std::max({values...}) < 2) struct value : detail::value {
    static constexpr void apply(uint8_t& output) requires can_write<AccessPolicyT>
    {
      bits::apply<AccessPolicyT, values...>(output);
    }
  };
};

// range::value implementation
template <uint8_t begin, uint8_t end>
requires(begin <= end) struct range : detail::pattern {
  using mask                     = bit::mask_range<begin, end>;
  static constexpr uint8_t count = end - begin + 1;

  template <typename AccessPolicyT>
  static constexpr uint8_t get(uint8_t input) requires can_read<AccessPolicyT>
  {
    return bit::get<mask>(input);
  }

  template <typename AccessPolicyT, bool single_value_for_range, uint8_t... values>
  static constexpr void apply(uint8_t& output) requires can_write<AccessPolicyT>
  {
    if constexpr (single_value_for_range) {
      using mask = bit::mask_range<begin, end>;
      bit::setclear<mask>(output, values...);
    } else {
      uint8_t index = 0;
      [&]<uint8_t... positions>(std::integer_sequence<uint8_t, positions...>)
      {
        (bit::setclear<positions...>(output, values, begin, index++), ...);
      }
      (std::make_integer_sequence<uint8_t, count>{});

      // setclear_range<values...>(detail::make_range<begin, end>(), output);
    }
  }

  template <typename AccessPolicyT>
  static constexpr void apply(uint8_t& output,
                              uint8_t  input) requires can_write<AccessPolicyT>
  {
    bit::setclear<mask>(output, input);
  }

  template <typename AccessPolicyT, uint8_t... values>
      requires((sizeof...(values) == count) || (sizeof...(values) == 1)) &&
      (sizeof...(values) == 1
           ? std::max({values...}) < (2 << (end - begin))  // single value < 2^range
           : std::max({values...}) < 2)  // multiple values between 0 and 1
      struct value : detail::value {
    static constexpr bool single_value_for_range = (sizeof...(values) == 1);

    static constexpr void apply(uint8_t& output) requires can_write<AccessPolicyT>
    {
      range::apply<AccessPolicyT, single_value_for_range, values...>(output);
    }
  };
};

template <typename>
struct BitProxy;

/**
 * \todo Implement BitProxy
 *
 * Adds a bit() method in ValueProxy to access (read or write) a single bit of a field
 * (range or bits). Might be complex to implement (indexed access into a parameter pack).
 */
template <typename>
struct BitProxy {};

template <typename>
struct ValueProxy;

template <template <typename, typename> typename FieldT,
          template <uint8_t...>
          typename PatternT,
          typename AccessPolicyT,
          uint8_t... positions>
struct ValueProxy<FieldT<PatternT<positions...>, AccessPolicyT>> {
  static constexpr bool pattern_is_range =
      !std::is_same_v<bits<positions...>, PatternT<positions...>>;

  explicit ValueProxy(uint8_t& value) : value(value) {}

  static constexpr bool write_valid =
      (pattern_is_range || (!pattern_is_range && (sizeof...(positions) == 1)));

  // Write

  inline constexpr void operator=(
      uint8_t input) requires can_write<AccessPolicyT>&& write_valid
  {
    set(input);
  }

  // Read

  constexpr operator int() const requires can_read<AccessPolicyT> { return get(); }
  constexpr operator uint8_t() const requires can_read<AccessPolicyT> { return get(); }
  constexpr operator bool() const requires can_read<AccessPolicyT> { return get(); }

 protected:
  inline constexpr void set(uint8_t input) requires can_write<AccessPolicyT>&& write_valid
  {
    PatternT<positions...>::template apply<AccessPolicyT>(value, input);
  }

  inline constexpr uint8_t get() const requires can_read<AccessPolicyT>
  {
    return PatternT<positions...>::template get<AccessPolicyT>(value);
  }

 protected:
  uint8_t& value;
};

template <typename FieldT>
inline bool operator==(const ValueProxy<FieldT>& lhs, const uint8_t& rhs)
{
  return static_cast<uint8_t>(lhs) == rhs;
}

template <typename FieldT>
inline bool operator==(const uint8_t& rhs, const ValueProxy<FieldT>& lhs)
{
  return lhs == rhs;
}

template <typename FieldT, typename register_type>
concept CompatibleField = std::is_same_v<typename FieldT::register_type, register_type>;

template <uint8_t address>
struct Register {
  using register_type                       = Register<address>;
  static constexpr uint8_t register_address = address;

  Register(const uint8_t& value = 0) : mValue(value) {}

  // Read/write implemented with ValueProxy
  template <CompatibleField<register_type> FieldT>
  constexpr auto field()
  {
    return ValueProxy<FieldT>{mValue};
  }

  // Write implemented from value type
  template <typename ValueT>
  constexpr void write() requires std::is_base_of_v<detail::value, ValueT>
  {
    ValueT::apply(mValue);
  }

  // Write implemented with ValueProxy
  template <typename FieldT>
  constexpr void write(uint8_t input)
  {
    ValueProxy<FieldT>{mValue} = input;
  }

  constexpr uint8_t value() const { return mValue; }
  constexpr void    setValue(uint8_t value) { mValue = value; }

  // Field type
  template <typename PatternT, typename AccessPolicyT = access_policy::read_write>
  struct Field {
    using register_type = Register<address>;

    template <uint8_t... values>
    using value = typename PatternT::template value<AccessPolicyT, values...>;
  };

 protected:
  uint8_t mValue;
};

}  // namespace blt::memory
