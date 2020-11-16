#pragma once

#include "utilities.hh"

namespace blt::memory2 {

/**
 * \brief Access policies for registers and register fields
 */
namespace access_policy {

/**
 * \brief Read write access policy
 */
struct rw {
  static constexpr bool can_read  = true;
  static constexpr bool can_write = true;
};

/**
 * \brief Read only access policy
 */
struct ro {
  static constexpr bool can_read  = true;
  static constexpr bool can_write = false;
};

/**
 * \brief Write only access policy
 */
struct wo {
  static constexpr bool can_read  = false;
  static constexpr bool can_write = true;
};

template <typename T>
concept type = T::can_read || T::can_write;

template <type access_policy_t>
inline constexpr bool can_write = access_policy_t::can_write;

template <type access_policy_t>
inline constexpr bool can_read = access_policy_t::can_read;

template <typename secondary_ap_t, typename primary_ap_t>
concept valid_subset = !(!primary_ap_t::can_read && secondary_ap_t::can_read) &&
                       !(!primary_ap_t::can_write && secondary_ap_t::can_write);

}  // namespace access_policy

struct reg_base {};
template <typename type>
concept register_concept = std::is_base_of_v<reg_base, type>;
struct field_base {};
template <typename type>
concept field_concept = std::is_base_of_v<field_base, type>;

/**
 * \brief Field base type
 *
 * \tparam register_type The type of the parent register, deduced automatically
 * \tparam field_offset The offset of the field in the register
 * \tparam field_size The size in bits of the field (defaults to 1)
 * \tparam field_ap The access policy of the field. Must be compatible with the register's access
 * policy. (defaults to parent register's access policy)
 * \tparam value_type The underlying storage type of the field (defaults to the smallest unsigned
 * integer type that can hold field_size bits)
 */
template <register_concept    register_type,
          uint16_t            field_offset,
          uint8_t             field_size,
          access_policy::type field_ap,
          typename value_type = utilities::smallestint_t<field_size>>
struct field : public field_base {
  using register_t                 = register_type;
  static constexpr uint16_t offset = field_offset;
  static constexpr uint8_t  size   = field_size;
  using ap                         = field_ap;
  using value_t                    = value_type;
};

template <typename field_t, typename register_t = typename field_t::register_t>
constexpr bool valid_field = (field_t::offset + field_t::size <= register_t::size) &&
                             (field_t::size > 0);

template <typename field_t, typename register_t>
concept compatible_field = std::is_same_v<register_t, typename field_t::register_t>&&
        access_policy::valid_subset<typename field_t::ap, typename register_t::ap>&&
        valid_field<field_t, register_t>;

/**
 * \brief Register base type
 *
 * \tparam register_address The address of the register in memory
 * \tparam register_size The size in bits of the register (defaults to 8)
 * \tparam register_ap The access policy of the register (defaults to read/write)
 */
template <uint16_t            register_address,
          uint8_t             register_size = 8,
          access_policy::type register_ap   = access_policy::rw>
struct reg : public reg_base {
  static constexpr uint8_t size       = register_size;
  static constexpr uint8_t size_bytes = utilities::bytecount<size>;
  static constexpr uint8_t address    = register_address;
  using ap                            = register_ap;
  using register_t                    = reg<address, size, ap>;
  using value_t                       = utilities::smallestint_t<size>;

  reg(value_t value = 0) : m_value(value) {}

  template <uint16_t                        field_offset,
            uint8_t                         field_size = 1,
            access_policy::valid_subset<ap> field_ap   = ap,
            typename value_type                        = utilities::smallestint_t<field_size>>
  requires(valid_field<field<reg, field_offset, field_size, field_ap, value_type>>) using field =
      field<reg, field_offset, field_size, field_ap, value_type>;

  value_t get() const { return m_value; }

  template <std::size_t i>
  requires(i < size_bytes) uint8_t getByte() const
  {
    return ((m_value >> (i * 8)) & 0xFF);
  }
  uint8_t getByte(std::size_t i) const { return ((m_value >> (i * 8)) & 0xFF); }

  template <compatible_field<register_t> field_t>
  requires field_t::ap::can_read field_t::value_t get() const
  {
    return (m_value >> field_t::offset) & ~(((value_t{1} << size) - 1) << field_t::size);
  }

  void set(value_t v) { m_value = v; }

  template <compatible_field<register_t> field_t>
  requires field_t::ap::can_write void set(typename field_t::value_t v)
  {
    constexpr auto mask = (((value_t{1} << field_t::size) - 1) << field_t::offset);
    m_value             = (m_value & (~mask)) | (mask & (value_t{v} << field_t::offset));
  }

 private:
  value_t m_value;
};

}  // namespace blt::memory2
