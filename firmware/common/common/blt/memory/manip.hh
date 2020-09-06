#include "utilities.hh"

namespace blt::memory {

/*!
 * Access policies for registers and register fields
 */
namespace access_policy {

struct rw {
  static constexpr bool can_read  = true;
  static constexpr bool can_write = true;
};

struct ro {
  static constexpr bool can_read  = true;
  static constexpr bool can_write = false;
};

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

template <register_concept    register_type,
          uint16_t            field_offset,
          uint8_t             field_size,
          access_policy::type field_ap>
struct field : public field_base {
  using register_t                 = register_type;
  static constexpr uint16_t offset = field_offset;
  static constexpr uint8_t  size   = field_size;
  using ap                         = field_ap;
  using value_t                    = utilities::smallestint_t<field_size>;
};

template <typename field_t, typename register_t = typename field_t::register_t>
constexpr bool valid_field = (field_t::offset + field_t::size <= register_t::size) &&
                             (field_t::size > 0);

template <typename field_t, typename register_t>
concept compatible_field = std::is_same_v<register_t, typename field_t::register_t>&&
        access_policy::valid_subset<typename field_t::ap, typename register_t::ap>&&
        valid_field<field_t, register_t>;

template <uint16_t            register_address,
          uint8_t             register_size = 8,
          access_policy::type register_ap   = access_policy::rw>
struct reg : public reg_base {
  static constexpr uint8_t size    = register_size;
  static constexpr uint8_t address = register_address;
  using ap                         = register_ap;
  using register_t                 = reg<address, size, ap>;
  using value_t                    = utilities::smallestint_t<size>;

  reg(value_t value = 0) : m_value(value) {}

  template <uint16_t                        field_offset,
            uint8_t                         field_size = 1,
            access_policy::valid_subset<ap> field_ap   = ap>
  requires(valid_field<field<reg, field_offset, field_size, field_ap>>) using field =
      field<reg, field_offset, field_size, field_ap>;

  auto get() const { return m_value; }

  template <compatible_field<register_t> field_t>
  requires field_t::ap::can_read auto get() const
  {
    return (m_value >> field_t::offset) & ~(((1 << size) - 1) << field_t::size);
  }

  void set(value_t v) { m_value = v; }

  template <compatible_field<register_t> field_t>
  requires field_t::ap::can_write void set(typename field_t::value_t v)
  {
    constexpr auto mask = (((1 << field_t::size) - 1) << field_t::offset);
    m_value             = (m_value & (~mask)) | (mask & (v << field_t::offset));
  }

 private:
  value_t m_value;
};

}  // namespace blt::memory
