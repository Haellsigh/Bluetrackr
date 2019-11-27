#ifndef BLT_UTILS_H
#define BLT_UTILS_H

namespace blt {

namespace utils {

class noncopyable {
 public:
  constexpr noncopyable() = default;
  noncopyable(const noncopyable&) = delete;
  noncopyable& operator=(const noncopyable&) = delete;
  // ~noncopyable() {} // This apparently uses the heap.
};

template <uint16_t... values>
struct disjunction;

template <uint16_t first, uint16_t... others>
struct disjunction<first, others...> {
  // static_assert(first < 32, "maximum 16 bits can be used");
  static const uint16_t value = first | disjunction<others...>::value;
};

template <>
struct disjunction<> {
  static const uint16_t value = 0;
};

}  // namespace utils
}  // namespace blt

#endif  // BLT_UTILS_H
