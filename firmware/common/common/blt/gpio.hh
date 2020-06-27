#pragma once

#include <blt/hal_include.hh>
#include <blt/time.hh>
#include <blt/utils.hh>

#include <initializer_list>

namespace blt::gpio {

namespace {

/**
 * Generates functions to get the GPIO ports.
 * Ports are named PA, ..., PF.
 */
#define PORT_HANDLE_FUNCTION(_NAME) \
  constexpr GPIO_TypeDef* P##_NAME() { return GPIO##_NAME; }

#ifdef GPIOA
PORT_HANDLE_FUNCTION(A)
#endif

#ifdef GPIOB
PORT_HANDLE_FUNCTION(B)
#endif

#ifdef GPIOC
PORT_HANDLE_FUNCTION(C)
#endif

#ifdef GPIOD
PORT_HANDLE_FUNCTION(D)
#endif

#ifdef GPIOE
PORT_HANDLE_FUNCTION(E)
#endif

#ifdef GPIOF
PORT_HANDLE_FUNCTION(F)
#endif

#ifdef GPIOG
PORT_HANDLE_FUNCTION(G)
#endif

}  // namespace

template <GPIO_TypeDef* port(), uint16_t... pins>
class pin_out {
  static constexpr uint16_t pins_flag = utils::disjunction_flag<pins...>();

 public:
  /**
   * Atomically sets the pins high.
   */
  static inline constexpr void set() { port()->BSRR = pins_flag; }

  /**
   * Atomically clears the pins.
   */
  static inline constexpr void clear() { port()->BRR = pins_flag; }

  /**
   * \warning Invalid results if there is more than one pin.
   * \note This is not an atomic function.
   */
  static inline constexpr bool state() { return (port()->ODR & pins_flag) == 0; }

  /**
   * \note This is an atomic function
   */
  static constexpr void write(bool v)
  {
    if (v) {
      set();
    } else {
      clear();
    }
  }

  /**
   * \note This is not an atomic function.
   */
  static constexpr void toggle() { ((port()->ODR ^= 1 << pins), ...); }
};

template <GPIO_TypeDef* port(), uint16_t pin>
class pin_in {
  static constexpr uint16_t pin_flag = utils::disjunction_flag<pin>();

 public:
  /**
   * \note This is not an atomic function.
   */
  static constexpr bool read()
  {
    return (port()->IDR & pin_flag) != (uint32_t)GPIO_PIN_RESET;
  }
};

/*
template <GPIO_TypeDef* p(), uint16_t pin>
class pin_out_inverted : public pin_out<p, pin> {
 public:
  static constexpr void write(bool v) override { write(!v); }
};

template <GPIO_TypeDef* p(), uint16_t pin>
class pin_in_inverted : public pin_in<p, pin> {
 public:
  static constexpr bool read() override { return !read(); }
};
*/

template <typename pin>
class invert {
 public:
  static constexpr inline void write(bool v) { pin::write(!v); }
  static constexpr inline void set() { pin::clear(); }
  static constexpr inline void clear() { pin::set(); }
  static constexpr inline bool read() { return !pin::read(); }
};

/**
 * \brief Settling time for pin_out.
 *
 * \tparam pin The pin to settle.
 * \tparam us  The settling time in microseconds.
 */
template <typename pin, uint32_t us>
class settle {
  static constexpr time::Microseconds mTime{us};

 public:
  static constexpr inline void write(bool v)
  {
    pin::write(v);
    time::delay(mTime);
  }

  static constexpr inline void set()
  {
    pin::set();
    time::delay(mTime);
  }

  static constexpr inline void clear()
  {
    pin::clear();
    time::delay(mTime);
  }

  static constexpr inline void toggle()
  {
    pin::toggle();
    time::delay(mTime);
  }
};

}  // namespace blt::gpio