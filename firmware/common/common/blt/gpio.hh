#ifndef BLT_LIBS_PIN_H_
#define BLT_LIBS_PIN_H_

#include <blt/hal_include.hh>
#include <blt/time.hh>
#include <blt/utils.hh>

#include <initializer_list>

namespace blt {

namespace gpio {

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

/**
 * Generates functions to access pins
 */
static constexpr uint16_t p0     = 1 << 0;
static constexpr uint16_t p1     = 1 << 1;
static constexpr uint16_t p2     = 1 << 2;
static constexpr uint16_t p3     = 1 << 3;
static constexpr uint16_t p4     = 1 << 4;
static constexpr uint16_t p5     = 1 << 5;
static constexpr uint16_t p6     = 1 << 6;
static constexpr uint16_t p7     = 1 << 7;
static constexpr uint16_t p8     = 1 << 8;
static constexpr uint16_t p9     = 1 << 9;
static constexpr uint16_t p10    = 1 << 10;
static constexpr uint16_t p11    = 1 << 11;
static constexpr uint16_t p12    = 1 << 12;
static constexpr uint16_t p13    = 1 << 13;
static constexpr uint16_t p14    = 1 << 14;
static constexpr uint16_t p15    = 1 << 15;
static constexpr uint16_t pinAll = -1;

}  // namespace

template <GPIO_TypeDef* port(), uint16_t... Pins>
class pin_out {
  static constexpr uint16_t pins = utils::disjunction<Pins...>::value;

 public:
  /**
   * Atomically sets the pins high.
   */
  static inline constexpr void set() { port()->BSRR = pins; }

  /**
   * Atomically clears the pins.
   */
  static inline constexpr void clear() { port()->BRR = pins; }

  /**
   * \warning Invalid results of there is more than one pin.
   * \note This is not an atomic function.
   */
  static inline constexpr bool state() { return (port()->ODR & pins) == 0; }

  /**
   * \note This is an atomic function
   */
  static constexpr void write(bool v) {
    if (v) {
      set();
    } else {
      clear();
    }
  }

  /**
   * \note This is not an atomic function.
   */
  static constexpr void toggle() {
    for (auto&& pin : {Pins...}) {
      port()->ODR ^= pin;
    }
  }
};

template <GPIO_TypeDef* port(), uint16_t pin>
class pin_in {
 public:
  /**
   * \note This is not an atomic function.
   */
  static constexpr bool read() {
    return (port()->IDR & pin) != (uint32_t)GPIO_PIN_RESET;
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
 public:
  static constexpr inline void write(bool v) {
    pin::write(v);
    delay::us(us);
  }

  static constexpr inline void set() {
    pin::set();
    delay::us(us);
  }

  static constexpr inline void clear() {
    pin::clear();
    delay::us(us);
  }

  static constexpr inline void toggle() {
    pin::toggle();
    delay::us(us);
  }
};

}  // namespace gpio

}  // namespace blt

#endif  // BLT_PIN_H_
