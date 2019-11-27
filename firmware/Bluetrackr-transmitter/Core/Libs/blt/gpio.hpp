#ifndef BLT_LIBS_PIN_H_
#define BLT_LIBS_PIN_H_

#include <initializer_list>

#include <main.h>

#include "delay.hpp"
#include "utils.hpp"

namespace blt {

namespace gpio {

namespace {

/**
 * 'Automatically' generates functions to get the GPIO ports.
 */
#define PORT_HANDLE_FUNCTION(_NAME) \
  constexpr GPIO_TypeDef* Port##_NAME() { return GPIO##_NAME; }

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
constexpr uint16_t pin0 = 1 << 0;
constexpr uint16_t pin1 = 1 << 1;
constexpr uint16_t pin2 = 1 << 2;
constexpr uint16_t pin3 = 1 << 3;
constexpr uint16_t pin4 = 1 << 4;
constexpr uint16_t pin5 = 1 << 5;
constexpr uint16_t pin6 = 1 << 6;
constexpr uint16_t pin7 = 1 << 7;
constexpr uint16_t pin8 = 1 << 8;
constexpr uint16_t pin9 = 1 << 9;
constexpr uint16_t pin10 = 1 << 10;
constexpr uint16_t pin11 = 1 << 11;
constexpr uint16_t pin12 = 1 << 12;
constexpr uint16_t pin13 = 1 << 13;
constexpr uint16_t pin14 = 1 << 14;
constexpr uint16_t pin15 = 1 << 15;
constexpr uint16_t pinAll = -1;

}  // namespace

template <GPIO_TypeDef* port(), uint16_t... Pins>
class pin_out {
  static constexpr uint16_t pins = utils::disjunction<Pins...>::value;

 public:
  /**
   * Atomically sets the pins.
   */
  static inline constexpr void on() { port()->BSRR = pins; }

  /**
   * Atomically clears the pins.
   */
  static inline constexpr void off() { port()->BRR = pins; }

  /**
   * \note Invalid results of there is more than one pin.
   */
  static inline constexpr bool state() { return (port()->ODR & pins) == 0; }

  /**
   * \note This is an atomic function
   */
  static constexpr void write(bool v) {
    if (v) {
      on();
    } else {
      off();
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

  static constexpr inline void toggle() {
    pin::toggle();
    delay::us(us);
  }
};

}  // namespace gpio

}  // namespace blt

#endif  // BLT_PIN_H_
