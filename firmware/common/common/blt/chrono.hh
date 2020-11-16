#pragma once

#include <blt/hal_include.hh>

#include <array>
#include <chrono>
#include <cstdint>

namespace blt::chrono {

struct clock {
  using rep                       = std::int32_t;
  using period                    = std::micro;
  using duration                  = std::chrono::duration<rep, period>;
  using time_point                = std::chrono::time_point<clock>;
  static constexpr bool is_steady = true;

  using now_function_type = clock::time_point (*)() noexcept;
  static inline now_function_type now;
};

using fsec = std::chrono::duration<float>;

namespace literals {
using namespace std::chrono_literals;

constexpr clock::duration operator"" _hz(uint64_t hz)
{
  return clock::duration{clock::period::den / hz};
}
}  // namespace literals

void init(clock::now_function_type now_function)
{
  clock::now = now_function;
}

void delay(clock::duration const& duration)
{
  const auto start = clock::now();
  while (clock::now() - start < duration)
    ;
}

template <typename Callable>
struct periodic_task {
  periodic_task(clock::duration period, Callable&& task) noexcept
      : period_{period}, last_{clock::now()}, task_{std::forward<Callable>(task)}
  {}

  template <typename... Args>
  void update(Args&&... args)
  {
    const auto now = clock::now();
    const auto dt  = now - last_;
    if (dt >= period_) {
      last_ = now;
      task_(dt, std::forward<Args>(args)...);
    }
  }

 private:
  clock::duration   period_;
  clock::time_point last_;
  Callable          task_;
};

struct stopwatch {
  stopwatch() : start_(clock::now()) {}

  clock::duration restart()
  {
    const auto now = clock::now();
    return now - std::exchange(start_, now);
  }

  clock::duration elapsed() { return clock::now() - start_; }

 private:
  clock::time_point start_;
};

}  // namespace blt::chrono