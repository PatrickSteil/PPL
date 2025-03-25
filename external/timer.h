/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#pragma once

#include <sys/time.h>

class Timer {
 public:
  Timer() : firstTime(0), secondTime(0) {}

  inline void start() noexcept { firstTime = timestamp(); }

  inline void restart() noexcept { firstTime = timestamp(); }

  inline void stop() noexcept { secondTime = timestamp(); }

  inline void reset() noexcept {
    firstTime = 0;
    secondTime = 0;
  }

  inline double elapsedMicroseconds() const noexcept {
    return secondTime - firstTime;
  }

  inline double elapsedMilliseconds() const noexcept {
    return (secondTime - firstTime) / 1000.0;
  }

 private:
  inline static double timestamp() noexcept {
    timeval tp;
    gettimeofday(&tp, nullptr);
    double mus = static_cast<double>(tp.tv_usec);
    return (tp.tv_sec * 1000000.0) + mus;
  }

 private:
  double firstTime, secondTime;
};

// Taken from Ben Strasser

inline long long get_micro_time() {
  timeval t;
  gettimeofday(&t, 0);
  return t.tv_sec * 1000000ll + t.tv_usec;
}