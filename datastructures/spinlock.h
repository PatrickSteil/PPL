#pragma once

#include <time.h>
#include <atomic>

// Spinlock implementation (from Fedor Pikus)
class Spinlock {
 public:
  Spinlock() : flag_(0) {}

  void lock() {
    static const timespec ns = {0, 1};
    for (int i = 0; flag_.load(std::memory_order_relaxed) ||
                    flag_.exchange(1, std::memory_order_acquire);
         ++i) {
      if (i == 8) {
        i = 0;
        nanosleep(&ns, nullptr);
      }
    }
  }

  bool try_lock() noexcept {
    return flag_.exchange(1, std::memory_order_acquire) == 0;
  }

  void unlock() { flag_.store(0, std::memory_order_release); }

 private:
  std::atomic<unsigned int> flag_;
};

// RAII wrapper for the spinlock (similar to std::lock_guard)
class SpinlockGuard {
 public:
  explicit SpinlockGuard(Spinlock& lock) : lock_(lock) { lock_.lock(); }
  ~SpinlockGuard() { lock_.unlock(); }
  SpinlockGuard(const SpinlockGuard&) = delete;
  SpinlockGuard& operator=(const SpinlockGuard&) = delete;

 private:
  Spinlock& lock_;
};