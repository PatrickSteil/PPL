#pragma once

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>

#include "timer.h"

// Helper function to get the current local time as a string.
std::string get_current_time_string() {
  auto now = std::chrono::system_clock::now();
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm *localTime = std::localtime(&now_time);
  char buffer[100];
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", localTime);
  return std::string(buffer);
}

class CSVStatusLog {
 public:
  // Nested proxy class for metric assignment.
  class MetricProxy {
    CSVStatusLog &log;
    std::string key;

   public:
    MetricProxy(CSVStatusLog &logObj, const std::string &k)
        : log(logObj), key(k) {}

    // Template operator= to assign any type that supports stream output.
    template <typename T>
    MetricProxy &operator=(const T &value) {
      std::ostringstream oss;
      oss << value;
      log.metrics[key] = oss.str();
      return *this;
    }

    // Overload for assignment from char arrays.
    MetricProxy &operator=(const char *value) {
      log.metrics[key] = value;
      return *this;
    }
  };

  // Overload operator[] to return a MetricProxy.
  MetricProxy operator[](const std::string &key) {
    return MetricProxy(*this, key);
  }

  // Disallow default construction and copy semantics.
  CSVStatusLog() = delete;
  CSVStatusLog(const CSVStatusLog &) = delete;
  CSVStatusLog &operator=(const CSVStatusLog &) = delete;

  // Static members for handling logging output.
  static std::ofstream logFile;
  static bool logToFile;

  // Constructor: stores the message and starts the timer.
  CSVStatusLog(const std::string &msg) : message(msg) {
    std::cout << msg << " ... " << std::flush;
    startTime = -get_micro_time();
  }
  CSVStatusLog(const char *msg) : message(msg) {
    std::cout << msg << " ... " << std::flush;
    startTime = -get_micro_time();
  }

  // Destructor: computes elapsed time, appends any metrics, and writes the
  // result.
  ~CSVStatusLog() {
    long long elapsed = startTime + get_micro_time();
    std::cout << "done [" << elapsed / 1000 << "ms]" << std::endl;

    // Build the CSV log entry.
    std::ostringstream oss;
    oss << get_current_time_string() << ",\"" << message << "\","
        << elapsed / 1000 << ",";

    // Combine metrics as key:value pairs separated by semicolons.
    if (!metrics.empty()) {
      bool first = true;
      for (const auto &entry : metrics) {
        if (!first) {
          oss << "; ";
        }
        first = false;
        oss << entry.first << ": " << entry.second;
      }
    }

    // Write the log entry either to the file or cout.
    if (logToFile && logFile.is_open()) {
      logFile << oss.str() << std::endl;
      logFile.flush();
    } else {
      std::cout << oss.str() << std::endl;
    }
  }

 private:
  // Private member variables.
  long long startTime;
  std::string message;
  std::map<std::string, std::string> metrics;

 public:
  // Call this once at program startup. If filename is empty, logging will be to
  // std::cout.
  static void init(const std::string &filename) {
    if (filename.empty()) {
      logToFile = false;
    } else {
      logToFile = true;
      logFile.open(filename, std::ios::app);
      if (!logFile.is_open()) {
        std::cerr << "Error: Could not open log file " << filename << std::endl;
        logToFile = false;
      } else {
        // Write CSV header.
        logFile << "Timestamp,Message,ExecutionTime(ms),Metrics" << std::endl;
      }
    }
  }
};

// Definition of static members.
std::ofstream CSVStatusLog::logFile;
bool CSVStatusLog::logToFile = false;
