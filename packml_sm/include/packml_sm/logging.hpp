// Copyright (c) 2026 PackML ROS2 Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ---
// Lightweight logging bridge for packml_sm.
//
// The state-machine library is intentionally framework-agnostic.  All
// diagnostic output is routed through the bridge declared here.  By default
// messages are written to std::cout / std::cerr.  Integrators (e.g. a ROS
// wrapper) can install a custom sink with `Logging::set_sink(...)` to forward
// messages to their preferred logger.
// ---

#ifndef PACKML_SM__LOGGING_HPP_
#define PACKML_SM__LOGGING_HPP_

#include <cstdarg>
#include <cstdio>
#include <functional>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>

namespace packml_sm
{

enum class LogLevel
{
  DEBUG = 0,
  INFO  = 1,
  WARN  = 2,
  ERROR = 3
};

inline const char * to_cstr(LogLevel level) noexcept
{
  switch (level) {
    case LogLevel::DEBUG: return "DEBUG";
    case LogLevel::INFO:  return "INFO ";
    case LogLevel::WARN:  return "WARN ";
    case LogLevel::ERROR: return "ERROR";
  }
  return "?    ";
}

using LogSink = std::function<void(LogLevel, const std::string & logger_name, const std::string & message)>;

class Logging
{
public:
  static Logging & instance() noexcept
  {
    static Logging inst;
    return inst;
  }

  void set_sink(LogSink sink)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sink_ = std::move(sink);
  }

  void clear_sink()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sink_ = LogSink{};
  }

  void set_level(LogLevel lvl) noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    min_level_ = lvl;
  }

  LogLevel level() const noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return min_level_;
  }

  bool enabled(LogLevel level) const noexcept
  {
    return static_cast<int>(level) >= static_cast<int>(this->level());
  }

  void log(LogLevel level, const std::string & logger_name, const std::string & message)
  {
    LogSink sink_copy;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (static_cast<int>(level) < static_cast<int>(min_level_)) {
        return;
      }
      sink_copy = sink_;
    }
    if (sink_copy) {
      sink_copy(level, logger_name, message);
    } else {
      default_sink(level, logger_name, message);
    }
  }

  static void default_sink(LogLevel level, const std::string & logger_name, const std::string & message)
  {
    std::ostream & os =
      (level == LogLevel::ERROR || level == LogLevel::WARN) ? std::cerr : std::cout;
    os << "[" << to_cstr(level) << "] [" << logger_name << "] " << message << '\n';
  }

private:
  Logging() = default;
  mutable std::mutex mutex_;
  LogSink sink_{};
  LogLevel min_level_{LogLevel::INFO};
};

}  // namespace packml_sm

// ---------------------------------------------------------------------------
// Stream-style macros (RCLCPP_*_STREAM lookalikes)
// ---------------------------------------------------------------------------

#define PACKML_LOG_STREAM(LEVEL, NAME, EXPR)                                                   \
  do {                                                                                         \
    if (::packml_sm::Logging::instance().enabled(LEVEL)) {                                     \
      std::ostringstream _packml_log_oss;                                                      \
      _packml_log_oss << EXPR;                                                                 \
      ::packml_sm::Logging::instance().log((LEVEL), (NAME), _packml_log_oss.str());            \
    }                                                                                          \
  } while (0)

#define PACKML_DEBUG_STREAM(NAME, EXPR) PACKML_LOG_STREAM(::packml_sm::LogLevel::DEBUG, NAME, EXPR)
#define PACKML_INFO_STREAM(NAME, EXPR)  PACKML_LOG_STREAM(::packml_sm::LogLevel::INFO,  NAME, EXPR)
#define PACKML_WARN_STREAM(NAME, EXPR)  PACKML_LOG_STREAM(::packml_sm::LogLevel::WARN,  NAME, EXPR)
#define PACKML_ERROR_STREAM(NAME, EXPR) PACKML_LOG_STREAM(::packml_sm::LogLevel::ERROR, NAME, EXPR)

// ---------------------------------------------------------------------------
// printf-style macros (RCLCPP_* lookalikes)
// ---------------------------------------------------------------------------

namespace packml_sm
{
namespace detail
{
inline std::string format_message(const char * fmt, ...)
{
  va_list args;
  va_start(args, fmt);
  va_list args_copy;
  va_copy(args_copy, args);
  const int needed = std::vsnprintf(nullptr, 0, fmt, args_copy);
  va_end(args_copy);

  std::string out;
  if (needed > 0) {
    out.resize(static_cast<std::size_t>(needed));
    std::vsnprintf(out.data(), static_cast<std::size_t>(needed) + 1, fmt, args);
  }
  va_end(args);
  return out;
}
}  // namespace detail
}  // namespace packml_sm

#define PACKML_LOG_FMT(LEVEL, NAME, ...)                                                        \
  do {                                                                                          \
    if (::packml_sm::Logging::instance().enabled(LEVEL)) {                                      \
      ::packml_sm::Logging::instance().log(                                                     \
        (LEVEL), (NAME), ::packml_sm::detail::format_message(__VA_ARGS__));                     \
    }                                                                                           \
  } while (0)

#define PACKML_DEBUG(NAME, ...) PACKML_LOG_FMT(::packml_sm::LogLevel::DEBUG, NAME, __VA_ARGS__)
#define PACKML_INFO(NAME, ...)  PACKML_LOG_FMT(::packml_sm::LogLevel::INFO,  NAME, __VA_ARGS__)
#define PACKML_WARN(NAME, ...)  PACKML_LOG_FMT(::packml_sm::LogLevel::WARN,  NAME, __VA_ARGS__)
#define PACKML_ERROR(NAME, ...) PACKML_LOG_FMT(::packml_sm::LogLevel::ERROR, NAME, __VA_ARGS__)

#endif  // PACKML_SM__LOGGING_HPP_
