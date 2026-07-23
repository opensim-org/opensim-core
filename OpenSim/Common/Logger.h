#ifndef OPENSIM_LOG_H_
#define OPENSIM_LOG_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Logger.h                               *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2019 Stanford University and the Authors                *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Common/LogLevel.h>
#include <OpenSim/Common/osimCommonDLL.h>

#include <format>
#include <memory>
#include <string>
#include <string_view>
#include <utility>

namespace OpenSim {

class LogSink;

/// This is a singleton class (single instance) for logging messages and
/// controlling how those messages are presented to the user.
///
/// @note Do not use this class (or any of the free functions) from the
/// destructor of any object with static storage duration. It accesses static
/// objects that are destructed in an undetermined order during static
/// de-initialization.
///
class OSIMCOMMON_API Logger {
public:
    using Level = LogLevel;

    /// This is a static singleton class: there is no way of constructing it.
    Logger() = delete;

#ifndef SWIG
    /// Write `fmt` formatted with `args` at level `logLevel` to the log.
    template<class... Args>
    static void logMessage(LogLevel logLevel, std::format_string<Args...> fmt, Args&&... args)
    {
        if (not shouldLog(logLevel)) {
            return;
        }
        sinkMessage(logLevel, std::format(fmt, std::forward<Args>(args)...));
    }
#endif  // SWIG

    /// Write `message` at level `logLevel` to the log.
    static void logMessage(LogLevel logLevel, std::string_view message)
    {
        logMessage(logLevel, "{}", message);
    }

    /// Log messages of importance `level` and greater.
    /// For example, if the level is set to Info, then Critical, Error, Warn,
    /// and Info messages are logged, while Debug and Trace messages are not
    /// logged.
    static void setLevel(Level level);
    static Level getLevel();

    /// Set the logging level using one of the following strings
    /// (case-insensitive):
    /// - Off
    /// - Critical
    /// - Error
    /// - Warn
    /// - Info
    /// - Debug
    /// - Trace
    /// This variant of setLevel() is for use in Matlab.
    /// @see Level.
    static void setLevelString(std::string level);
    static std::string getLevelString();

    /// Returns true if messages at the provided level should be logged,
    /// based on the set logging level. The following code will produce output:
    /// @code
    /// Log::setLevel(Log::Level::Warn);
    /// if (shouldLog(Log::Level::Error)) {
    ///     std::cout << "Error encountered." << std::endl;
    /// }
    /// @endcode
    static bool shouldLog(Level level);

    /// @name Commands to log messages
    /// @{
#ifndef SWIG
    template <typename... Args>
    static void critical(std::format_string<Args...> fmt, Args&&... args) {
        logMessage(Level::Critical, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    static void error(std::format_string<Args...> fmt, Args&&... args) {
        logMessage(Level::Error, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    static void warn(std::format_string<Args...> fmt, Args&&... args) {
        logMessage(Level::Warn, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    static void info(std::format_string<Args...> fmt, Args&&... args) {
        logMessage(Level::Info, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    static void debug(std::format_string<Args...> fmt, Args&&... args) {
        logMessage(Level::Debug, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    static void trace(std::format_string<Args...> fmt, Args&&... args) {
        logMessage(Level::Trace, fmt, std::forward<Args>(args)...);
    }

    /// Use this function to log messages that would normally be sent to
    /// std::cout. These messages always appear, and are also logged to the
    /// filesink (addFileSink()) and any sinks added via addSink().
    /// The main use case for this function is inside of functions whose intent
    /// is to print information (e.g., Component::printSubcomponentInfo()).
    /// Besides such use cases, this function should be used sparingly to
    /// give users control over what gets logged.
    template <typename... Args>
    static void cout(std::format_string<Args...> fmt, Args&&... args) {
        sinkCoutMessage(std::format(fmt, std::forward<Args>(args)...));
    }

    static void cout(std::string_view msg) {
        cout("{}", msg);
    }
#endif  // SWIG

    /// @}

    /// Log messages to a file at the level getLevel().
    /// OpenSim logs messages to the file opensim.log by default.
    /// If we are already logging messages to a file, then this
    /// function issues a warning and returns; invoke removeFileSink() first.
    /// @note This function is not thread-safe. Do not invoke this function
    /// concurrently, or concurrently with addSink() or removeSink().
    /// @note If filepath can't be opened, no log file is created.
    static void addFileSink(const std::string& filepath = "opensim.log");

    /// Remove the filesink if it exists.
    /// If the filesink was already removed, then this does nothing.
    /// @note This function is not thread-safe. Do not invoke this function
    /// concurrently, or concurrently with addSink() or removeSink().
    static void removeFileSink();

    /// Start reporting messages to the provided sink.
    /// @note This function is not thread-safe. Do not invoke this function
    /// concurrently, or concurrently with addLogFile() or removeSink().
    static void addSink(std::shared_ptr<LogSink> sink);

    /// Remove a sink. If it doesn't exist, do nothing.
    /// @note This function is not thread-safe. Do not invoke this function
    /// concurrently, or concurrently with addLogFile() or addSink().
    static void removeSink(const std::shared_ptr<LogSink>& sink);

private:
    static void sinkMessage(LogLevel level, std::string&& payload);
    static void sinkCoutMessage(std::string&&);
};

/// @name Logging functions
/// @{
#ifndef SWIG
/// @related Logger
template <typename... Args>
void log_critical(std::format_string<Args...> fmt, Args&&... args) {
    Logger::logMessage(LogLevel::Critical, fmt, std::forward<Args>(args)...);
}
/// @related Logger
inline void log_critical(std::string_view msg) {
    Logger::logMessage(LogLevel::Critical, msg);
}

/// @related Logger
template <typename... Args>
void log_error(std::format_string<Args...> fmt, Args&&... args) {
    Logger::logMessage(LogLevel::Error, fmt, std::forward<Args>(args)...);
}
/// @related Logger
inline void log_error(std::string_view msg) {
    Logger::logMessage(LogLevel::Error, msg);
}

/// @related Logger
template <typename... Args>
void log_warn(std::format_string<Args...> fmt, Args&&... args) {
    Logger::logMessage(LogLevel::Warn, fmt, std::forward<Args>(args)...);
}
/// @related Logger
inline void log_warn(std::string_view msg) {
    Logger::logMessage(LogLevel::Warn, msg);
}

/// @related Logger
template <typename... Args>
void log_info(std::format_string<Args...> fmt, Args&&... args) {
    Logger::logMessage(LogLevel::Info, fmt, std::forward<Args>(args)...);
}
/// @related Logger
inline void log_info(std::string_view msg) {
    Logger::logMessage(LogLevel::Info, msg);
}

/// @related Logger
template <typename... Args>
void log_debug(std::format_string<Args...> fmt, Args&&... args) {
    Logger::logMessage(LogLevel::Debug, fmt, std::forward<Args>(args)...);
}
/// @related Logger
inline void log_debug(std::string_view msg) {
    Logger::logMessage(LogLevel::Debug, msg);
}


/// @related Logger
template <typename... Args>
void log_trace(std::format_string<Args...> fmt, Args&&... args) {
    Logger::logMessage(LogLevel::Trace, fmt, std::forward<Args>(args)...);
}
/// @related Logger
inline void log_trace(std::string_view msg) {
    Logger::logMessage(LogLevel::Trace, msg);
}

/// @copydoc Logger::cout()
/// @related Logger
template <typename... Args>
void log_cout(std::format_string<Args...> fmt, Args&&... args) {
    Logger::cout(fmt, std::forward<Args>(args)...);
}
/// @copydoc Logger::cout()
/// @related Logger
inline void log_cout(std::string_view msg) {
    Logger::cout(msg);
}

#endif

} // namespace OpenSim

#endif // OPENSIM_LOG_H_
