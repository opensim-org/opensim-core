/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Logger.cpp                             *
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

#include "Logger.h"

#include <OpenSim/Common/Exception.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/LogLevel.h>
#include <OpenSim/Common/LogMessage.h>
#include <OpenSim/Common/LogSink.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <concepts>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

using namespace OpenSim;

namespace
{
    // Used to ensure file sink auto-initialization happens once with no races.
    constinit std::once_flag g_FileSinkAutoInitOnceFlag;

    // Used when mutating `g_FileSink`.
    constinit std::mutex g_FileSinkMutex;

    // Initialized at runtime.
    class BasicFileSink;
    constinit std::shared_ptr<BasicFileSink> g_FileSink;

    /// Returns a human-readable label for `level` (matches `LogLevel`).
    std::string_view labelFor(LogLevel level) {
        switch (level) {
        case LogLevel::Off:      return "Off";
        case LogLevel::Critical: return "Critical";
        case LogLevel::Error:    return "Error";
        case LogLevel::Warn:     return "Warn";
        case LogLevel::Info:     return "Info";
        case LogLevel::Debug:    return "Debug";
        case LogLevel::Trace:    return "Trace";
        default:
            OPENSIM_THROW(Exception, "Internal error.");
        }
    }

    /// Returns a log-visible label for `level` (matches spdlog legacy labels).
    std::string_view loggingLabelFor(LogLevel level) {
        switch (level) {
        case LogLevel::Off:      return "off";
        case LogLevel::Critical: return "critical";
        case LogLevel::Error:    return "error";
        case LogLevel::Warn:     return "warn";
        case LogLevel::Info:     return "info";
        case LogLevel::Debug:    return "debug";
        case LogLevel::Trace:    return "trace";
        default:
            OPENSIM_THROW(Exception, "Internal error.");
        }
    }

    /// Parses `str` into a `LogLevel`. Throws if it cannot be parsed.
    LogLevel parseLogLevel(std::string str) {
        str = IO::Lowercase(str);
        if (str == "off")      { return LogLevel::Off;      }
        if (str == "critical") { return LogLevel::Critical; }
        if (str == "error")    { return LogLevel::Error;    }
        if (str == "warn")     { return LogLevel::Warn;     }
        if (str == "info")     { return LogLevel::Info;     }
        if (str == "debug")    { return LogLevel::Debug;    }
        if (str == "trace")    { return LogLevel::Trace;    }

        OPENSIM_THROW(Exception, "Expected log level to be Off, Critical, Error, Warn, Info, Debug, or Trace; got {}.", str);
    }

    /// Formats a `LogMessage` similarly to `spdlog`'s default pattern (legacy behavior)
    std::string formatLikeSpdlog(const LogMessage& msg) {
        namespace chrono = std::chrono;

        static_assert(std::same_as<LogMessage::time_point_type, chrono::system_clock::time_point>);
        const auto t = msg.getTime();
        const auto seconds = chrono::floor<chrono::seconds>(t);
        const auto millis = chrono::duration_cast<chrono::milliseconds>(t - seconds);

        return std::format("[{:%Y-%m-%d %H:%M:%S}.{:03d}] [{}] [{}] {}\n",
            seconds,
            millis.count(),
            msg.getLoggerName(),
            loggingLabelFor(msg.getLevel()),
            msg.getPayload()
        );
    }

    /// A `LogSink` that writes log messages to `std::cout`.
    class CoutSink final : public LogSink {
        void sinkImpl(const LogMessage& msg) override {
            std::cout << msg.getPayload() << '\n';
        }

        void flushImpl() override {
            std::cout.flush();
        }
    };

    /// A "default" `LogSink`: format matches OpenSim's legacy (spdlog) format.
    class DefaultLogSink final : public LogSink {
        void sinkImpl(const LogMessage& msg) override {
            std::cout << '[' << loggingLabelFor(msg.getLevel()) << "] " << msg.getPayload() << '\n';
        }

        void flushImpl() override {
            std::cout.flush();
        }
    };

    /// A basic file `LogSink` that writes messages to the provided path.
    class BasicFileSink final : public LogSink {
    public:
        explicit BasicFileSink(std::filesystem::path path) :
            path_{std::move(path)}
        {
            if (!out_.is_open() || out_.fail()) {
                OPENSIM_THROW(Exception, "Failed to open log file at '{}'", path_.string());
            }
        }

        std::filesystem::path filename() const { return path_.filename(); }
    private:
        void sinkImpl(const LogMessage& msg) override {
            out_ << formatLikeSpdlog(msg);
        }
        void flushImpl() override { out_.flush(); }

        std::filesystem::path path_;
        std::ofstream out_{path_};
    };

    /// Private implementation data for a logger.
    class LoggerImpl final {
    public:
        template<typename T, typename... Args>
        requires std::constructible_from<T, Args&&...>
        static LoggerImpl with_sink(Args&&... args) {
            return LoggerImpl{std::make_shared<T>(std::forward<Args>(args)...)};
        }

        LogLevel getLevel() const { return level_; }
        void setLevel(LogLevel level) { level_ = level; }

        bool shouldLog(LogLevel level) const {
            const LogLevel cur = level_.load(std::memory_order_relaxed);
            return level != LogLevel::Off and cur != LogLevel::Off and level >= cur;
        }

        void addSink(std::shared_ptr<LogSink> sink) {
            std::lock_guard guard{mutex_};
            sinks_.push_back(std::move(sink));
        }

        void removeSink(const std::shared_ptr<LogSink>& sink) {
            std::lock_guard guard{mutex_};
            std::erase(sinks_, sink);
        }

        void sinkMessage(const LogMessage& msg) {
            if (not shouldLog(msg.getLevel())) {
                return;
            }

            std::lock_guard guard{mutex_};

            ++messagesSinceLastFlush;

            const bool isHighSeverity =
                msg.getLevel() >= LogLevel::Warn;

            const bool shouldFlush =
                isHighSeverity or
                (messagesSinceLastFlush >= c_FlushInterval);

            for (auto& sink : sinks_) {
                if (sink->shouldLog(msg.getLevel())) {
                    sink->sink(msg);
                    if (shouldFlush) {
                        sink->flush();
                    }
                }
            }

            if (shouldFlush) {
                messagesSinceLastFlush = 0;
            }
        }

        template<typename... Args>
        requires std::constructible_from<LogMessage, std::string&&, Args&&...>
        void sinkMessage(Args&&... args) {
            sinkMessage(LogMessage{std::string{"OpenSim"}, std::forward<Args>(args)...});
        }
    private:
        explicit LoggerImpl(std::shared_ptr<LogSink> sink) :
            sinks_{std::move(sink)}
        {}

        std::mutex mutex_;
        std::vector<std::shared_ptr<LogSink>> sinks_;
        std::atomic<LogLevel> level_ = LogLevel::Info;
        size_t messagesSinceLastFlush = 0;
        static constexpr size_t c_FlushInterval = 16;
    };

    /// Returns a reference to the "global" logger state. Initialized on first-use.
    LoggerImpl& getGlobalLogger() {
        static LoggerImpl s_GlobalLogState = LoggerImpl::with_sink<DefaultLogSink>();
        return s_GlobalLogState;
    }

    /// Returns a reference to the "cout" logger state. Initialized on first-use.
    LoggerImpl& getCoutLogger() {
        static LoggerImpl s_CoutLogState = LoggerImpl::with_sink<CoutSink>();
        return s_CoutLogState;
    }

    /// Initializes the global file log
    void initializeFileLoggingGlobally(
        const std::string& filepath = "opensim.log")
    {
        std::lock_guard guard{g_FileSinkMutex};

        if (g_FileSink) {
            getGlobalLogger().sinkMessage(
                LogLevel::Warn,
                std::format("Already logging to file '{}'; log file not added. Call removeFileSink() first.", g_FileSink->filename().string())
            );
            return;
        }

        // Check if file can be opened at the specified path if not return
        // meaningful warning rather than bubble the exception up.
        try {
            g_FileSink = std::make_shared<BasicFileSink>(filepath);
        }
        catch (...) {
            getGlobalLogger().sinkMessage(
                LogLevel::Warn,
                std::format("Can't open file '{}' for writing. Log file will not be created. Check that you have write permissions to the specified path.",
                    filepath
                )
            );
            return;
        }
        getGlobalLogger().addSink(g_FileSink);
        getCoutLogger().addSink(g_FileSink);
    }

    void automaticallyInitializeFileLoggingGlobally()
    {
#ifndef OPENSIM_DISABLE_LOG_FILE
        std::call_once(g_FileSinkAutoInitOnceFlag, [] { initializeFileLoggingGlobally(); });
#endif
    }
}

void Logger::setLevel(Level level) {
    automaticallyInitializeFileLoggingGlobally();
    getGlobalLogger().setLevel(level);
    getCoutLogger().setLevel(level);
    info("Set log level to {}.", getLevelString());
}

Logger::Level Logger::getLevel() {
    automaticallyInitializeFileLoggingGlobally();
    return getGlobalLogger().getLevel();
}

void Logger::setLevelString(std::string str) {
    setLevel(parseLogLevel(std::move(str)));
}

std::string Logger::getLevelString() {
    return std::string{labelFor(getLevel())};
}

bool Logger::shouldLog(Level level) {
    automaticallyInitializeFileLoggingGlobally();
    return getGlobalLogger().shouldLog(level);
}

void Logger::addFileSink(const std::string& filepath) {
    // This method is either called by the file log auto-initializer, which
    // should now be disabled, or by downstream code trying to manually
    // specify a file sink (and, therefore, auto-initialization should be
    // disabled from now-on).
    std::call_once(g_FileSinkAutoInitOnceFlag, []{});
    initializeFileLoggingGlobally(filepath);
}

void Logger::removeFileSink() {
    // This method is either called by the file log auto-initializer, which
    // should now be disabled, or by downstream code trying to manually specify
    // a file sink (and, therefore, auto-initialization should be disabled).
    std::call_once(g_FileSinkAutoInitOnceFlag, []{});

    std::lock_guard guard{g_FileSinkMutex};
    if (not g_FileSink) {
        return;
    }
    removeSink(g_FileSink);
    g_FileSink.reset();
}

void Logger::addSink(std::shared_ptr<LogSink> sink) {
    automaticallyInitializeFileLoggingGlobally();
    getGlobalLogger().addSink(sink);
    getCoutLogger().addSink(std::move(sink));
}

void Logger::removeSink(const std::shared_ptr<LogSink>& sink) {
    automaticallyInitializeFileLoggingGlobally();
    getGlobalLogger().removeSink(sink);
    getCoutLogger().removeSink(sink);
}

void Logger::sinkMessage(LogLevel level, std::string&& payload) {
    automaticallyInitializeFileLoggingGlobally();
    getGlobalLogger().sinkMessage(level, std::move(payload));
}

void Logger::sinkCoutMessage(std::string&& payload) {
    automaticallyInitializeFileLoggingGlobally();
    getCoutLogger().sinkMessage(getCoutLogger().getLevel(), std::move(payload));
}
