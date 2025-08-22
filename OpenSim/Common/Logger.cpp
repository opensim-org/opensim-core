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

#include "Exception.h"                       // for Exception, OPENSIM_THROW
#include "IO.h"                              // for IO
#include "LogSink.h"                         // for LogSink
#include <algorithm>                         // for remove
#include <spdlog/fmt/bundled/format.h>       // for format
#include <spdlog/sinks/basic_file_sink.h>    // for basic_file_sink_mt, bas...
#include <spdlog/sinks/sink.h>               // for sink
#include <spdlog/sinks/stdout_color_sinks.h> // for stdout_color_mt
#include <spdlog/spdlog.h>                   // for set_level, default_logger
#include <vector>                            // for vector

using namespace OpenSim;

// Static initialization of default and cout loggers.
//
// There are objects from other compilation units that call into the default
// (and probably cout) logger objects during static initialization so use the
// "Construct On First Use" idiom to ensure that each logger is allocated and
// completely initialized before that happens the first time.

// Initialize a logger
static std::shared_ptr<spdlog::logger> initializeLogger(
    std::shared_ptr<spdlog::logger> l,
    const char* pattern)
{
    l->set_level(spdlog::level::info);
    l->flush_on(spdlog::level::info);
    l->set_pattern(pattern);
    return l;
}

// Return a reference to the static cout logger object, allocating and
// initializing it on the first call.
static spdlog::logger& coutLoggerInternal() {
    static std::shared_ptr<spdlog::logger> l =
      initializeLogger(spdlog::stdout_color_mt("cout"), "%v");
    return *l;
}

// Return a reference to the static default logger object, allocating and
// initializing it on the first call.
static spdlog::logger& defaultLoggerInternal() {
    static std::shared_ptr<spdlog::logger> l =
      initializeLogger(spdlog::default_logger(), "[%l] %v");
    return *l;
}

// the file log sink (e.g. `opensim.log`) is lazily initialized.
//
// it is only initialized when the first log message is about to be written to
// it. Users *may* disable this functionality before the first log message is
// written (or disable it statically, by setting OPENSIM_DISABLE_LOG_FILE)
static std::shared_ptr<spdlog::sinks::basic_file_sink_mt> m_filesink = nullptr;

// if a user manually calls `Logger::(remove|add)FileSink`, auto-initialization
// should be disabled. Manual usage "overrides" lazy auto-initialization.
static bool fileSinkAutoInitDisabled = false;

// attempt to auto-initialize the file log, if applicable
static bool initFileLoggingAsNeeded() {
#ifdef OPENSIM_DISABLE_LOG_FILE
// software builders may want to statically ensure that automatic file logging
// *cannot* happen - even during static initialization. This compiler define
// outright disables the behavior, which is important in Windows applications
// that run multiple instances of OpenSim-linked binaries. In Windows, the
// logs files collide and cause a "multiple processes cannot open the same
// file" error).
return true;
#else
    static bool initialized = []() {
        if (fileSinkAutoInitDisabled) {
            return true;
        }
        Logger::addFileSink();
        return true;
    }();

    return initialized;
#endif
}

// this function is only called when the caller is about to log something, so
// it should perform lazy initialization of the file sink
spdlog::logger& Logger::getCoutLogger() {
    initFileLoggingAsNeeded();
    return coutLoggerInternal();
}

// this function is only called when the caller is about to log something, so
// it should perform lazy initialization of the file sink
spdlog::logger& Logger::getDefaultLogger() {
    initFileLoggingAsNeeded();
    return defaultLoggerInternal();
}

static void addSinkInternal(std::shared_ptr<spdlog::sinks::sink> sink) {
    coutLoggerInternal().sinks().push_back(sink);
    defaultLoggerInternal().sinks().push_back(sink);
}

static void removeSinkInternal(const std::shared_ptr<spdlog::sinks::sink> sink)
{
    {
        auto& sinks = defaultLoggerInternal().sinks();
        auto new_end = std::remove(sinks.begin(), sinks.end(), sink);
        sinks.erase(new_end, sinks.end());
    }
    {
        auto& sinks = coutLoggerInternal().sinks();
        auto new_end = std::remove(sinks.begin(), sinks.end(), sink);
        sinks.erase(new_end, sinks.end());
    }
}

void Logger::setLevel(Level level) {
    switch (level) {
    case Level::Off:
        spdlog::set_level(spdlog::level::off);
        break;
    case Level::Critical:
        spdlog::set_level(spdlog::level::critical);
        break;
    case Level::Error:
        spdlog::set_level(spdlog::level::err);
        break;
    case Level::Warn:
        spdlog::set_level(spdlog::level::warn);
        break;
    case Level::Info:
        spdlog::set_level(spdlog::level::info);
        break;
    case Level::Debug:
        spdlog::set_level(spdlog::level::debug);
        break;
    case Level::Trace:
        spdlog::set_level(spdlog::level::trace);
        break;
    default:
        OPENSIM_THROW(Exception, "Internal error.");
    }
    Logger::info("Set log level to {}.", getLevelString());
}

Logger::Level Logger::getLevel() {
    switch (defaultLoggerInternal().level()) {
    case spdlog::level::off: return Level::Off;
    case spdlog::level::critical: return Level::Critical;
    case spdlog::level::err: return Level::Error;
    case spdlog::level::warn: return Level::Warn;
    case spdlog::level::info: return Level::Info;
    case spdlog::level::debug: return Level::Debug;
    case spdlog::level::trace: return Level::Trace;
    default:
        OPENSIM_THROW(Exception, "Internal error.");
    }
}

void Logger::setLevelString(std::string str) {
    Level level;
    str = IO::Lowercase(str);
    if (str == "off") level = Level::Off;
    else if (str == "critical") level = Level::Critical;
    else if (str == "error") level = Level::Error;
    else if (str == "warn") level = Level::Warn;
    else if (str == "info") level = Level::Info;
    else if (str == "debug") level = Level::Debug;
    else if (str == "trace") level = Level::Trace;
    else {
        OPENSIM_THROW(Exception,
                "Expected log level to be Off, Critical, Error, "
                "Warn, Info, Debug, or Trace; got {}.",
                str);
    }
    setLevel(level);
}

std::string Logger::getLevelString() {
    const auto level = getLevel();
    switch (level) {
    case Level::Off: return "Off";
    case Level::Critical: return "Critical";
    case Level::Error: return "Error";
    case Level::Warn: return "Warn";
    case Level::Info: return "Info";
    case Level::Debug: return "Debug";
    case Level::Trace: return "Trace";
    default:
        OPENSIM_THROW(Exception, "Internal error.");
    }
}

bool Logger::shouldLog(Level level) {
    spdlog::level::level_enum spdlogLevel;
    switch (level) {
    case Level::Off: spdlogLevel = spdlog::level::off; break;
    case Level::Critical: spdlogLevel = spdlog::level::critical; break;
    case Level::Error: spdlogLevel = spdlog::level::err; break;
    case Level::Warn: spdlogLevel = spdlog::level::warn; break;
    case Level::Info: spdlogLevel = spdlog::level::info; break;
    case Level::Debug: spdlogLevel = spdlog::level::debug; break;
    case Level::Trace: spdlogLevel = spdlog::level::trace; break;
    default:
        OPENSIM_THROW(Exception, "Internal error.");
    }
    return defaultLoggerInternal().should_log(spdlogLevel);
}

void Logger::addFileSink(const std::string& filepath) {
    // this method is either called by the file log auto-initializer, which
    // should now be disabled, or by downstream code trying to manually specify
    // a file sink
    //
    // downstream callers would find it quite surprising if the auto-initializer
    // runs *after* they manually specify a log, so just disable it
    fileSinkAutoInitDisabled = true;
    spdlog::logger& logger = defaultLoggerInternal();

    if (m_filesink) {
        logger.warn("Already logging to file '{}'; log file not added. Call "
                    "removeFileSink() first.", m_filesink->filename());
        return;
    }

    // check if file can be opened at the specified path if not return meaningful
    // warning rather than bubble the exception up.
    try {
        m_filesink =
                std::make_shared<spdlog::sinks::basic_file_sink_mt>(filepath);
    }
    catch (...) {
        logger.warn("Can't open file '{}' for writing. Log file will not be created. "
                    "Check that you have write permissions to the specified path.",
                    filepath);
        return;
    }
    addSinkInternal(m_filesink);
}

void Logger::removeFileSink() {
    // if this method is called, then we are probably at a point in the
    // application's lifetime where automatic log allocation is going to cause
    // confusion.
    //
    // callers will be surpised if, after calling this method, auto
    // initialization happens afterwards and the log file still exists - even
    // if they called it to remove some manually-specified log
    fileSinkAutoInitDisabled = true;

    if (m_filesink == nullptr) {
        return;
    }

    removeSinkInternal(
            std::static_pointer_cast<spdlog::sinks::sink>(m_filesink));
    m_filesink.reset();
}

void Logger::addSink(const std::shared_ptr<LogSink> sink) {
    addSinkInternal(std::static_pointer_cast<spdlog::sinks::sink>(sink));
}

void Logger::removeSink(const std::shared_ptr<LogSink> sink) {
    removeSinkInternal(std::static_pointer_cast<spdlog::sinks::sink>(sink));
}


