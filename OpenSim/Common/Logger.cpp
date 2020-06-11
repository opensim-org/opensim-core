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

#include "Exception.h"
#include "IO.h"
#include "LogSink.h"

#include "spdlog/sinks/stdout_color_sinks.h"

using namespace OpenSim;

std::shared_ptr<spdlog::logger> Logger::m_cout_logger = 
        spdlog::stdout_color_mt("cout");
std::shared_ptr<spdlog::sinks::basic_file_sink_mt> Logger::m_filesink = {};
std::shared_ptr<spdlog::logger> Logger::m_default_logger;

// Force creation of the Logger instane to initialize spdlog::loggers
std::shared_ptr<OpenSim::Logger> Logger::m_osimLogger = Logger::getInstance();

Logger::Logger() {
    m_default_logger = spdlog::default_logger();
    m_default_logger->set_level(spdlog::level::info);
    m_default_logger->set_pattern("[%l] %v");
    m_cout_logger->set_level(spdlog::level::info);
    m_cout_logger->set_pattern("%v");
    // This ensures log files are updated regularly, instead of only when the
    // program shuts down.
    spdlog::flush_on(spdlog::level::info);
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
    const auto level = m_default_logger->level();
    switch (level) {
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
                fmt::format("Expected log level to be Off, Critical, Error, "
                            "Warn, Info, Debug, or Trace; got {}.",
                        str));
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
    return m_default_logger->should_log(spdlogLevel);
}

void Logger::addFileSink(const std::string& filepath) {
    if (m_filesink) {
        warn("Already logging to file '{}'; log file not added. Call "
             "removeFileSink() first.", m_filesink->filename());
        return;
    }
    // verify if file can be opened at the specified path
    std::ifstream ifs(filepath);

    if (ifs.is_open()) {
        ifs.close();
    } else {
        // show message:
         warn("Can't open file '{}' for writing. Log file will not be created. "
             "Check that you have write permissions to the specified path.", 
             filepath);
        return;
    }
    m_filesink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(filepath);
    addSinkInternal(m_filesink);
}

void Logger::removeFileSink() {
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

void Logger::addSinkInternal(std::shared_ptr<spdlog::sinks::sink> sink) {
    m_default_logger->sinks().push_back(sink);
    m_cout_logger->sinks().push_back(sink);
}

void Logger::removeSinkInternal(const std::shared_ptr<spdlog::sinks::sink> sink)
{
    {
        auto& sinks = m_default_logger->sinks();
        auto to_erase = std::find(sinks.cbegin(), sinks.cend(), sink);
        if (to_erase != sinks.cend()) sinks.erase(to_erase);
    }
    {
        auto& sinks = m_cout_logger->sinks();
        auto to_erase = std::find(sinks.cbegin(), sinks.cend(), sink);
        if (to_erase != sinks.cend()) sinks.erase(to_erase);
    }
}



