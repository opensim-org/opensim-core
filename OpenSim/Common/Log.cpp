/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Log.cpp                                *
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

#include "Log.h"
#include "LogSink.h"
#include "Exception.h"

#include "spdlog/sinks/basic_file_sink.h"

using namespace OpenSim;

std::shared_ptr<Log> Log::m_log = Log::getInstance();
std::set<std::string> Log::m_filepaths = {};

Log::Log() {
    spdlog::set_level(spdlog::level::info);
    spdlog::set_pattern("[%l] %v");
}

void Log::setLevel(Level level) {
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
    }
}

Log::Level Log::getLevel() {
    const auto level = spdlog::default_logger()->level();
    switch (level) {
    case spdlog::level::off: return Level::Off;
    case spdlog::level::critical: return Level::Critical;
    case spdlog::level::err: return Level::Error;
    case spdlog::level::warn: return Level::Warn;
    case spdlog::level::info: return Level::Info;
    case spdlog::level::debug: return Level::Debug;
    case spdlog::level::trace: return Level::Trace;
    }
}

void Log::setLevelString(const std::string& str) {
    Level level;
    if (str == "Off") level = Level::Off;
    else if (str == "Critical") level = Level::Critical;
    else if (str == "Error") level = Level::Error;
    else if (str == "Warn") level = Level::Warn;
    else if (str == "Info") level = Level::Info;
    else if (str == "Debug") level = Level::Debug;
    else if (str == "Trace") level = Level::Trace;
    else {
        OPENSIM_THROW(Exception,
                fmt::format("Expected log level to be Off, Critical, Error, "
                            "Warn, Info, Debug, or Trace; got {}.",
                        str));
    }
    setLevel(level);
}

std::string Log::getLevelString() {
    const auto level = getLevel();
    switch (level) {
    case Level::Off: return "Off";
    case Level::Critical: return "Critical";
    case Level::Error: return "Error";
    case Level::Warn: return "Warn";
    case Level::Info: return "Info";
    case Level::Debug: return "Debug";
    case Level::Trace: return "Trace";
    }
}

bool Log::shouldLog(Level level) {
    spdlog::level::level_enum spdlogLevel;
    switch (level) {
    case Level::Off: spdlogLevel = spdlog::level::off; break;
    case Level::Critical: spdlogLevel = spdlog::level::critical; break;
    case Level::Error: spdlogLevel = spdlog::level::err; break;
    case Level::Warn: spdlogLevel = spdlog::level::warn; break;
    case Level::Info: spdlogLevel = spdlog::level::info; break;
    case Level::Debug: spdlogLevel = spdlog::level::debug; break;
    case Level::Trace: spdlogLevel = spdlog::level::trace; break;
    }
    return spdlog::default_logger()->should_log(spdlogLevel);
}

void Log::addLogFile(const std::string& filepath) {
    auto sinks = spdlog::default_logger()->sinks();
    if (m_filepaths.count(filepath)) {
        warn("Already logging to file '{}'.", filepath);
        return;
    }
    sinks.push_back(
            std::make_shared<spdlog::sinks::basic_file_sink_mt>(filepath));
    m_filepaths.insert(filepath);
}

void Log::addSink(const std::shared_ptr<LogSink> sink) {
    spdlog::default_logger()->sinks().push_back(
            std::static_pointer_cast<spdlog::sinks::sink>(sink));
}

void Log::removeSink(const std::shared_ptr<LogSink> sink) {
    auto spdlogSink = std::static_pointer_cast<spdlog::sinks::sink>(sink);
    auto sinks = spdlog::default_logger()->sinks();
    auto to_erase = std::find(sinks.cbegin(), sinks.cend(), spdlogSink);
    if (to_erase != sinks.cend()) sinks.erase(to_erase);
}
