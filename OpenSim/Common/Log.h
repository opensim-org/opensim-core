#ifndef OPENSIM_LOG_H_
#define OPENSIM_LOG_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Log.h                                  *
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

#include "osimCommonDLL.h"

#include <spdlog/spdlog.h>

#include <string>

namespace OpenSim {

class OSIMCOMMON_API Log {
public:
    /// This enum lists the types of messages that should be logged. These
    /// levels match those of the spdlog logging library that OpenSim uses for
    /// logging.
    enum class Level {
        /// Do not log any messages. Useful when running an optimization or
        /// automated pipeline.
        Off = 6,
        /// Only log critical errors.
        Critical = 5,
        /// Log all messages that require user intervention.
        Error = 4,
        /// Log warnings. Warnings are generated when the software will proceed
        /// but the user should check their input.
        Warn = 3,
        /// Default.
        Info = 2,
        /// Log information that may be useful when debugging the operation of
        /// the
        /// software to investigate unexpected results.
        Debug = 1,
        /// Log as much as possible, including messages that describe the
        /// software's
        /// behavior step by step. Note: OpenSim has very few Trace-level
        /// messages.
        Trace = 0
    };

    /// Log messages of importance `level` and greater.
    /// For example, if the level is set to Info, then Critical, Error, Warn,
    /// and Info messages are logged, while Debug and Trace messages are not
    /// logged.
    static void setLevel(Level level);
    static Level getLevel();

    /// Set the logging level using one of the following strings:
    /// - Off
    /// - Critical
    /// - Error
    /// - Warn
    /// - Info
    /// - Debug
    /// - Trace
    /// This variant of setLevel() is for use in Matlab.
    /// @see Level.
    static void setLevelString(const std::string& level);
    static std::string getLevelString();

    template <typename... Args>
    static void critical(spdlog::string_view_t fmt, const Args&... args) {
        spdlog::critical(fmt, args...);
    }

    template <typename... Args>
    static void error(spdlog::string_view_t fmt, const Args&... args) {
        spdlog::error(fmt, args...);
    }

    template <typename... Args>
    static void warn(spdlog::string_view_t fmt, const Args&... args) {
        spdlog::warn(fmt, args...);
    }

    template <typename... Args>
    static void info(spdlog::string_view_t fmt, const Args&... args) {
        spdlog::info(fmt, args...);
    }

    template <typename... Args>
    static void debug(spdlog::string_view_t fmt, const Args&... args) {
        spdlog::debug(fmt, args...);
    }

    template <typename... Args>
    static void trace(spdlog::string_view_t fmt, const Args&... args) {
        spdlog::trace(fmt, args...);
    }
};

} // namespace OpenSim

#endif // OPENSIM_LOG_H_
