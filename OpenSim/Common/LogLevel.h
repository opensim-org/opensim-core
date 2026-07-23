#ifndef OPENSIM_LOG_LEVEL_H_
#define OPENSIM_LOG_LEVEL_H_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  LogLevel.h                                 *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2026 Stanford University and the Authors                *
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

namespace OpenSim {

    /// This enum lists the types of messages that should be logged, ordered
    /// from least-severe to most-severe (or off).
    enum class LogLevel {
        /// Log as much as possible, including messages that describe the
        /// software's behavior step by step. Note: OpenSim has very few Trace-level
        /// messages.
        Trace = 0,

        /// Log information that may be useful when debugging the operation of
        /// the software to investigate unexpected results.
        Debug = 1,

        /// Default.
        Info = 2,

        /// Log warnings. Warnings are generated when the software will proceed
        /// but the user should check their input.
        Warn = 3,

        /// Log all messages that require user intervention.
        Error = 4,

        /// Only log critical errors.
        Critical = 5,

        /// Do not log any messages. Useful when running an optimization or
        /// automated pipeline.
        Off = 6,
    };
}

#endif
