#ifndef _LogManager_h_
#define _LogManager_h_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  LogManager.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include "Array.h"
#include "LogCallback.h"
#include <iostream>
#include <sstream>

namespace OpenSim {

class OSIMCOMMON_API StreamLogCallback : public LogCallback
{
public:
    StreamLogCallback(const std::string &aFilename);
    StreamLogCallback(std::ostream *aOut, bool aOwnsStream = true);
    ~StreamLogCallback();
    void log(const std::string &aStr) override;

private:
    std::ostream *_out;
    bool _ownsStream;

};


class OSIMCOMMON_API LogBuffer : public std::stringbuf
{
public:
    LogBuffer();
    ~LogBuffer();
    bool addLogCallback(LogCallback *aLogCallback);
    bool removeLogCallback(LogCallback *aLogCallback);

private:
    Array<LogCallback*> _logCallbacks;

    int sync() override;
};

/// This enum lists the types of messages that should be logged. These levels
/// match those of the spdlog logging library that OpenSim uses for logging.
enum class LogLevel {
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
    /// Log information that may be useful when debugging the operation of the
    /// software to investigate unexpected results.
    Debug = 1,
    /// Log as much as possible, including messages that describe the software's
    /// behavior step by step. Note: OpenSim has very few Trace-level messages.
    Trace = 0
};

class OSIMCOMMON_API LogManager
{
public:
    // Expose these members so users can manipulate output formats by calling
    // functions on LogManager::out/err
    static LogBuffer out;
    static LogBuffer err;

    // LogManager's cout and cerr act as the normal standard output and error
    // (i.e. they write to the terminal)
    static std::ostream cout;
    static std::ostream cerr;

    LogManager();
    ~LogManager();

    static LogManager *getInstance();

    LogBuffer *getOutBuffer();
    LogBuffer *getErrBuffer();

    /// Log messages of importance `level` and greater.
    /// For example, if the level is set to Info, then Critical, Error, Warn,
    /// and Info messages are logged, while Debug and Trace messages are not
    /// logged.
    static void setLogLevel(LogLevel level);
    static LogLevel getLogLevel();

    /// Set the logging level using one of the following strings:
    /// - Off
    /// - Critical
    /// - Error
    /// - Warn
    /// - Info
    /// - Debug
    /// - Trace
    /// This variant of setLogLevel() is for use from Matlab.
    /// @see LogLevel.
    static void setLogLevelString(const std::string& level);
    static std::string getLogLevelString();
};

}

#endif