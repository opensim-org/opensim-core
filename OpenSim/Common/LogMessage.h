#ifndef OPENSIM_LOG_MESSAGE_H_
#define OPENSIM_LOG_MESSAGE_H_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  LogMessage.h                               *
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

#include <chrono>
#include <string>
#include <string_view>
#include <utility>

namespace OpenSim {

    class OSIMCOMMON_API LogMessage final {
    public:
        using clock_type = std::chrono::system_clock;
        using time_point_type = clock_type::time_point;

        LogMessage() = default;

        explicit LogMessage(
            std::string loggerName,
            LogLevel level,
            std::string payload) :
            _loggerName{std::move(loggerName)},
            _level{level},
            _payload{std::move(payload)}
        {}

        std::string_view getLoggerName() const { return _loggerName; }
        time_point_type getTime() const { return _time; }
        LogLevel getLevel() const { return _level; }
        const std::string& getPayload() const { return _payload; }

    private:
        std::string _loggerName;
        time_point_type _time = clock_type::now();
        LogLevel _level = LogLevel::Off;
        std::string _payload;
    };
}

#endif // OPENSIM_LOG_MESSAGE_H_
