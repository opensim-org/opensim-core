/* -------------------------------------------------------------------------- *
 *                          OpenSim:  CommonUtilities.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2019 Stanford University and the Authors                *
 * Author(s): Christopher Dembia                                              *
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

#include "CommonUtilities.h"

#include <chrono>
#include <iomanip>
#include <ctime>
#include <sstream>

std::string OpenSim::getFormattedDateTime(
        bool appendMicroseconds, std::string format) {
    using namespace std::chrono;
    auto now = system_clock::now();
    auto time_now = system_clock::to_time_t(now);
    struct tm buf;
#if defined(_WIN32)
    localtime_s(&buf, &time_now);
#else
    localtime_r(&time_now, &buf);
#endif
    if (format == "ISO") { format = "%Y-%m-%dT%H:%M:%S"; }

    // To get the date/time in the desired format, we would ideally use
    // std::put_time, but that is not available in GCC < 5.
    // https://stackoverflow.com/questions/30269657/what-is-an-intelligent-way-to-determine-max-size-of-a-strftime-char-array
    int size = 32;
    std::unique_ptr<char[]> formatted(new char[size]);
    while (strftime(formatted.get(), size - 1, format.c_str(), &buf) == 0) {
        size *= 2;
        formatted.reset(new char[size]);
    }

    std::stringstream ss;
    ss << std::string(formatted.get(), size);

    if (appendMicroseconds) {
        // Get number of microseconds since last second.
        auto microsec =
                duration_cast<microseconds>(now.time_since_epoch()) % 1000000;
        ss << '.' << std::setfill('0') << std::setw(6) << microsec.count();
    }
    return ss.str();
}
