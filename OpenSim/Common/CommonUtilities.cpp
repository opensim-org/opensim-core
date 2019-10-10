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
#include <ctime>
#include <iomanip>
#include <memory>
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

int OpenSim::getStateIndex(
        const SimTK::Array_<std::string>& labels, const std::string& name) {

    auto find = [&labels](const std::string& s) -> int {
      auto it = std::find(labels.cbegin(), labels.cend(), s);
      if (it == labels.cend()) return -1;
      return std::distance(labels.cbegin(), it);
    };

    int index = -1;

    // This uses the `do while(false)` idiom to run common code if one of a
    // number of conditions succeeds (much like what a goto would be used for).
    index = find(name);
    if (index != -1) return index;

    // 4.0 and its beta versions differ slightly in the absolute path but
    // the <joint>/<coordinate>/value (or speed) will be common to both.
    // Likewise, for muscle states <muscle>/activation (or fiber_length)
    // must be common to the state variable (path) name and column label.
    std::string shortPath = name;
    std::string::size_type front = shortPath.find("/");
    while (index < 0 && front < std::string::npos) {
        shortPath = shortPath.substr(front + 1, name.length());
        index = find(shortPath);
        front = shortPath.find("/");
    }
    if (index != -1) return index;

    // Assume labels follow pre-v4.0 state variable labeling.
    // Redo search with what the pre-v4.0 label might have been.

    // First, try just the last element of the path.
    std::string::size_type back = name.rfind("/");
    std::string prefix = name.substr(0, back);
    std::string shortName = name.substr(back + 1, name.length() - back);
    index = find(shortName);
    if (index != -1) return index;

    // If that didn't work, specifically check for coordinate state names
    // (<coord_name>/value and <coord_name>/speed) and muscle state names
    // (<muscle_name>/activation <muscle_name>/fiber_length).
    if (shortName == "value") {
        // pre-v4.0 did not have "/value" so remove it if here
        back = prefix.rfind("/");
        shortName = prefix.substr(back + 1, prefix.length());
        index = find(shortName);
    } else if (shortName == "speed") {
        // replace "/speed" (the v4.0 labeling for speeds) with "_u"
        back = prefix.rfind("/");
        shortName = prefix.substr(back + 1, prefix.length() - back) + "_u";
        index = find(shortName);
    } else if (back < name.length()) {
        // try replacing the '/' with '.' in the last segment
        shortName = name;
        shortName.replace(back, 1, ".");
        back = shortName.rfind("/");
        shortName = shortName.substr(back + 1, shortName.length() - back);
        index = find(shortName);
    }
    if (index != -1) return index;

    // If all of the above checks failed, return -1.
    return -1;
}
