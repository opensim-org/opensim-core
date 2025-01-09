#ifndef OPENSIM_STOPWATCH_H_
#define OPENSIM_STOPWATCH_H_
/* -------------------------------------------------------------------------- *
 *                             OpenSim: Stopwatch.h                           *
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

namespace OpenSim {

/// Record and report elapsed real time ("clock" or "wall" time) in seconds.
class Stopwatch {
public:
    /// This stores the start time as the current time.
    Stopwatch() { reset(); }
    /// Reset the start time to the current time.
    void reset() { m_startTime = SimTK::realTimeInNs(); }
    /// Return the amount of time that has elapsed since this object was
    /// constructed or since reset() has been called.
    double getElapsedTime() const {
        return SimTK::realTime() - SimTK::nsToSec(m_startTime);
    }
    /// Get elapsed time in nanoseconds. See SimTK::realTimeInNs() for more
    /// information.
    long long getElapsedTimeInNs() const {
        return SimTK::realTimeInNs() - m_startTime;
    }
    /// This provides the elapsed time as a formatted string (using format()).
    std::string getElapsedTimeFormatted() const {
        return formatNs(getElapsedTimeInNs());
    }
    /// Format the provided elapsed time in nanoseconds into a string.
    /// The time may be converted into seconds, milliseconds, or microseconds.
    /// Additionally, if the time is greater or equal to 60 seconds, the time in
    /// hours and/or minutes is also added to the string.
    /// Usually, you can call getElapsedTimeFormatted() instead of calling this
    /// function directly. If you call this function directly, use
    /// getElapsedTimeInNs() to get a time in nanoseconds (rather than
    /// getElapsedTime()).
    static std::string formatNs(const long long& nanoseconds) {
        std::stringstream ss;
        double seconds = SimTK::nsToSec(nanoseconds);
        int secRounded = (int)std::round(seconds);
        if (seconds > 1)
            ss << secRounded << " second(s)";
        else if (nanoseconds >= 1000000)
            ss << nanoseconds / 1000000 << " millisecond(s)";
        else if (nanoseconds >= 1000)
            ss << nanoseconds / 1000 << " microsecond(s)";
        else
            ss << nanoseconds << " nanosecond(s)";
        int minutes = secRounded / 60;
        int hours = minutes / 60;
        if (minutes || hours) {
            ss << " (";
            if (hours) {
                ss << hours << " hour(s), ";
                ss << minutes % 60 << " minute(s), ";
                ss << secRounded % 60 << " second(s)";
            } else {
                ss << minutes % 60 << " minute(s), ";
                ss << secRounded % 60 << " second(s)";
            }
            ss << ")";
        }
        return ss.str();
    }

private:
    long long m_startTime;
};

} // namespace OpenSim

#endif // OPENSIM_STOPWATCH_H_
