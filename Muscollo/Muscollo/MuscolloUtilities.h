#ifndef MUSCOLLO_MUSCOLLOUTILITIES_H
#define MUSCOLLO_MUSCOLLOUTILITIES_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MuscolloUtilities.h                                               *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Common/Storage.h>

#include <set>

#include "osimMuscolloDLL.h"

namespace OpenSim {

class StatesTrajectory;
class Model;

/// Create a SimTK::Vector with the provided length whose elements are
/// linearly spaced between start and end.
OSIMMUSCOLLO_API
SimTK::Vector createVectorLinspace(int length, double start, double end);

/// Linearly interpolate y(x) at new values of x.
/// The returned vector will have NaN for any values of newX outside of the
/// range of x.
OSIMMUSCOLLO_API
SimTK::Vector interpolate(const SimTK::Vector& x,
        const SimTK::Vector& y, const SimTK::Vector& newX);

/// Create a Storage from a TimeSeriesTable. Metadata from the
/// TimeSeriesTable is *not* copied to the Storage.
/// You should use TimeSeriesTable if possible, as support for Storage may be
/// reduced in future versions of OpenSim. However, Storage supports some
/// operations not supported by TimeSeriesTable (e.g., filtering, resampling).
// TODO move to the Storage class.
OSIMMUSCOLLO_API Storage convertTableToStorage(const TimeSeriesTable&);

/// TODO: doc
OSIMMUSCOLLO_API TimeSeriesTable filterLowpass(const TimeSeriesTable& table, 
    double cutoffFreq, bool padData = false);

/// Play back a motion (from the Storage) in the simbody-visuailzer. The Storage
/// should contain all generalized coordinates. The visualizer window allows the
/// user to control playback speed.
/// This function blocks until the user exits the simbody-visualizer window.
// TODO handle degrees.
OSIMMUSCOLLO_API void visualize(Model, Storage);

#ifndef SWIG
/// The map provides the index of each state variable in
/// SimTK::State::getY() from its state variable path string.
std::unordered_map<std::string, int> createSystemYIndexMap(const Model& model);
#endif

/// Throw an exception if the property's value is not in the provided set.
/// We assume that `p` is a single-value property.
template <typename T>
void checkPropertyInSet(const Object& obj,
        const Property<T>& p, const std::set<T>& set) {
    const auto& value = p.getValue();
    if (set.find(value) == set.end()) {
        std::stringstream msg;
        msg << "Property '" << p.getName() << "' (in ";
        if (!obj.getName().empty()) {
            msg << "object '" << obj.getName() << "' of type ";
        }
        msg << obj.getConcreteClassName() << ") has invalid value "
                << value << "; expected one of the following:";
        std::string separator(" ");
        for (const auto& s : set) {
            msg << separator << " " << s;
            if (separator.empty()) separator = ", ";
        }
        msg << ".";
        OPENSIM_THROW(Exception, msg.str());
    }
}

/// Throw an exception if the property's value is not positive.
/// We assume that `p` is a single-value property.
template <typename T>
void checkPropertyIsPositive(const Object& obj, const Property<T>& p) {
    const auto& value = p.getValue();
    if (value <= 0) {
        std::stringstream msg;
        msg << "Property '" << p.getName() << "' (in ";
        if (!obj.getName().empty()) {
            msg << "object '" << obj.getName() << "' of type ";
        }
        msg << obj.getConcreteClassName() << ") must be positive, but is "
                << value << ".";
        OPENSIM_THROW(Exception, msg.str());
    }
}

/// Throw an exception if the property's value is neither in the provided
/// range nor in the provided set.
/// We assume that `p` is a single-value property.
template <typename T>
void checkPropertyInRangeOrSet(const Object& obj, const Property<T>& p,
        const T& lower, const T& upper, const std::set<T>& set) {
    const auto& value = p.getValue();
    if ((value < lower || value > upper) && set.find(value) == set.end()) {
        std::stringstream msg;
        msg << "Property '" << p.getName() << "' (in ";
        if (!obj.getName().empty()) {
            msg << "object '" << obj.getName() << "' of type ";
        }
        msg << obj.getConcreteClassName() << ") has invalid value "
                << value << "; expected value to be in range "
                << lower << "," << upper << ", or one of the following:";
        std::string separator("");
        for (const auto& s : set) {
            msg << " " << separator << s;
            if (separator.empty()) separator = ",";
        }
        msg << ".";
        OPENSIM_THROW(Exception, msg.str());
    }
}

/// Record and report elapsed real time ("clock" or "wall" time) in seconds.
class Stopwatch {
public:
    /// This stores the start time as the current time.
    Stopwatch() {
        reset();
    }
    /// Reset the start time to the current time.
    void reset() {
        m_startTime = SimTK::realTime();
    }
    /// Return the amount of time that has elapsed since this object was
    /// constructed or since reset() has been called.
    double getElapsedTime() const {
        return SimTK::realTime() - m_startTime;
    }
    /// This provides the elapsed time as a formatted string (using format()).
    std::string getElapsedTimeFormatted() const {
        return format(getElapsedTime());
    }
    /// Convert the provided elapsed time in seconds to a string, which may
    /// contain the time in hours and minutes if the elapsed time is 60 seconds
    /// or greater.
    static std::string format(const double& seconds) {
        std::stringstream ss;
        int sec = (int)std::round(seconds);
        ss << sec << " seconds";
        int minutes = sec / 60;
        int hours = minutes / 60;
        if (minutes || hours) {
            ss << " (";
            if (hours) {
                ss << hours << " hours, ";
                ss << minutes % 60 << " minutes, ";
                ss << sec % 60 << " seconds";
            } else {
                ss << minutes % 60 << " minutes, ";
                ss << sec % 60 << " seconds";
            }
            ss << ")";
        }
        return ss.str();
    }
private:
    double m_startTime;
};

} // namespace OpenSim

#endif // MUSCOLLO_MUSCOLLOUTILITIES_H
