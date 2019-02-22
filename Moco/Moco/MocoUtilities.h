#ifndef MOCO_MOCOUTILITIES_H
#define MOCO_MOCOUTILITIES_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoUtilities.h                                              *
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

#include "osimMocoDLL.h"
#include <set>
#include <stack>

#include <OpenSim/Common/Storage.h>

namespace OpenSim {

class StatesTrajectory;
class Model;
class MocoIterate;

/// Since Moco does not require C++14 (which contains std::make_unique()),
/// here is an implementation of make_unique().
template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

/// Determine if `string` ends with the substring `ending`.
/// https://stackoverflow.com/questions/874134/find-if-string-ends-with-another-string-in-c
inline bool endsWith(const std::string& string, const std::string& ending) {
    if (string.length() >= ending.length()) {
        return string.compare(string.length() - ending.length(),
                       ending.length(), ending) == 0;
    }
    return false;
}

/// Create a SimTK::Vector with the provided length whose elements are
/// linearly spaced between start and end.
OSIMMOCO_API
SimTK::Vector createVectorLinspace(int length, double start, double end);

#ifndef SWIG
/// Create a SimTK::Vector using modern C++ syntax.
OSIMMOCO_API
SimTK::Vector createVector(std::initializer_list<SimTK::Real> elements);
#endif

/// Linearly interpolate y(x) at new values of x. The optional 'ignoreNaNs'
/// argument will ignore any NaN values contained in the input vectors and
/// create the interpolant from the non-NaN values only. Note that this option
/// does not necessarily prevent NaN values from being returned in 'newX', which
/// will have NaN for any values of newX outside of the range of x.
OSIMMOCO_API
SimTK::Vector interpolate(const SimTK::Vector& x, const SimTK::Vector& y,
        const SimTK::Vector& newX, const bool ignoreNaNs = false);

/// Create a Storage from a TimeSeriesTable. Metadata from the
/// TimeSeriesTable is *not* copied to the Storage.
/// You should use TimeSeriesTable if possible, as support for Storage may be
/// reduced in future versions of OpenSim. However, Storage supports some
/// operations not supported by TimeSeriesTable (e.g., filtering, resampling).
// TODO move to the Storage class.
OSIMMOCO_API Storage convertTableToStorage(const TimeSeriesTable&);

/// Lowpass filter the data in a TimeSeriesTable at a provided cutoff frequency.
/// The table is converted to a Storage object to use the lowpassIIR() method
/// to filter, and then converted back to TimeSeriesTable.
OSIMMOCO_API TimeSeriesTable filterLowpass(
        const TimeSeriesTable& table, double cutoffFreq, bool padData = false);

/// Play back a motion (from the Storage) in the simbody-visuailzer. The Storage
/// should contain all generalized coordinates. The visualizer window allows the
/// user to control playback speed.
/// This function blocks until the user exits the simbody-visualizer window.
// TODO handle degrees.
OSIMMOCO_API void visualize(Model, Storage);

/// This function is the same as visualize(Model, Storage), except that
/// the states are provided in a TimeSeriesTable.
OSIMMOCO_API void visualize(Model, TimeSeriesTable);

/// Given a valid MocoSolution obtained from solving a MocoProblem and the
/// associated OpenSim model, return the model with a prescribed controller
/// appended that will compute the control values from the MocoSolution. This
/// can be useful when computing state-dependent model quantities that require
/// realization to the Dynamics stage or later.
OSIMMOCO_API void prescribeControlsToModel(
        const MocoIterate& iterate, Model& model);

/// Replace muscles in a model with a PathActuator of the same GeometryPath,
/// optimal force, and min/max control defaults.
/// @note This only replaces muscles within the model's ForceSet.
OSIMMOCO_API void replaceMusclesWithPathActuators(Model& model);

/// Remove muscles from the model.
/// @note This only removes muscles within the model's ForceSet.
OSIMMOCO_API void removeMuscles(Model& model);

/// Replace a joint in the model with a WeldJoint. 
/// @note This assumes the joint is in the JointSet and that the joint's
///       connectees are PhysicalOffsetFrames.
OSIMMOCO_API void replaceJointWithWeldJoint(Model& model, 
    const std::string& jointName);

/// The map provides the index of each state variable in
/// SimTK::State::getY() from its each state variable path string.
/// Empty slots in Y (e.g., for quaternions) are ignored.
OSIMMOCO_API
std::vector<std::string> createStateVariableNamesInSystemOrder(
        const Model& model);

#ifndef SWIG
/// Same as above, but you can obtain a map from the returned state variable
/// names to the index in SimTK::State::getY() that accounts for empty slots
/// in Y.
OSIMMOCO_API
std::vector<std::string> createStateVariableNamesInSystemOrder(
        const Model& model, std::unordered_map<int, int>& yIndexMap);

/// The map provides the index of each state variable in
/// SimTK::State::getY() from its state variable path string.
OSIMMOCO_API
std::unordered_map<std::string, int> createSystemYIndexMap(const Model& model);
#endif

/// Throw an exception if the property's value is not in the provided set.
/// We assume that `p` is a single-value property.
template <typename T>
void checkPropertyInSet(
        const Object& obj, const Property<T>& p, const std::set<T>& set) {
    const auto& value = p.getValue();
    if (set.find(value) == set.end()) {
        std::stringstream msg;
        msg << "Property '" << p.getName() << "' (in ";
        if (!obj.getName().empty()) {
            msg << "object '" << obj.getName() << "' of type ";
        }
        msg << obj.getConcreteClassName() << ") has invalid value " << value
            << "; expected one of the following:";
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
        msg << obj.getConcreteClassName() << ") has invalid value " << value
            << "; expected value to be in range " << lower << "," << upper
            << ", or one of the following:";
        std::string separator("");
        for (const auto& s : set) {
            msg << " " << separator << s;
            if (separator.empty()) separator = ",";
        }
        msg << ".";
        OPENSIM_THROW(Exception, msg.str());
    }
}

/// @name Filling in a string with variables.
/// @{

#ifndef SWIG
/// Return type for make_printable()
template <typename T>
struct make_printable_return {
    typedef T type;
};
/// Convert to types that can be printed with sprintf() (vsnprintf()).
/// The generic template does not alter the type.
template <typename T>
inline typename make_printable_return<T>::type make_printable(const T& x) {
    return x;
}

/// Specialization for std::string.
template <>
struct make_printable_return<std::string> {
    typedef const char* type;
};
/// Specialization for std::string.
template <>
inline typename make_printable_return<std::string>::type make_printable(
        const std::string& x) {
    return x.c_str();
}

/// Format a char array using (C interface; mainly for internal use).
OSIMMOCO_API std::string format_c(const char*, ...);

/// Format a string in the style of sprintf. For example, the code
/// `format("%s %d and %d yields %d", "adding", 2, 2, 4)` will produce
/// "adding 2 and 2 yields 4".
template <typename... Types>
std::string format(const std::string& formatString, Types... args) {
    return format_c(formatString.c_str(), make_printable(args)...);
}

/// Print a formatted string to std::cout. A newline is not included, but the
/// stream is flushed.
template <typename... Types>
void printMessage(const std::string& formatString, Types... args) {
    std::cout << format(formatString, args...);
    std::cout.flush();
}

#endif // SWIG

/// @}

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

/// This obtains the value of the OPENSIM_MOCO_PARALLEL environment variable.
/// The value has the following meanings:
/// - 0: run in series (not parallel).
/// - 1: run in parallel using all cores.
/// - greater than 1: run in parallel with this number of threads.
/// If the environment variable is not set, this function returns -1.
///
/// This variable does not indicate which calculations are parallelized
/// or how the parallelization is achieved. Moco may even ignore or override
/// the setting from the environment variable. See documentation elsewhere
/// (e.g., from a specific MocoSolver) for more information.
OSIMMOCO_API int getMocoParallelEnvironmentVariable();

/// This class lets you store objects of a single type for reuse by multiple
/// threads, ensuring threadsafe access to each of those objects.
// TODO: Find a way to always give the same thread the same object.
template <typename T>
class ThreadsafeJar {
public:
    /// Request an object for your exclusive use on your thread. This function
    /// halts the thread until an object is available. Make sure to return
    /// (leave()) the object when you're done!
    std::unique_ptr<T> take() {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_inventoryMonitor.wait(lock, [this]{ return m_entries.size() > 0; });
        std::unique_ptr<T> top = std::move(m_entries.top());
        m_entries.pop();
        return top;
    }
    /// Add or return an object so that another thread can use it. You will need
    /// to std::move() the entry, ensuring that you will no longer have access
    /// to the entry in your code (the pointer will now be null).
    void leave(std::unique_ptr<T> entry) {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_entries.push(std::move(entry));
        lock.unlock();
        m_inventoryMonitor.notify_one();
    }
    /// Obtain the number of entries that can be taken.
    int size() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return (int)m_entries.size();
    }
private:
    std::stack<std::unique_ptr<T>> m_entries;
    mutable std::mutex m_mutex;
    std::condition_variable m_inventoryMonitor;
};

} // namespace OpenSim

#endif // MOCO_MOCOUTILITIES_H
