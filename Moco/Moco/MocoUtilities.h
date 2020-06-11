#ifndef MOCO_MOCOUTILITIES_H
#define MOCO_MOCOUTILITIES_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoUtilities.h                                              *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia, Nicholas Bianco                             *
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

/// The utilities in this file are categorized as follows:
///   - generic utilities (Doxygen group mocogenutil),
///   - string and logging utilities (mocologutil),
///   - numeric and data utilities (moconumutil), and
///   - model and trajectory utilities (mocomodelutil).
/// When adding a new function to this file, make sure to add it to one of the
/// groups above.

#include "MocoTrajectory.h"
#include "osimMocoDLL.h"
#include <Common/Reporter.h>
#include <Simulation/Model/Model.h>
#include <Simulation/StatesTrajectory.h>
#include <condition_variable>
#include <regex>
#include <set>
#include <stack>

#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/PiecewiseLinearFunction.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/Logger.h>

namespace OpenSim {

class StatesTrajectory;
class Model;
class MocoTrajectory;
class MocoProblem;

/// Since Moco does not require C++14 (which contains std::make_unique()),
/// here is an implementation of make_unique().
/// @ingroup mocogenutil
template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

/// Get a string with the current date and time formatted as %Y-%m-%dT%H%M%S
/// (year, month, day, "T", hour, minute, second). You can change the datetime
/// format via the `format` parameter.
/// If you specify "ISO", then we use the ISO 8601 extended datetime format
/// %Y-%m-%dT%H:%M:%S.
/// See https://en.cppreference.com/w/cpp/io/manip/put_time.
/// @ingroup mocogenutil
OSIMMOCO_API std::string getMocoFormattedDateTime(
        bool appendMicroseconds = false,
        std::string format = "%Y-%m-%dT%H%M%S");

/// Determine if `string` starts with the substring `start`.
/// https://stackoverflow.com/questions/874134/find-if-string-ends-with-another-string-in-c
/// @ingroup mocogenutil
inline bool startsWith(const std::string& string, const std::string& start) {
    if (string.length() >= start.length()) {
        return string.compare(0, start.length(), start) == 0;
    }
    return false;
}

/// Determine if `string` ends with the substring `ending`.
/// https://stackoverflow.com/questions/874134/find-if-string-ends-with-another-string-in-c
/// @ingroup mocogenutil
inline bool endsWith(const std::string& string, const std::string& ending) {
    if (string.length() >= ending.length()) {
        return string.compare(string.length() - ending.length(),
                       ending.length(), ending) == 0;
    }
    return false;
}

/// An OpenSim XML file may contain file paths that are relative to the
/// directory containing the XML file; use this function to convert that
/// relative path into an absolute path.
/// @ingroup mocogenutil
OSIMMOCO_API
std::string getAbsolutePathnameFromXMLDocument(
        const std::string& documentFileName,
        const std::string& pathnameRelativeToDocument);

/// This class stores the formatting of a stream and restores that format
/// when the StreamFormat is destructed.
/// @ingroup mocologutil
class StreamFormat {
public:
    StreamFormat(std::ostream& stream) : m_stream(stream) {
        m_format.copyfmt(stream);
    }
    ~StreamFormat() { m_stream.copyfmt(m_format); }

private:
    std::ostream& m_stream;
    std::ios m_format{nullptr};
}; // StreamFormat

/// Create a SimTK::Vector with the provided length whose elements are
/// linearly spaced between start and end.
/// @ingroup moconumutil
OSIMMOCO_API
SimTK::Vector createVectorLinspace(int length, double start, double end);

#ifndef SWIG
/// Create a SimTK::Vector using modern C++ syntax.
/// @ingroup moconumutil
OSIMMOCO_API
SimTK::Vector createVector(std::initializer_list<SimTK::Real> elements);
#endif

/// Linearly interpolate y(x) at new values of x. The optional 'ignoreNaNs'
/// argument will ignore any NaN values contained in the input vectors and
/// create the interpolant from the non-NaN values only. Note that this option
/// does not necessarily prevent NaN values from being returned in 'newX', which
/// will have NaN for any values of newX outside of the range of x.
/// @throws Exception if x and y are different sizes, or x or y is empty.
/// @ingroup moconumutil
OSIMMOCO_API
SimTK::Vector interpolate(const SimTK::Vector& x, const SimTK::Vector& y,
        const SimTK::Vector& newX, const bool ignoreNaNs = false);

#ifndef SWIG
/// @ingroup moconumutil
template <typename FunctionType>
std::unique_ptr<FunctionSet> createFunctionSet(const TimeSeriesTable& table) {
    auto set = make_unique<FunctionSet>();
    const auto& time = table.getIndependentColumn();
    const auto numRows = (int)table.getNumRows();
    for (int icol = 0; icol < (int)table.getNumColumns(); ++icol) {
        const double* y =
                table.getDependentColumnAtIndex(icol).getContiguousScalarData();
        set->adoptAndAppend(new FunctionType(numRows, time.data(), y));
    }
    return set;
}

/// @ingroup moconumutil
template <>
inline std::unique_ptr<FunctionSet> createFunctionSet<GCVSpline>(
        const TimeSeriesTable& table) {
    const auto& time = table.getIndependentColumn();
    return std::unique_ptr<GCVSplineSet>(new GCVSplineSet(table,
            std::vector<std::string>{}, std::min((int)time.size() - 1, 5)));
}
#endif // SWIG

/// Resample (interpolate) the table at the provided times. In general, a
/// 5th-order GCVSpline is used as the interpolant; a lower order is used if the
/// table has too few points for a 5th-order spline. Alternatively, you can
/// provide a different function type as a template argument (e.g.,
/// PiecewiseLinearFunction).
/// @throws Exception if new times are
/// not within existing initial and final times, if the new times are
/// decreasing, or if getNumTimes() < 2.
/// @ingroup moconumutil
template <typename TimeVector, typename FunctionType = GCVSpline>
TimeSeriesTable resample(const TimeSeriesTable& in, const TimeVector& newTime) {

    const auto& time = in.getIndependentColumn();

    OPENSIM_THROW_IF(time.size() < 2, Exception,
            "Cannot resample if number of times is 0 or 1.");
    OPENSIM_THROW_IF(newTime[0] < time[0], Exception,
            fmt::format("New initial time ({}) cannot be less than existing "
                   "initial time ({})",
                    newTime[0], time[0]));
    OPENSIM_THROW_IF(newTime[newTime.size() - 1] > time[time.size() - 1],
            Exception,
            fmt::format("New final time ({}) cannot be greater than existing "
                        "final time ({})",
                    newTime[newTime.size() - 1], time[time.size() - 1]));
    for (int itime = 1; itime < (int)newTime.size(); ++itime) {
        OPENSIM_THROW_IF(newTime[itime] < newTime[itime - 1], Exception,
                fmt::format("New times must be non-decreasing, but "
                       "time[{}] < time[{}] ({} < {}).",
                        itime, itime - 1, newTime[itime], newTime[itime - 1]));
    }

    // Copy over metadata.
    TimeSeriesTable out = in;
    for (int irow = (int)out.getNumRows() - 1; irow >= 0; --irow) {
        out.removeRowAtIndex(irow);
    }

    std::unique_ptr<FunctionSet> functions =
            createFunctionSet<FunctionType>(in);
    SimTK::Vector curTime(1);
    SimTK::RowVector row(functions->getSize());
    for (int itime = 0; itime < (int)newTime.size(); ++itime) {
        curTime[0] = newTime[itime];
        for (int icol = 0; icol < functions->getSize(); ++icol) {
            row(icol) = functions->get(icol).calcValue(curTime);
        }
        // Not efficient!
        out.appendRow(curTime[0], row);
    }
    return out;
}

/// Create a Storage from a TimeSeriesTable. Metadata from the
/// TimeSeriesTable is *not* copied to the Storage.
/// You should use TimeSeriesTable if possible, as support for Storage may be
/// reduced in future versions of OpenSim. However, Storage supports some
/// operations not supported by TimeSeriesTable (e.g., filtering, resampling).
// TODO move to the Storage class.
/// @ingroup moconumutil
OSIMMOCO_API Storage convertTableToStorage(const TimeSeriesTable&);

/// Update a vector of state labels (in place) to use post-4.0 state paths
/// instead of pre-4.0 state names. For example, this converts labels as
/// follows:
///   - `pelvis_tilt` -> `/jointset/ground_pelvis/pelvis_tilt/value`
///   - `pelvis_tilt_u` -> `/jointset/ground_pelvis/pelvis_tilt/speed`
///   - `soleus.activation` -> `/forceset/soleus/activation`
///   - `soleus.fiber_length` -> `/forceset/soleus/fiber_length`
/// This can also be used to update the column labels of an Inverse Kinematics
/// Tool solution MOT file so that the data can be used as states. If a label
/// does not identify a state in the model, the column label is not changed.
/// @throws Exception if labels are not unique.
OSIMMOCO_API void updateStateLabels40(
        const Model& model, std::vector<std::string>& labels);

/// Lowpass filter the data in a TimeSeriesTable at a provided cutoff frequency.
/// The table is converted to a Storage object to use the lowpassIIR() method
/// to filter, and then converted back to TimeSeriesTable.
/// @ingroup moconumutil
OSIMMOCO_API TimeSeriesTable filterLowpass(
        const TimeSeriesTable& table, double cutoffFreq, bool padData = false);

/// Write a single TimeSeriesTable to a file, using the FileAdapter associated
/// with the provided file extension.
/// @ingroup moconumutil
OSIMMOCO_API void writeTableToFile(const TimeSeriesTable&, const std::string&);

/// Play back a motion (from the Storage) in the simbody-visuailzer. The Storage
/// should contain all generalized coordinates. The visualizer window allows the
/// user to control playback speed.
/// This function blocks until the user exits the simbody-visualizer window.
/// @ingroup mocomodelutil
// TODO handle degrees.
OSIMMOCO_API void visualize(Model, Storage);

/// This function is the same as visualize(Model, Storage), except that
/// the states are provided in a TimeSeriesTable.
/// @ingroup mocomodelutil
OSIMMOCO_API void visualize(Model, TimeSeriesTable);

/// Calculate the requested outputs using the model in the problem and the
/// states and controls in the MocoTrajectory.
/// The output paths can be regular expressions. For example,
/// ".*activation" gives the activation of all muscles.
/// Constraints are not enforced but prescribed motion (e.g.,
/// PositionMotion) is.
/// The output paths must correspond to outputs that match the type provided in
/// the template argument, otherwise they are not included in the report.
/// @note Parameters and Lagrange multipliers in the MocoTrajectory are **not**
///       applied to the model.
/// @ingroup mocomodelutil
template <typename T>
TimeSeriesTable_<T> analyze(Model model, const MocoTrajectory& trajectory,
        std::vector<std::string> outputPaths) {

    // Initialize the system so we can access the outputs.
    model.initSystem();
    // Create the reporter object to which we'll add the output data to create
    // the report.
    auto* reporter = new TableReporter_<T>();
    // Loop through all the outputs for all components in the model, and if
    // the output path matches one provided in the argument and the output type
    // agrees with the template argument type, add it to the report.
    for (const auto& comp : model.getComponentList()) {
        for (const auto& outputName : comp.getOutputNames()) {
            const auto& output = comp.getOutput(outputName);
            auto thisOutputPath = output.getPathName();
            for (const auto& outputPathArg : outputPaths) {
                if (std::regex_match(
                            thisOutputPath, std::regex(outputPathArg))) {
                    // Make sure the output type agrees with the template.
                    if (dynamic_cast<const Output<T>*>(&output)) {
                        reporter->addToReport(output);
                    } else {
                        log_warn("Ignoring output {} of type {}.",
                                output.getPathName(), output.getTypeName());
                    }
                }
            }
        }
    }
    model.addComponent(reporter);
    model.initSystem();

    // Get states trajectory.
    Storage storage = trajectory.exportToStatesStorage();
    auto statesTraj = StatesTrajectory::createFromStatesStorage(model, storage);

    // Loop through the states trajectory to create the report.
    for (int i = 0; i < (int)statesTraj.getSize(); ++i) {
        // Get the current state.
        auto state = statesTraj[i];

        // Enforce any SimTK::Motion's included in the model.
        model.getSystem().prescribe(state);

        // Create a SimTK::Vector of the control values for the current state.
        SimTK::RowVector controlsRow =
                trajectory.getControlsTrajectory().row(i);
        SimTK::Vector controls(controlsRow.size(),
                controlsRow.getContiguousScalarData(), true);

        // Set the controls on the state object.
        model.realizeVelocity(state);
        model.setControls(state, controls);

        // Generate report results for the current state.
        model.realizeReport(state);
    }

    return reporter->getTable();
}

/// Given a MocoTrajectory and the associated OpenSim model, return the model
/// with a prescribed controller appended that will compute the control values
/// from the MocoSolution. This can be useful when computing state-dependent
/// model quantities that require realization to the Dynamics stage or later.
/// The function used to fit the controls can either be GCVSpline or
/// PiecewiseLinearFunction.
/// @ingroup mocomodelutil
OSIMMOCO_API void prescribeControlsToModel(const MocoTrajectory& trajectory,
        Model& model, std::string functionType = "GCVSpline");

/// Use the controls and initial state in the provided trajectory to simulate
/// the model using an ODE time stepping integrator (OpenSim::Manager), and
/// return the resulting states and controls. We return a MocoTrajectory (rather
/// than a StatesTrajectory) to facilitate comparing optimal control solutions
/// with time stepping. Use integratorAccuracy to override the default setting.
/// @ingroup mocomodelutil
OSIMMOCO_API MocoTrajectory simulateTrajectoryWithTimeStepping(
        const MocoTrajectory& trajectory, Model model,
        double integratorAccuracy = -1);

/// The map provides the index of each state variable in
/// SimTK::State::getY() from its each state variable path string.
/// Empty slots in Y (e.g., for quaternions) are ignored.
/// @ingroup mocomodelutil
OSIMMOCO_API
std::vector<std::string> createStateVariableNamesInSystemOrder(
        const Model& model);

#ifndef SWIG
/// Same as above, but you can obtain a map from the returned state variable
/// names to the index in SimTK::State::getY() that accounts for empty slots
/// in Y.
/// @ingroup mocomodelutil
OSIMMOCO_API
std::vector<std::string> createStateVariableNamesInSystemOrder(
        const Model& model, std::unordered_map<int, int>& yIndexMap);

/// The map provides the index of each state variable in
/// SimTK::State::getY() from its state variable path string.
/// @ingroup mocomodelutil
OSIMMOCO_API
std::unordered_map<std::string, int> createSystemYIndexMap(const Model& model);
#endif

/// Create a vector of control names based on the actuators in the model for
/// which appliesForce == True. For actuators with one control (e.g.
/// ScalarActuator) the control name is simply the actuator name. For actuators
/// with multiple controls, each control name is the actuator name appended by
/// the control index (e.g. "/actuator_0"); modelControlIndices has length equal
/// to the number of controls associated with actuators that apply a force
/// (appliesForce == True). Its elements are the indices of the controls in the
/// Model::updControls() that are associated with actuators that apply a force.
/// @ingroup mocomodelutil
OSIMMOCO_API
std::vector<std::string> createControlNamesFromModel(
        const Model& model, std::vector<int>& modelControlIndices);
/// Same as above, but when there is no mapping to the modelControlIndices.
/// @ingroup mocomodelutil
OSIMMOCO_API
std::vector<std::string> createControlNamesFromModel(const Model& model);
/// The map provides the index of each control variable in the SimTK::Vector
/// return by OpenSim::Model::getControls() from its control name.
/// @ingroup mocomodelutil
OSIMMOCO_API
std::unordered_map<std::string, int> createSystemControlIndexMap(
        const Model& model);

/// Throws an exception if the order of the controls in the model is not the
/// same as the order of the actuators in the model.
/// @ingroup mocomodelutil
OSIMMOCO_API void checkOrderSystemControls(const Model& model);

/// Throws an exception if the same label appears twice in the list of labels.
/// The argument copies the provided labels since we need to sort them to check
/// for redundancies.
/// @ingroup mocomodelutil
OSIMMOCO_API void checkRedundantLabels(std::vector<std::string> labels);

/// Throws an exception if any label in the provided list does not match any
/// state variable names in the model.
OSIMMOCO_API void checkLabelsMatchModelStates(const Model& model,
        const std::vector<std::string>& labels);

/// Get a list of reference pointers to all outputs whose names (not paths)
/// match a substring defined by a provided regex string pattern. The regex
/// string pattern could be the full name of the output. Only Output%s that
/// match the template argument type will be returned (double is the default
/// type). Set the argument 'includeDescendents' to true to include outputs
/// from all descendents from the provided component.
/// @ingroup mocomodelutil
template <typename T = double>
std::vector<SimTK::ReferencePtr<const Output<T>>> getModelOutputReferencePtrs(
        const Component& component, const std::string& pattern,
        bool includeDescendents = false) {

    // Create regex.
    std::regex regex(pattern);
    // Initialize outputs array.
    std::vector<SimTK::ReferencePtr<const Output<T>>> outputs;

    std::function<void(const Component&, const std::regex&, bool,
            std::vector<SimTK::ReferencePtr<const Output<T>>>&)> helper;
    helper = [&helper](const Component& component, const std::regex& regex,
            bool includeDescendents,
            std::vector<SimTK::ReferencePtr<const Output<T>>>& outputs) {
        // Store a reference to outputs that match the template
        // parameter type and whose names contain the provided
        // substring.
        for (const auto& entry : component.getOutputs()) {
            const std::string& name = entry.first;
            const auto foundSubstring = std::regex_match(name, regex);
            const auto* output =
                    dynamic_cast<const Output<T>*>(entry.second.get());
            if (output && foundSubstring) {
                outputs.emplace_back(output);
            }
        }

        // Repeat for all subcomponents.
        if (includeDescendents) {
            for (const Component& thisComp :
                    component.getComponentList<Component>()) {
                if (&thisComp == &component) { continue; }
                helper(thisComp, regex, false, outputs);
            }
        }
    };

    helper(component, regex, includeDescendents, outputs);
    return outputs;
}

/// Convert a trajectory covering half the period of a symmetric motion into a
/// trajectory over the full period. This is useful for simulations of half a
/// gait cycle.
/// This converts time, states, controls, and derivatives; all other quantities
/// from the input trajectory are ignored.
/// If a column in the trajectory does not match addPatterns, negatePatterns,
/// negateAndShiftPatterns, or symmetryPatterns, then the second half of the
/// period contains the same data as the first half.
///
/// @param halfPeriodTrajectory The input trajectory covering half a period.
/// @param addPatterns If a column label matches an addPattern, then the second
/// half of the period for that column is (first_half_trajectory +
/// half_period_value - initial_value).
/// @param negatePatterns If a column label matches a negatePattern, then the
/// second half of the period for that column is (-first_half_trajectory).
/// This is usually relevant for only 3D models.
/// @param negateAndShiftPatterns If a column label matches a
/// negateAndShiftPattern, then the second half of the period for that column is
/// (-first_half_trajectory + 2 * half_period_value). This is usually relevant
/// for only 3D models.
/// @param symmetryPatterns This argument is a list of pairs, where the first
/// element of the pair is a pattern to match, and the second is a substitution
/// to convert the column label into the opposite column label of the symmetric
/// pair. If a column label matches a symmetryPattern, then its first
/// half-period is copied into the second half of the period for the column
/// identified by the substitution.
///
/// The default values for the patterns are intended to handle the column labels
/// for typical 2D or 3D OpenSim gait models.
/// The default values for negatePatterns, negateAndShiftPatterns, and
/// symmetryPatterns warrant an explanation. The string pattern before the
/// regex "(?!/value)" is followed by
/// anything except "/value" since it is contained in the negative lookahead
/// "(?!...)".  R"()" is a string literal that permits us to not escape
/// backslash characters. The regex "_r(\/|_|$)" matches "_r" followed by either
/// a forward slash (which is escaped), an underscore, OR the end of the string
/// ($). Since the forward slash and end of the string are within parentheses,
/// whatever matches this is captured and is available in the substitution (the
/// second element of the pair) as $1. The default symmetry patterns cause the
/// following replacements:
/// - "/jointset/hip_r/hip_flexion_r/value" becomes "/jointset/hip_l/hip_flexion_l/value"
/// - "/forceset/soleus_r" becomes "/forceset/soleus_l"
/// @ingroup mocomodelutil
OSIMMOCO_API MocoTrajectory createPeriodicTrajectory(
        const MocoTrajectory& halfPeriodTrajectory,
        std::vector<std::string> addPatterns = {".*pelvis_tx/value"},
        std::vector<std::string> negatePatterns = {
                                            ".*pelvis_list(?!/value).*",
                                            ".*pelvis_rotation.*",
                                            ".*pelvis_tz(?!/value).*",
                                            ".*lumbar_bending(?!/value).*",
                                            ".*lumbar_rotation.*"},
        std::vector<std::string> negateAndShiftPatterns = {
                                                   ".*pelvis_list/value",
                                                   ".*pelvis_tz/value",
                                                   ".*lumbar_bending/value"},
        std::vector<std::pair<std::string, std::string>> symmetryPatterns =
                {{R"(_r(\/|_|$))", "_l$1"}, {R"(_l(\/|_|$))", "_r$1"}});

/// Throw an exception if the property's value is not in the provided set.
/// We assume that `p` is a single-value property.
/// @ingroup mocogenutil
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
/// @ingroup mocogenutil
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
/// @ingroup mocogenutil
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

/// Record and report elapsed real time ("clock" or "wall" time) in seconds.
/// @ingroup mocogenutil
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
    /// This provides the elapsed time as a formatted string (using formatNs()).
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
/// @ingroup mocogenutil
OSIMMOCO_API int getMocoParallelEnvironmentVariable();

/// This class lets you store objects of a single type for reuse by multiple
/// threads, ensuring threadsafe access to each of those objects.
/// @ingroup mocogenutil
// TODO: Find a way to always give the same thread the same object.
template <typename T> class ThreadsafeJar {
public:
    /// Request an object for your exclusive use on your thread. This function
    /// blocks the thread until an object is available. Make sure to return
    /// (leave()) the object when you're done!
    std::unique_ptr<T> take() {
        // Only one thread can lock the mutex at a time, so only one thread
        // at a time can be in any of the functions of this class.
        std::unique_lock<std::mutex> lock(m_mutex);
        // Block this thread until the condition variable is woken up
        // (by a notify_...()) and the lambda function returns true.
        m_inventoryMonitor.wait(lock, [this] { return m_entries.size() > 0; });
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

/// Thrown by FileDeletionThrower::throwIfDeleted().
/// @ingroup mocogenutil
class FileDeletionThrowerException : public Exception {
public:
    FileDeletionThrowerException(const std::string& file, size_t line,
            const std::string& func, const std::string& deletedFile)
            : Exception(file, line, func) {
        addMessage("File '" + deletedFile + "' deleted.");
    }
};
/// This class helps a user cause an exception within the code. The constructor
/// writes a file, and the destructor deletes the file. The programmer can call
/// throwIfDeleted() to throw the FileDeletionThrowerException exception if the
/// file is deleted (by a user) before the object is destructed. If the file
/// could not be written by the constructor, then throwIfDeleted() does not
/// throw an exception.
/// @ingroup mocogenutil
class FileDeletionThrower {
public:
    FileDeletionThrower()
            : FileDeletionThrower(
                      "OpenSimMoco_delete_this_to_throw_exception_" +
                      getMocoFormattedDateTime() + ".txt") {}
    FileDeletionThrower(std::string filepath)
            : m_filepath(std::move(filepath)) {
        std::ofstream f(m_filepath);
        m_wroteInitialFile = f.good();
        f.close();
    }
    ~FileDeletionThrower() {
        if (m_wroteInitialFile) {
            std::ifstream f(m_filepath);
            if (f.good()) {
                f.close();
                std::remove(m_filepath.c_str());
            }
        }
    }
    void throwIfDeleted() const {
        if (m_wroteInitialFile) {
            OPENSIM_THROW_IF(!std::ifstream(m_filepath).good(),
                    FileDeletionThrowerException, m_filepath);
        }
    }

private:
    bool m_wroteInitialFile = false;
    const std::string m_filepath;
};

/// Obtain the ground reaction forces, centers of pressure, and torques
/// resulting from Force elements (e.g., SmoothSphereHalfSpaceForce), using a
/// model and states trajectory. Forces and torques are expressed in the ground
/// frame with respect to the ground origin. Hence, the centers of pressure are
/// at the origin. Paths to Force elements should be provided separately for
/// elements of the right and left feet. The output is a table formatted for use
/// with OpenSim tools; the labels of the columns distinguish between right
/// ("<>_r") and left ("<>_l") forces, centers of pressure, and torques. The
/// forces and torques used are taken from the first six outputs of
/// getRecordValues(); this order is of use for, for example, the
/// SmoothSphereHalfSpaceForce contact model but might have a different meaning
/// for different contact models.
/// @ingroup mocomodelutil
OSIMMOCO_API
TimeSeriesTable createExternalLoadsTableForGait(Model model,
        const StatesTrajectory& trajectory,
        const std::vector<std::string>& forcePathsRightFoot,
        const std::vector<std::string>& forcePathsLeftFoot);

/// Same as above, but with a MocoTrajectory instead of a StatesTrajectory.
/// @ingroup mocomodelutil
OSIMMOCO_API
TimeSeriesTable createExternalLoadsTableForGait(Model model,
        const MocoTrajectory& trajectory,
        const std::vector<std::string>& forcePathsRightFoot,
        const std::vector<std::string>& forcePathsLeftFoot);

/// Solve for the root of a scalar function using the bisection method.
/// @param calcResidual a function that computes the error
/// @param left lower bound on the root
/// @param right upper bound on the root
/// @param tolerance convergence requires that the bisection's "left" and
///     "right" are less than tolerance apart.
/// @param maxIterations abort after this many iterations.
/// @ingroup mocogenutil
OSIMMOCO_API
SimTK::Real solveBisection(
        std::function<double(const double&)> calcResidual,
        double left, double right, const double& tolerance = 1e-6,
        int maxIterations = 1000);

} // namespace OpenSim

#endif // MOCO_MOCOUTILITIES_H
