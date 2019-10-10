#ifndef OPENSIM_COMMONUTILITIES_H_
#define OPENSIM_COMMONUTILITIES_H_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  CommonUtilities.h                       *
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

#include "FunctionSet.h"
#include "GCVSplineSet.h"
#include "TimeSeriesTable.h"
#include "osimCommonDLL.h"
#include <iostream>

namespace OpenSim {

/// Since OpenSim does not require C++14 (which contains std::make_unique()),
/// we provide an implementation of make_unique().
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
OSIMCOMMON_API std::string getFormattedDateTime(bool appendMicroseconds = false,
        std::string format = "%Y-%m-%dT%H%M%S");

#ifndef SWIG
/// Create a FunctionSet from a table for any Function that can be constructed
/// from arrays of independent and dependent data.
template <typename FunctionType>
std::unique_ptr<FunctionSet> createFunctionSetFromTable(
        const TimeSeriesTable& table) {
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

/// For each column in the table, create a 5-th order spline, or a lower-order
/// spline if the table has fewer than 5 rows.
template <>
inline std::unique_ptr<FunctionSet> createFunctionSetFromTable<GCVSpline>(
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
template <typename TimeVector, typename FunctionType = GCVSpline>
TimeSeriesTable resampleTable(
        const TimeSeriesTable& in, const TimeVector& newTime) {

    const auto& time = in.getIndependentColumn();

    OPENSIM_THROW_IF(time.size() < 2, Exception,
            "Cannot resample if number of times is 0 or 1.");
    OPENSIM_THROW_IF(newTime[0] < time[0], Exception,
            format("New initial time (%f) cannot be less than existing "
                   "initial time (%f)",
                    newTime[0], time[0]));
    OPENSIM_THROW_IF(newTime[newTime.size() - 1] > time[time.size() - 1],
            Exception,
            format("New final time (%f) cannot be greater than existing final "
                   "time (%f)",
                    newTime[newTime.size() - 1], time[time.size() - 1]));
    for (int itime = 1; itime < (int)newTime.size(); ++itime) {
        OPENSIM_THROW_IF(newTime[itime] < newTime[itime - 1], Exception,
                format("New times must be non-decreasing, but "
                       "time[%i] < time[%i] (%f < %f).",
                        itime, itime - 1, newTime[itime], newTime[itime - 1]));
    }

    // Copy over metadata.
    TimeSeriesTable out = in;
    for (int irow = (int)out.getNumRows() - 1; irow >= 0; --irow) {
        out.removeRowAtIndex(irow);
    }

    std::unique_ptr<FunctionSet> functions =
            createFunctionSetFromTable<FunctionType>(in);
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

/// Resample the table using the provided time interval.
template <typename FunctionType = GCVSpline>
TimeSeriesTable resampleTable(const TimeSeriesTable& in, double interval) {
    std::vector<double> time;
    for (double t = in.getFirstTime(); t <= in.getLastTime(); t += interval) {
        time.push_back(t);
    }
    return resampleTable(in, time);
}

/** Get the index in labels of the specified state name. This
 * function attempts to handle the change in state variable names that
 * occurred in OpenSim version 4.0; for example, if you search for
 * `<coord-name>/speed` and it is not found, then this function looks for
 * `<coord-name>_u`.
 *
 * @return Index of name or -1. */
OSIMCOMMON_API int getStateIndex(
        const SimTK::Array_<std::string>& labels, const std::string& name);

/// @copydoc getStateIndex(const SimTK::Array_<std::string>&, const std::string&);
inline int getStateIndex(
        const OpenSim::Array<std::string>& labels, const std::string& name) {
    return getStateIndex(
            SimTK::ArrayViewConst_<std::string>(labels.cbegin(), labels.cend()),
            name);
}

/// @copydoc getStateIndex(const SimTK::Array_<std::string>&, const std::string&);
inline int getStateIndex(
        const std::vector<std::string>& labels, const std::string& name) {
    return getStateIndex(SimTK::ArrayViewConst_<std::string>(labels), name);
}

} // namespace OpenSim

#endif // OPENSIM_COMMONUTILITIES_H_
