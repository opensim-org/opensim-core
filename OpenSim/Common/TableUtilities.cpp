/* -------------------------------------------------------------------------- *
 *                          OpenSim:  TableUtilities.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2020 Stanford University and the Authors                *
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

#include "TableUtilities.h"

#include "CommonUtilities.h"
#include "FunctionSet.h"
#include "GCVSplineSet.h"
#include "PiecewiseLinearFunction.h"
#include "Signal.h"
#include "Storage.h"

using namespace OpenSim;

void TableUtilities::checkNonUniqueLabels(std::vector<std::string> labels) {
    std::sort(labels.begin(), labels.end());
    auto it = std::adjacent_find(labels.begin(), labels.end());
    OPENSIM_THROW_IF(it != labels.end(), NonUniqueLabels,
            "Label '{}' appears more than once.", *it);
}

bool TableUtilities::isInDegrees(const TimeSeriesTable& table) {
    OPENSIM_THROW_IF(!table.hasTableMetaDataKey("inDegrees"), Exception,
            "Table does not have 'inDegrees' metadata.");
    const std::string inDegrees =
            table.getTableMetaData<std::string>("inDegrees");
    OPENSIM_THROW_IF(inDegrees != "yes" && inDegrees != "no", Exception,
            "Expected table's 'inDegrees' metadata to be 'yes' or 'no', "
            "but got '{}'.",
            inDegrees);
    return inDegrees == "yes";
}

int TableUtilities::findStateLabelIndex(
        const Array<std::string>& labels, const std::string& desired) {
    return findStateLabelIndexInternal(
            labels.get(), labels.get() + labels.getSize(), desired);
}

int TableUtilities::findStateLabelIndex(
        const std::vector<std::string>& labels, const std::string& desired) {
    return findStateLabelIndexInternal(
            labels.data(), labels.data() + labels.size(), desired);
}

int TableUtilities::findStateLabelIndexInternal(const std::string* begin,
        const std::string* end, const std::string& desired) {

    auto found = std::find(begin, end, desired);
    if (found != end) return (int)std::distance(begin, found);

    // 4.0 and its beta versions differ slightly in the absolute path but
    // the <joint>/<coordinate>/value (or speed) will be common to both.
    // Likewise, for muscle states <muscle>/activation (or fiber_length)
    // must be common to the state variable (path) name and column label.
    std::string shortPath = desired;
    std::string::size_type front = shortPath.find('/');
    while (found == end && front < std::string::npos) {
        shortPath = shortPath.substr(front + 1, desired.length());
        found = std::find(begin, end, shortPath);
        front = shortPath.find('/');
    }
    if (found != end) return (int)std::distance(begin, found);

    // Assume column labels follow pre-v4.0 state variable labeling.
    // Redo search with what the pre-v4.0 label might have been.

    // First, try just the last element of the path.
    std::string::size_type back = desired.rfind('/');
    std::string prefix = desired.substr(0, back);
    std::string shortName = desired.substr(back + 1, desired.length() - back);
    found = std::find(begin, end, shortName);
    if (found != end) return (int)std::distance(begin, found);

    // If that didn't work, specifically check for coordinate state names
    // (<coord_name>/value and <coord_name>/speed) and muscle state names
    // (<muscle_name>/activation <muscle_name>/fiber_length).
    if (shortName == "value") {
        // pre-v4.0 did not have "/value" so remove it if here
        back = prefix.rfind('/');
        shortName = prefix.substr(back + 1, prefix.length());
        found = std::find(begin, end, shortName);
    } else if (shortName == "speed") {
        // replace "/speed" (the v4.0 labeling for speeds) with "_u"
        back = prefix.rfind('/');
        shortName = prefix.substr(back + 1, prefix.length() - back) + "_u";
        found = std::find(begin, end, shortName);
    } else if (back < desired.length()) {
        // try replacing the '/' with '.' in the last segment
        shortName = desired;
        shortName.replace(back, 1, ".");
        back = shortName.rfind('/');
        shortName = shortName.substr(back + 1, shortName.length() - back);
        found = std::find(begin, end, shortName);
    }
    if (found != end) return (int)std::distance(begin, found);

    // If all of the above checks failed, return -1.
    return -1;
}

void TableUtilities::filterLowpass(
        TimeSeriesTable& table, double cutoffFreq, bool padData) {
    OPENSIM_THROW_IF(cutoffFreq < 0, Exception,
            "Cutoff frequency must be non-negative; got {}.", cutoffFreq);

    if (padData) { pad(table, (int)table.getNumRows() / 2); }

    const int numRows = (int)table.getNumRows();
    OPENSIM_THROW_IF(numRows < 4, Exception,
            "Expected at least 4 rows to filter, but got {} rows.", numRows);

    const auto& time = table.getIndependentColumn();

    double dtMin = SimTK::Infinity;
    for (int irow = 1; irow < numRows; ++irow) {
        double dt = time[irow] - time[irow - 1];
        if (dt < dtMin) dtMin = dt;
    }
    OPENSIM_THROW_IF(
            dtMin < SimTK::Eps, Exception, "Storage cannot be resampled.");

    double dtAvg = (time.back() - time.front()) / (numRows - 1);

    // Resample if the sampling interval is not uniform.
    if (dtAvg - dtMin > SimTK::Eps) {
        table = resampleWithInterval(table, dtMin);
    }

    SimTK::Vector filtered(numRows);
    for (int icol = 0; icol < (int)table.getNumColumns(); ++icol) {
        SimTK::VectorView column = table.getDependentColumnAtIndex(icol);
        Signal::LowpassIIR(dtMin, cutoffFreq, numRows,
                column.getContiguousScalarData(),
                filtered.updContiguousScalarData());
        table.updDependentColumnAtIndex(icol) = filtered;
    }
}

void TableUtilities::pad(
        TimeSeriesTable& table, int numRowsToPrependAndAppend) {
    if (numRowsToPrependAndAppend == 0) return;

    OPENSIM_THROW_IF(numRowsToPrependAndAppend < 0, Exception,
            "Expected numRowsToPrependAndAppend to be non-negative; but "
            "got {}.",
            numRowsToPrependAndAppend);

    table._indData = Signal::Pad(numRowsToPrependAndAppend,
            (int)table._indData.size(), table._indData.data());

    size_t numColumns = table.getNumColumns();

    // _indData.size() is now the number of rows after padding.
    SimTK::Matrix newMatrix((int)table._indData.size(), (int)numColumns);
    for (size_t icol = 0; icol < numColumns; ++icol) {
        SimTK::Vector column = table.getDependentColumnAtIndex(icol);
        const std::vector<double> newColumn =
                Signal::Pad(numRowsToPrependAndAppend, column.size(),
                        column.getContiguousScalarData());
        newMatrix.updCol((int)icol) =
                SimTK::Vector((int)newColumn.size(), newColumn.data(), true);
    }
    table.updMatrix() = newMatrix;
}

namespace {
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

template <>
inline std::unique_ptr<FunctionSet> createFunctionSet<GCVSpline>(
        const TimeSeriesTable& table) {
    const auto& time = table.getIndependentColumn();
    return OpenSim::make_unique<GCVSplineSet>(table, std::vector<std::string>{},
            std::min((int)time.size() - 1, 5));
}
} // namespace

/// Resample (interpolate) the table at the provided times. In general, a
/// 5th-order GCVSpline is used as the interpolant; a lower order is used if the
/// table has too few points for a 5th-order spline. Alternatively, you can
/// provide a different function type as a template argument (e.g.,
/// PiecewiseLinearFunction).
/// @throws Exception if new times are
/// not within existing initial and final times, if the new times are
/// decreasing, or if getNumTimes() < 2.
/// @ingroup moconumutil
template <typename TimeVector, typename FunctionType>
TimeSeriesTable TableUtilities::resample(
        const TimeSeriesTable& in, const TimeVector& newTime) {

    const auto& time = in.getIndependentColumn();

    OPENSIM_THROW_IF(time.size() < 2, Exception,
            "Cannot resample if number of times is 0 or 1.");
    OPENSIM_THROW_IF(newTime[0] < time[0], Exception,
            "New initial time ({}) cannot be less than existing initial time "
            "({})",
            newTime[0], time[0]);
    OPENSIM_THROW_IF(newTime[newTime.size() - 1] > time[time.size() - 1],
            Exception,
            "New final time ({}) cannot be greater than existing final time "
            "({})",
            newTime[newTime.size() - 1], time[time.size() - 1]);
    for (int itime = 1; itime < (int)newTime.size(); ++itime) {
        OPENSIM_THROW_IF(newTime[itime] < newTime[itime - 1], Exception,
                "New times must be non-decreasing, but "
                "time[{}] < time[{}] ({} < {}).",
                itime, itime - 1, newTime[itime], newTime[itime - 1]);
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

template <typename FunctionType>
TimeSeriesTable TableUtilities::resampleWithInterval(
        const TimeSeriesTable& in, double interval) {
    std::vector<double> time;
    double t = in.getIndependentColumn().front();
    double finalTime = in.getIndependentColumn().back();
    while (t <= finalTime) {
        time.push_back(t);
        t += interval;
    }
    return resample<std::vector<double>, FunctionType>(in, time);
}

template <typename FunctionType>
TimeSeriesTable TableUtilities::resampleWithIntervalBounded(
        const TimeSeriesTable& in, double interval) {
    const auto& time = in.getIndependentColumn();
    double duration = time.back() - time.front();
    if (duration / interval > Storage::MAX_RESAMPLE_SIZE) {
        double newInterval = duration / Storage::MAX_RESAMPLE_SIZE;
        log_warn("Requested resampling time interval {} leads to more than {} "
                 "sampling points; using larger time interval {}.",
                interval, Storage::MAX_RESAMPLE_SIZE, newInterval);
        interval = newInterval;
    }
    return resampleWithInterval<FunctionType>(in, interval);
}
TimeSeriesTable_<SimTK::Vec3> TableUtilities::convertRotationsToEulerAngles(
        const TimeSeriesTable_<SimTK::Rotation>& rotTable) {
    auto labels = rotTable.getColumnLabels();
    auto& times = rotTable.getIndependentColumn();
    const auto& rotations = rotTable.getMatrix();

    int nc = int(labels.size());
    int nt = int(times.size());

    SimTK::Matrix_<SimTK::Vec3> eulerMatrix(nt, nc, SimTK::Vec3(SimTK::NaN));

    for (int i = 0; i < nt; ++i) {
        for (int j = 0; j < nc; ++j) {
            eulerMatrix.updElt(i, j) =
                    rotations(i, j).convertRotationToBodyFixedXYZ();
        }
    }
    TimeSeriesTable_<SimTK::Vec3> eulerData{times, eulerMatrix, labels};
    eulerData.updTableMetaData().setValueForKey(
            "Units", std::string("Radians"));

    return eulerData;
}
// Explicit template instantiations.
namespace OpenSim {
template OSIMCOMMON_API TimeSeriesTable TableUtilities::resample<SimTK::Vector, GCVSpline>(
        const TimeSeriesTable&, const SimTK::Vector&);
template OSIMCOMMON_API TimeSeriesTable
TableUtilities::resample<SimTK::Vector, PiecewiseLinearFunction>(
        const TimeSeriesTable&, const SimTK::Vector&);

template OSIMCOMMON_API TimeSeriesTable
TableUtilities::resample<std::vector<double>, GCVSpline>(
        const TimeSeriesTable&, const std::vector<double>&);
template OSIMCOMMON_API TimeSeriesTable
TableUtilities::resample<std::vector<double>, PiecewiseLinearFunction>(
        const TimeSeriesTable&, const std::vector<double>&);

template OSIMCOMMON_API TimeSeriesTable TableUtilities::resampleWithInterval<GCVSpline>(
        const TimeSeriesTable&, double);
template OSIMCOMMON_API TimeSeriesTable
TableUtilities::resampleWithInterval<PiecewiseLinearFunction>(
        const TimeSeriesTable&, double);

template OSIMCOMMON_API TimeSeriesTable TableUtilities::resampleWithIntervalBounded<GCVSpline>(
        const TimeSeriesTable&, double);
template OSIMCOMMON_API TimeSeriesTable
TableUtilities::resampleWithIntervalBounded<PiecewiseLinearFunction>(
        const TimeSeriesTable&, double);
} // namespace OpenSim
