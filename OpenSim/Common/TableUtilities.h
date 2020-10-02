#ifndef OPENSIM_TABLEUTILITIES_H_
#define OPENSIM_TABLEUTILITIES_H_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  TableUtilities.h                        *
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

#include "Array.h"
#include "GCVSpline.h"
#include "TableUtilities.h"
#include "TimeSeriesTable.h"
#include "osimCommonDLL.h"

namespace OpenSim {

class OSIMCOMMON_API TableUtilities {
public:
    /// Throws an exception if the same label appears more than once in the list
    /// of labels.
    /// @throws NonUniqueLabels
    static void checkNonUniqueLabels(std::vector<std::string> labels);

    /// Returns true if the table contains 'inDegrees' metadata set to 'yes',
    /// and returns false if the table contains 'inDegrees' metadata set to
    /// 'no'.
    /// @throws Exception if table does not have 'inDegrees' table metadata.
    /// @throws Exception if the 'inDegrees' metadata is neither 'yes' or 'no'.
    static bool isInDegrees(const TimeSeriesTable& table);

    /// Get the index in the provided array of labels that corresponds to the
    /// desired label. This function attempts to handle the change in
    /// state variable names that occurred in OpenSim version 4.0; for example,
    /// if you search for `<coord-name>/speed` and it is not found, then this
    /// function looks for `<coord-name>_u`. If you search for
    /// `<muscle>/activation` and it is not found, then this function looks for
    /// `<muscle>.activation`. This function returns -1 if the desired label is
    /// not found.
    static int findStateLabelIndex(
            const Array<std::string>& labels, const std::string& desired);

    /// @copydoc findStateLabelIndex()
    static int findStateLabelIndex(
            const std::vector<std::string>& labels, const std::string& desired);

    /// Lowpass filter the data in a TimeSeriesTable at a provided cutoff
    /// frequency. If padData is true, then the data is first padded with pad()
    /// using numRowsToPrependAndAppend = table.getNumRows() / 2.
    /// The filtering is performed with Signal::LowpassIIR()
    static void filterLowpass(TimeSeriesTable& table,
            double cutoffFreq, bool padData = false);

    /// Pad each column by the number of rows specified. The padded data is
    /// obtained by reflecting and negating the data in the table.
    /// Postcondition: the number of rows is table.getNumRows() + 2 *
    /// numRowsToPrependAndAppend.
    static void pad(TimeSeriesTable& table, int numRowsToPrependAndAppend);

    /// Resample (interpolate) the table at the provided times. In general, a
    /// 5th-order GCVSpline is used as the interpolant; a lower order is used if
    /// the table has too few points for a 5th-order spline. Alternatively, you
    /// can provide a different function type as a template argument; currently,
    /// the only other supported function is PiecewiseLinearFunction.
    /// @throws Exception if new times are
    /// not within existing initial and final times, if the new times are
    /// decreasing, or if getNumTimes() < 2.
    template <typename TimeVector, typename FunctionType = GCVSpline>
    static TimeSeriesTable resample(
            const TimeSeriesTable& in, const TimeVector& newTime);

    /// Resample the table using the given time interval (using resample()).
    /// The new final time is not guaranteed to match the original final
    /// time.
    template <typename FunctionType = GCVSpline>
    static TimeSeriesTable resampleWithInterval(
            const TimeSeriesTable& in, double interval);

    /// Same as resampleWithInterval() but the interval may be reduced
    /// to ensure the number of sampling points does not exceed
    /// Storage::MAX_RESAMPLE_SIZE.
    template <typename FunctionType = GCVSpline>
    static TimeSeriesTable resampleWithIntervalBounded(
            const TimeSeriesTable& in, double interval);

    // Utility to convert TimeSeriesTable of Rotations to a 
    // corresponding TimeSeriesTableVec3 of BodyFixedXYZ Euler angles
    static TimeSeriesTable_<SimTK::Vec3> convertRotationsToEulerAngles(
            const TimeSeriesTable_<SimTK::Rotation>& rotTable);

private:
    static int findStateLabelIndexInternal(const std::string* begin,
            const std::string* end, const std::string& desired);
};

} // namespace OpenSim

#endif // OPENSIM_TABLEUTILITIES_H_
