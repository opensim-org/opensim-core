/* -------------------------------------------------------------------------- *
 *                   OpenSim:  BufferedMarkersReference.cpp                   *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2023 Stanford University and the Authors                *
 * Author(s): Selim Gilon                                                     *
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
#include "BufferedMarkersReference.h"
#include "MarkersReference.h"
#include <OpenSim/Common/Units.h>
#include <OpenSim/Common/TRCFileAdapter.h>
#include <SimTKcommon/internal/State.h>

using namespace std;
using namespace SimTK;

namespace OpenSim {

BufferedMarkersReference::BufferedMarkersReference()
        : MarkersReference() {
    setAuthors("Selim Gilon");
    // Don't set column labels in constructor - they will be set when first data is added
}

BufferedMarkersReference::BufferedMarkersReference(
        const TimeSeriesTable_<SimTK::Vec3>& markerData,
        const Set<MarkerWeight>& markerWeightSet,
        Units units)
        : MarkersReference(markerData, markerWeightSet, units) {
    setAuthors("Selim Gilon");
}

/** get the values of the MarkersReference */
void BufferedMarkersReference::getValuesAtTime(
        double time, SimTK::Array_<Vec3>& values) const
{
    // First try to get values from the base MarkersReference table
    auto& times = getMarkerTable().getIndependentColumn();
    SimTK::RowVector_<SimTK::Vec3> nextRow;

    // Check if the time is within the base table range
    if (times.size() > 0 && time >= times.front() && time <= times.back()) {
        nextRow = getMarkerTable().getRow(time);
    } else {
        // If not in the base table, get from the buffer using TimeSeriesTable
        // This supports time-based lookup without blocking
        auto& bufferTimes = _markerBuffer.getIndependentColumn();
        if (bufferTimes.size() > 0 && time >= bufferTimes.front() && time <= bufferTimes.back()) {
            nextRow = _markerBuffer.getRow(time);
        } else {
            // If time is not in buffer, return empty values
            nextRow = SimTK::RowVector_<SimTK::Vec3>(0);
        }
    }
    
    int n = nextRow.size();
    values.resize(n);

    for (int i = 0; i < n; ++i) { 
        values[i] = nextRow[i];
    }
}

double BufferedMarkersReference::getNextValuesAndTime(
        SimTK::Array_<SimTK::Vec3>& values) {

    auto& bufferTimes = _markerBuffer.getIndependentColumn();
    if (bufferTimes.size() == 0) {
        values.resize(0);
        return NaN;
    }

    double returnTime = bufferTimes.front();
    SimTK::RowVector_<SimTK::Vec3> nextRow = _markerBuffer.getRowAtIndex(0);
    
    int n = nextRow.size();
    values.resize(n);

    for (int i = 0; i < n; ++i) { 
        values[i] = nextRow[i]; 
    }
    
    // Remove the first row after getting it (FIFO behavior)
    _markerBuffer.removeRowAtIndex(0);
    
    return returnTime;
}

void BufferedMarkersReference::putValues(
        double time, const SimTK::RowVector_<SimTK::Vec3>& dataRow) {
    // Check if we need to set column labels
    if (!_markerBuffer.hasColumnLabels()) {
        // Try to get from base table first
        const auto& baseLabels = getMarkerTable().getColumnLabels();
        std::cout << "[DEBUG] BufferedMarkersReference::putValues: baseLabels.size() = " << baseLabels.size() << std::endl;
        if (baseLabels.size() > 0) {
            _markerBuffer.setColumnLabels(baseLabels);
            std::cout << "[DEBUG] BufferedMarkersReference::putValues: Set column labels from base table" << std::endl;
        } else {
            // Fallback: create generic labels based on dataRow size
            std::vector<std::string> labels;
            for (int i = 0; i < dataRow.size(); ++i) {
                labels.push_back("marker" + std::to_string(i));
            }
            _markerBuffer.setColumnLabels(labels);
            std::cout << "[DEBUG] BufferedMarkersReference::putValues: Set generic column labels, size = " << labels.size() << std::endl;
        }
    }
    
    _markerBuffer.appendRow(time, dataRow);
}

} // end of namespace OpenSim 