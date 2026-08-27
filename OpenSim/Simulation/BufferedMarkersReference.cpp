/* -------------------------------------------------------------------------- *
 *                   OpenSim:  BufferedMarkersReference.cpp                   *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2026 Stanford University and the Authors                *
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

namespace OpenSim {

BufferedMarkersReference::BufferedMarkersReference()
        : Object() {
    setAuthors("Selim Gilon");
}

BufferedMarkersReference::BufferedMarkersReference(
        const TimeSeriesTable_<SimTK::Vec3>& markerData,
        const Set<MarkerWeight>& markerWeightSet, Units units)
        : Object(), _seedReference(markerData, markerWeightSet, units) {
    setAuthors("Selim Gilon");
}

double BufferedMarkersReference::getNextValuesAndTime(
        SimTK::Array_<SimTK::Vec3>& values) {

    double returnTime;
    SimTK::RowVector_<SimTK::Vec3> nextRow;
    _markerDataQueue.pop_front(returnTime, nextRow);
    int n = nextRow.size();
    values.resize(n);

    for (int i = 0; i < n; ++i) { values[i] = nextRow[i]; }
    return returnTime;
}

void BufferedMarkersReference::putValues(
        double time, const SimTK::RowVector_<SimTK::Vec3>& dataRow) {
    OPENSIM_THROW_IF(dataRow.size() != _seedReference.getNumRefs(), Exception,
            "BufferedMarkersReference::putValues(): expected {} marker "
            "values, but received {}.",
            _seedReference.getNumRefs(), dataRow.size());
    _markerDataQueue.push_back(time, dataRow);
}

} // end of namespace OpenSim
