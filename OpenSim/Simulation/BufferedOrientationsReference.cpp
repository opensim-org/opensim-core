/* -------------------------------------------------------------------------- *
 *                   OpenSim:  BufferedOrientationsReference.cpp              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2020 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
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
#include "BufferedOrientationsReference.h"
#include "OrientationsReference.h"
#include <OpenSim/Common/Units.h>
#include <OpenSim/Common/TRCFileAdapter.h>
#include <SimTKcommon/internal/State.h>

using namespace std;
using namespace SimTK;

namespace OpenSim {

BufferedOrientationsReference::BufferedOrientationsReference()
        : OrientationsReference() {
    setAuthors("Ayman Habib");
}

BufferedOrientationsReference::BufferedOrientationsReference(const std::string& orientationFile, Units modelUnits)
        : OrientationsReference(orientationFile, modelUnits) {
}


BufferedOrientationsReference::BufferedOrientationsReference(
        const TimeSeriesTable_<Rotation>& orientationData,
        const Set<OrientationWeight>* orientationWeightSet)
        : OrientationsReference(orientationData, orientationWeightSet) {
    // Copy data from TimeSeriesTable into Queue for future reference
    auto times = orientationData.getIndependentColumn();
    for (int i = 0; i < orientationData.getNumRows(); ++i) {
        _orientationDataQueue.push_back(times[i], orientationData.getRowAtIndex(i));
    }
}
/** get the values of the OrientationsReference */
void BufferedOrientationsReference::getValues(
        const SimTK::State& s,
    SimTK::Array_<Rotation> &values) const
{
    double time =  s.getTime();
    /**
    // get values for time
    SimTK::RowVector_<Rotation> row = _orientationData.getRow(time);

    int n = row.size();
    values.resize(n);

    for (int i = 0; i < n; ++i) {
        values[i] = row[i];
    }*/
}

} // end of namespace OpenSim