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

/** get the values of the OrientationsReference */
void BufferedOrientationsReference::getValuesAtTime(
        double time, SimTK::Array_<Rotation> &values) const
{
    auto& times = _orientationData.getIndependentColumn();
    SimTK::RowVector_<SimTK::Rotation> nextRow;

    if (time >= times.front() && time <= times.back()) {
        nextRow = _orientationData.getRow(time);
    } else {
        _orientationDataQueue.pop_front(time, nextRow);
    }
    int n = nextRow.size();
    values.resize(n);

    for (int i = 0; i < n; ++i) { 
        values[i] = nextRow[i];
    }
}

double BufferedOrientationsReference::getNextValuesAndTime(
        SimTK::Array_<SimTK::Rotation_<double>>& values) {

    double returnTime;
    SimTK::RowVector_<SimTK::Rotation> nextRow;
    _orientationDataQueue.pop_front(returnTime, nextRow);
    int n = nextRow.size();
    values.resize(n);

    for (int i = 0; i < n; ++i) { values[i] = nextRow[i]; }
    return returnTime;
}

void BufferedOrientationsReference::putValues(
        double time, const SimTK::RowVector_<SimTK::Rotation_<double>>& dataRow) {
    _orientationDataQueue.push_back(time, dataRow);
}
} // end of namespace OpenSim