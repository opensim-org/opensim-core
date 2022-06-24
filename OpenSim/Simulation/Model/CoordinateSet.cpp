/* -------------------------------------------------------------------------- *
 *                        OpenSim:  CoordinateSet.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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

#include "CoordinateSet.h"
#include <OpenSim/Simulation/SimbodyEngine/Joint.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace std;
using namespace OpenSim;


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
/*
 * Populate a flat list of Coordinates given a Model with Joints.
 */
void CoordinateSet::populate(Model& model)
{
    // Aggregate Coordinates owned by the Joint's into a single CoordinateSet
    setSize(0);
    setMemoryOwner(false);

    auto joints = model.updComponentList<Joint>();

    for(Joint& joint : joints) {
        for(int j=0; j< joint.numCoordinates(); ++j){
            adoptAndAppend(&joint.upd_coordinates(j));
        }
    }
}

void CoordinateSet::getSpeedNames(OpenSim::Array<std::string> &rNames) const {
    for (int i = 0; i<_objects.getSize(); ++i) {
        Coordinate *obj = _objects[i];
        OPENSIM_THROW_IF_FRMOBJ(!obj, Exception,
            "Has a Coordinate that is null.");
        rNames.append(obj->getSpeedName());
    }
}

