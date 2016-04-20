/* -------------------------------------------------------------------------- *
 *                          OpenSim:  PathPoint.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "PathPoint.h"
#include "BodySet.h"
#include "Model.h"
#include "GeometryPath.h"
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h> 
#include <OpenSim/Simulation/Wrap/WrapObject.h>
#include <OpenSim/Simulation/Model/Geometry.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;
using SimTK::Transform;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/* Copy data members from one PathPoint to another.
 *
 * @param aPoint PathPoint to be copied.
 */
void PathPoint::copyData(const PathPoint &aPoint)
{
    _location = aPoint._location;
    _path = aPoint._path;
}

PathPoint* PathPoint::makePathPointOfType(PathPoint* aPoint, const string& aNewTypeName)
{
    PathPoint* newPoint = NULL;

    if (aPoint != NULL) {
        Object* newObject = Object::newInstanceOfType(aNewTypeName);
        if (newObject) {
            newPoint = dynamic_cast<PathPoint*>(newObject);
            if (newPoint) {
                // Copy the contents from aPoint.
                newPoint->init(*aPoint);
            }
        }
    }

    return newPoint;
}

