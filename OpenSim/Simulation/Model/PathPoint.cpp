/* -------------------------------------------------------------------------- *
 *                           OpenSim:  PathPoint.cpp                          *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "PathPoint.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;
using SimTK::Transform;

PathPoint::PathPoint() : Super() 
{
    constructProperties();
}

void PathPoint::constructProperties()
{
    //Default location
    SimTK::Vec3 origin(0.0, 0.0, 0.0);
    // Location in Body 
    constructProperty_location(origin);
}

void PathPoint::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();
    updStation().upd_location() = get_location();
    updStation().updSocket("parent_frame").
        setConnecteeName(updSocket("parent_frame").getConnecteeName());
}

void PathPoint::setLocation(const SimTK::Vec3& location) {
    upd_location() = location;
    updStation().upd_location() = location;
}

void PathPoint::extendConnectToModel(Model& model)
{
    Super::extendConnectToModel(model);
    // connect the underlying Station to the PathPoint's parent_frame
    updStation().setParentFrame(getParentFrame());
}

void PathPoint::
changeBodyPreserveLocation(const SimTK::State& s, const PhysicalFrame& frame)
{
    if (!hasOwner()) {
        throw Exception("PathPoint::changeBodyPreserveLocation attempted to "
            " change the frame of PathPoint which was not assigned to a frame.");
    }
    // if it is already assigned to body, do nothing
    const PhysicalFrame& currentFrame = getStation().getParentFrame();

    if (currentFrame == frame)
        return;

    // Preserve location means to switch bodies without changing
    // the location of the point in the inertial reference frame.
    setLocation(
        currentFrame.findStationLocationInAnotherFrame(s, 
            getStation().get_location(), frame) );

    // now make frame this PathPoint's parent Frame
    setParentFrame(frame);
}

void PathPoint::scale(const SimTK::Vec3& scaleFactors) {
    auto& loc = upd_location();
    loc = loc.elementwiseMultiply(scaleFactors);
    updStation().upd_location() = loc;
}
