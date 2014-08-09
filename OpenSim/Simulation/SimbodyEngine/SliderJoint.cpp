/* -------------------------------------------------------------------------- *
 *                         OpenSim:  SliderJoint.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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
#include "SliderJoint.h"
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <OpenSim/Simulation/Model/Model.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//_____________________________________________________________________________
/**
 * Default constructor.
 */
SliderJoint::SliderJoint() :
    Joint()
{
    setAuthors("Ajay Seth");
    constructCoordinates();

    const CoordinateSet& coordinateSet = get_CoordinateSet();
    coordinateSet[0].setMotionType(Coordinate::Translational);
}

//_____________________________________________________________________________
/**
 * Convenience Constructor.
 */
SliderJoint::SliderJoint(const std::string &name, const Body& parent,
                         const SimTK::Vec3& locationInParent, const SimTK::Vec3& orientationInParent,
                         const Body& child,
                         const SimTK::Vec3& locationInchild, const SimTK::Vec3& orientationInChild,
                         bool reverse) :
    Super(name, parent, locationInParent, orientationInParent,
          child, locationInchild, orientationInChild, reverse)
{
    setAuthors("Ajay Seth");
    constructCoordinates();

    const CoordinateSet& coordinateSet = get_CoordinateSet();
    coordinateSet[0].setMotionType(Coordinate::Translational);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================


//=============================================================================
// Simbody Model building.
//=============================================================================
//_____________________________________________________________________________
void SliderJoint::addToSystem(SimTK::MultibodySystem& system) const
{
    createMobilizedBody<SimTK::MobilizedBody::Slider>(system);

    // TODO: Joints require super class to be called last.
    Super::addToSystem(system);
}

