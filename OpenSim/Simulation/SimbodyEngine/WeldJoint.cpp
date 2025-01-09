/* -------------------------------------------------------------------------- *
 *                          OpenSim:  WeldJoint.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include "WeldJoint.h"
#include <simbody/internal/MobilizedBody_Weld.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace SimTK;
using namespace OpenSim;

//=============================================================================
// Simbody Model building.
//=============================================================================
//_____________________________________________________________________________
void WeldJoint::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);
    createMobilizedBody<SimTK::MobilizedBody::Weld>(system);
}

/* Specialize Template method for the Weld mobilizer creation*/
namespace OpenSim {
template <> SimTK::MobilizedBody::Weld Joint::createMobilizedBody<SimTK::MobilizedBody::Weld>(
    SimTK::MobilizedBody& inboard,
    const SimTK::Transform& inboardTransform,
    const SimTK::Body& outboard,
    const SimTK::Transform& outboardTransform,
    int& startingCoorinateIndex,
    const PhysicalFrame* associatedBod) const 
{
    // CREATE MOBILIZED BODY
    SimTK::MobilizedBody::Weld simtkBody(inboard, inboardTransform, outboard, outboardTransform);

    startingCoorinateIndex = assignSystemIndicesToBodyAndCoordinates(simtkBody,
        associatedBod,
        0,
        startingCoorinateIndex);

    return simtkBody;
}
} // namespace OpenSim
