/* --------------------------------------------------------------------------*
 *                             OpenSim:  Ground.cpp                           *
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
#include "Ground.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

const char* Ground::GroundNameString{"ground"};

//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/*
 * Default constructor.
 */
Ground::Ground() : PhysicalFrame() {
    setName(GroundNameString);
    setAuthors("Ajay Seth");
}

/* Ground should never change name */
void Ground::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();
    // Ground is always named "ground"
    if (getName() != GroundNameString) {
        setName(GroundNameString);
        log_info("'{}' was renamed and is being reset to '{}'.",
                getConcreteClassName(), getName());
    }
}

/*
 * Implementation of Frame interface by Ground.
 */
SimTK::Transform Ground::calcTransformInGround(const SimTK::State& s) const {
    return SimTK::Transform();
}

SimTK::SpatialVec Ground::calcVelocityInGround(const SimTK::State& s) const {
    return SimTK::SpatialVec(0);
}

/** The spatial acceleration {alpha; a} for this Frame in ground */
SimTK::SpatialVec Ground::calcAccelerationInGround(
        const SimTK::State& s) const {
    return SimTK::SpatialVec(0);
}

void Ground::extendAddToSystem(SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    setMobilizedBodyIndex(SimTK::GroundIndex);
}
