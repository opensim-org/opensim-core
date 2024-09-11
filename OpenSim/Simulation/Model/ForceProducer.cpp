/* -------------------------------------------------------------------------- *
 *                       OpenSim: ForceProducer.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2024 Stanford University and the Authors                *
 * Author(s): Adam Kewley                                                     *
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

#include "ForceProducer.h"

#include <OpenSim/Simulation/Model/ForceApplier.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

void OpenSim::ForceProducer::computeForce(
    const SimTK::State& state,
    SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
    SimTK::Vector& generalizedForces) const
{
    if (!appliesForce(state)) {
        // This `Force` has explicitly stated that it doesn't want to apply
        // the forces.
        return;
    }

    // create a consumer that uses each produced force to compute the
    // underlying body- and generalized-forces
    ForceApplier forceApplier{&_model->getMatterSubsystem(), &bodyForces, &generalizedForces};

    // produce forces and feed them into the consumer, satisfying the `computeForce` API
    produceForces(state, forceApplier);
}
