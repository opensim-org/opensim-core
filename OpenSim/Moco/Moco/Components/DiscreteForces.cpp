/* -------------------------------------------------------------------------- *
 * OpenSim Moco: DiscreteForces.cpp                                           *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "DiscreteForces.h"
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

void DiscreteForces::extendAddToSystem(SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);

    SimTK::SubsystemIndex forcesIdx = 
        getModel().getForceSubsystem().getMySubsystemIndex();
    SimTK::ForceSubsystem& forces = 
        SimTK::ForceSubsystem::updDowncast(system.updSubsystem(forcesIdx));
    m_discrete_forces = SimTK::Force::DiscreteForces(
        SimTK::GeneralForceSubsystem::updDowncast(forces),
        system.getMatterSubsystem());
}

void DiscreteForces::setAllForces(SimTK::State& s, 
        const SimTK::Vector& generalizedForces,
        const SimTK::Vector_<SimTK::SpatialVec>& bodyForcesInG) const {

    m_discrete_forces.setAllMobilityForces(s, generalizedForces);
    m_discrete_forces.setAllBodyForces(s, bodyForcesInG);
}
