#ifndef OPENSIM_FORCE_ADAPTER_H_
#define OPENSIM_FORCE_ADAPTER_H_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  ForceAdapter.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s):   Peter Eastman                                                 *
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

// INCLUDES
#include "OpenSim/Simulation/osimSimulationDLL.h"
#include "Force.h"
#include "Actuator.h"

#include <SimTKsimbody.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * This acts as an adapter to allow a Force or Actuator to be used as a SimTK::Force.
 *
 * @authors Peter Eastman
 */
class OSIMSIMULATION_API ForceAdapter : public SimTK::Force::Custom::Implementation
{
//=============================================================================
// DATA
//=============================================================================
private:
    const Force* _force;

//=============================================================================
// METHODS
//=============================================================================
public:
    // CONSTRUCTION AND DESTRUCTION
    ForceAdapter(const Force& force);

    // CALC FORCES (Called by Simbody)
    void calcForce(const SimTK::State& state,
        SimTK::Vector_<SimTK::SpatialVec>& bodyForces,SimTK::Vector_<SimTK::Vec3>& particleForces,
        SimTK::Vector& mobilityForces) const override;   

    // CALC POTENTIAL ENERGY (Called by Simbody)
    SimTK::Real calcPotentialEnergy(const SimTK::State& state) const override;   

    // SIMBODY PARALLELISM FLAG 
    bool shouldBeParallelized() const;

    // No need to override realize() methods; we don't provide that service
    // to OpenSim Force elements.
};

} // end of namespace OpenSim

#endif // OPENSIM_FORCE_ADAPTER_H_
