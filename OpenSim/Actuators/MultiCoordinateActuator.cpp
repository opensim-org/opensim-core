/* -------------------------------------------------------------------------- *
 *                      OpenSim:  MultiCoordinateActuator.cpp                 *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2014 Stanford University and the Authors                *
 * Author(s): Chris Dembia                                                    *
 * Contributor(s): Soha Pouya, Michael Sherman                                *
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

//==============================================================================
// INCLUDES
//==============================================================================

#include "MultiCoordinateActuator.h"
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;
using namespace std;

MultiCoordinateActuator::MultiCoordinateActuator() : Actuator()
{
    constructInfrastructure();
}

void MultiCoordinateActuator::constructProperties()
{
}

void MultiCoordinateActuator::computeForce( const SimTK::State& s, 
							   SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							   SimTK::Vector& mobilityForces) const
{
    if(!_model) return;

    mobilityForces = getControls(s);

    /* TODO
    SimTK::VectorView_<double> actuatorControls = getControls(s);
    const CoordinateSet& coordSet = getModel().getCoordinateSet();
    for (unsigned int iCoord = 0; iCoord < coordSet.getSize(); iCoord++)
    {
        applyGeneralizedForce(s, coordSet[iCoord],
                actuatorControls[coordSet[iCoord].getMobilizerQIndex()],
                mobilityForces);
    }
    */
}

int MultiCoordinateActuator::numControls() const 
{
    return getModel().getCoordinateSet().getSize();
}

void MultiCoordinateActuator::connectToModel(Model& aModel)
{
	Super::connectToModel(aModel);
}

void MultiCoordinateActuator::addToSystem(SimTK::MultibodySystem& system) const
{
     Super::addToSystem(system);
}


