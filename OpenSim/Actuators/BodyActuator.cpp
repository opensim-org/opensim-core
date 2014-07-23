/* -------------------------------------------------------------------------- *
 *                       OpenSim:  BodyActuator.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2014 Stanford University and the Authors                     *
 * Author(s): Soha Pouya, Michael Sherman                                     *
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
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>

#include "BodyActuator.h"

using namespace OpenSim;
using namespace std;
using SimTK::Vec3;


//=============================================================================
// STATICS
//=============================================================================


//=============================================================================
// CONSTRUCTORS
//=============================================================================
// Uses default (compiler-generated) destructor, copy constructor, copy 
// assignment operator.

//_____________________________________________________________________________
/**
 * Also serves as default constructor.
 */
BodyActuator::BodyActuator()
{
	setNull();
    constructInfrastructure();
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
// Set the data members of this actuator to their null values.
void BodyActuator::setNull()
{
	setAuthors("Soha Pouya, Michael Sherman");
}

//_____________________________________________________________________________
// Allocate and initialize properties.
void BodyActuator::constructProperties()
{
}

void BodyActuator::constructStructuralConnectors() {
	constructStructuralConnector<Body>("body");
}

//=============================================================================
// GET AND SET
//=============================================================================


//==============================================================================
// COMPUTATIONS
//==============================================================================


//==============================================================================
// APPLICATION
//==============================================================================
//_____________________________________________________________________________
/**
 * Apply the actuator force to BodyA and BodyB.
 */
void BodyActuator::computeForce(const SimTK::State& s, 
							     SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							     SimTK::Vector& generalizedForces) const
{
	const SimbodyEngine& engine = getModel().getSimbodyEngine();

    //if (!getConnector<Body>("body").isConnected())
     //   printf("OH SHIT!!\n");

	const Body& body = getConnector<Body>("body").getConnectee();

    const SimTK::Vector spatial = getControls(s);
	//spatial.dump("internal actuator controls.");
    const Vec3 torqueVec_B(spatial[0],spatial[1],spatial[2]);
    const Vec3 forceVec_B(spatial[3],spatial[4],spatial[5]);

    Vec3 torqueVec_G, forceVec_G;
    engine.transform(s, body, torqueVec_B, 
                         engine.getGroundBody(), torqueVec_G);
    engine.transform(s, body, forceVec_B, 
                         engine.getGroundBody(), forceVec_G);

	applyTorque(s, body, torqueVec_G, bodyForces);
	applyForceToPoint(s, body, Vec3(0), forceVec_G, bodyForces);

}
//_____________________________________________________________________________
/**
 * Sets the actual Body reference _body
 */
void BodyActuator::connectToModel(Model& model)
{
	Super::connectToModel(model);
}


