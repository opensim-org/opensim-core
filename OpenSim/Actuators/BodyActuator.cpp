/* ------------------------------------------------------------------------- *
*                       OpenSim:  BodyActuator.cpp                           *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include <OpenSim/Simulation/Model/Model.h>

#include "BodyActuator.h"

using namespace OpenSim;
using namespace std;
using SimTK::Vec3;


//=============================================================================
// CONSTRUCTORS
//=============================================================================
// Uses default (compiler-generated) destructor, copy constructor, copy 
// assignment operator.
//_____________________________________________________________________________
/**
* Default constructor.
*/
BodyActuator::BodyActuator()
{
    setAuthors("Soha Pouya, Michael Sherman");
    constructProperties();

}
//_____________________________________________________________________________
/**
* Convenience constructor.
*/
BodyActuator::BodyActuator(const Body& body, 
                           const SimTK::Vec3& point,
                           bool pointIsGlobal,
                           bool spatialForceIsGlobal)
{
    setAuthors("Soha Pouya, Michael Sherman");
    constructProperties();

    connectSocket_body(body);

    set_point(point); // origin
    set_point_is_global(pointIsGlobal);
    set_spatial_force_is_global(spatialForceIsGlobal);
}

void BodyActuator::constructProperties()
{
    constructProperty_point(Vec3(0)); // origin
    constructProperty_point_is_global(false);
    constructProperty_spatial_force_is_global(true);
}

void BodyActuator::setBodyName(const std::string& name)
{
    updSocket<Body>("body").setConnecteePath(name);
}

const std::string& BodyActuator::getBodyName() const
{
    return getSocket<Body>("body").getConnecteePath();
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
* Set the Body to which the BodyActuator is applied
*/
void BodyActuator::setBody(const Body& body)
{
    connectSocket_body(body);
}

/**
* Get the Body to which the BodyActuator is applied
*/
const Body& BodyActuator::getBody() const
{
    return getConnectee<Body>("body");
}

//==============================================================================
// APPLICATION
//==============================================================================
//_____________________________________________________________________________
/**
* Apply the actuator force/torque to Body.
*/
void BodyActuator::computeForce(const SimTK::State& s,
                                SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
                                SimTK::Vector& generalizedForces) const
{
    if (!_model) return;

    const bool spatialForceIsGlobal = getSpatialForceIsGlobal();
    
    const Body& body = getBody();
    // const SimTK::MobilizedBody& body_mb = body.getMobilizedBody();

    Vec3 pointOfApplication = get_point(); 

    // get the control signals
    const SimTK::Vector bodyForceVals = getControls(s);

    // Read spatialForces which should be in ground frame by default
    Vec3 torqueVec(bodyForceVals[0], bodyForceVals[1],
        bodyForceVals[2]);
    Vec3 forceVec(bodyForceVals[3], bodyForceVals[4],
        bodyForceVals[5]);

    // if the user has given the spatialForces in body frame, transform them to
    // global (ground) frame
    if (!spatialForceIsGlobal){
        torqueVec = body.expressVectorInGround(s, torqueVec);
        forceVec = body.expressVectorInGround(s, forceVec);
    }

    // if the point of applying force is not in body frame (which is the default 
    // case) transform it to body frame
    if (get_point_is_global())
        pointOfApplication = getModel().getGround().
            findStationLocationInAnotherFrame(s, pointOfApplication, body);

    applyTorque(s, body, torqueVec, bodyForces);
    applyForceToPoint(s, body, pointOfApplication, forceVec, bodyForces);

}

//_____________________________________________________________________________
/**
* Compute power consumed by moving the body via applied spatial force.
* Reads the body spatial velocity vector and spatial force vector applied via
* BodyActuator and computes the power as p = F (dotProdcut) V.
*/
double BodyActuator::getPower(const SimTK::State& s) const
{
    const Body& body = getBody();

    const SimTK::MobilizedBody& body_mb = body.getMobilizedBody();
    SimTK::SpatialVec bodySpatialVelocities = body_mb.getBodyVelocity(s);

    SimTK::Vector bodyVelocityVec(6);
    bodyVelocityVec[0] = bodySpatialVelocities[0][0];
    bodyVelocityVec[1] = bodySpatialVelocities[0][1];
    bodyVelocityVec[2] = bodySpatialVelocities[0][2];
    bodyVelocityVec[3] = bodySpatialVelocities[1][0];
    bodyVelocityVec[4] = bodySpatialVelocities[1][1];
    bodyVelocityVec[5] = bodySpatialVelocities[1][2];

    const SimTK::Vector bodyForceVals = getControls(s);

    double power = ~bodyForceVals * bodyVelocityVec;

    return power;
}


