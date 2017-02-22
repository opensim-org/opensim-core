/* -------------------------------------------------------------------------- *
 *                     OpenSim:  PointToPointActuator.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Matt S. DeMers                                                  *
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

/* 
 * Author: Matt DeMers
 */

//==============================================================================
// INCLUDES
//==============================================================================
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>

#include "PointToPointActuator.h"

using namespace OpenSim;
using std::string;
using SimTK::Vec3; using SimTK::Vector_; using SimTK::Vector; 
using SimTK::SpatialVec; using SimTK::UnitVec3; using SimTK::State;

//==============================================================================
// CONSTRUCTOR(S)
//==============================================================================
// default destructor and copy construction & assignment
//_____________________________________________________________________________
// Default constructor.
PointToPointActuator::PointToPointActuator()
{
    constructProperties();
}
//_____________________________________________________________________________
// Constructor with given body names.
PointToPointActuator::PointToPointActuator(const string& bodyNameA, 
                                           const string& bodyNameB)
{
    constructProperties();

    if (!bodyNameA.empty()) set_bodyA(bodyNameA);
    if (!bodyNameB.empty()) set_bodyB(bodyNameB);
}

//_____________________________________________________________________________
// Construct and initialize properties.
void PointToPointActuator::constructProperties()
{
    constructProperty_bodyA();
    constructProperty_bodyB();
    constructProperty_points_are_global(false);
    constructProperty_pointA(Vec3(0));  // origin
    constructProperty_pointB(Vec3(0));
    constructProperty_optimal_force(1.0);
}


//==============================================================================
// GET AND SET
//==============================================================================
//-----------------------------------------------------------------------------
// BodyID
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the generalized Body to which the Body actuator is applied.
 *
 * @param aBody Pointer to the generalized Body.
 */
void PointToPointActuator::setBodyA(Body* aBody)
{
    _bodyA = aBody;
    if(aBody)
        set_bodyA(aBody->getName());
}
//_____________________________________________________________________________
/**
 * Set the generalized Body to which the equal and opposite Body actuation 
 * is applied.
 *
 * @param aBody Pointer to the generalized Body.
 */
void PointToPointActuator::setBodyB(Body* aBody)
{
    _bodyB = aBody;
    if(aBody)
        set_bodyB(aBody->getName());
}


//==============================================================================
// COMPUTATIONS
//==============================================================================
//_____________________________________________________________________________
/**
 * Get the stress of the force. This would be the force or torque provided by 
 * this actuator divided by its optimal force.
 * @return Stress.
 */
double PointToPointActuator::getStress( const SimTK::State& s) const
{
    return std::abs(getActuation(s) / getOptimalForce()); 
}
//_____________________________________________________________________________
/**
 * Compute all quantities necessary for applying the actuator force to the
 * model.
 *
 * @param s current SimTK::State 
 */

double PointToPointActuator::computeActuation( const SimTK::State& s ) const
{
    if(!_model) return 0;

    // FORCE
    return getControl(s) * getOptimalForce();
}



//==============================================================================
// APPLICATION
//==============================================================================
//_____________________________________________________________________________
/**
 * Apply the actuator force to BodyA and BodyB.
 *
 * @param s current SimTK::State
 */
void PointToPointActuator::computeForce(const SimTK::State& s, 
                                SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                                SimTK::Vector& generalizedForces) const
{
    const bool pointsAreGlobal = getPointsAreGlobal();
    const SimTK::Vec3& pointA = getPointA();
    const SimTK::Vec3& pointB = getPointB();

    if(!_model) return;
    
    if( !_bodyA || !_bodyB )
        return;
    
    // Get pointA and pointB positions in both the global frame, and in 
    // the local frame of bodyA and bodyB, respectively. Points may have
    // been supplied either way.

    SimTK::Vec3 pointA_inGround, pointB_inGround, 
                pointA_inBodyA, pointB_inBodyB;
    Ground ground = getModel().getGround();

    if (pointsAreGlobal)
    {
        pointA_inGround = pointA;
        pointB_inGround = pointB;
        pointA_inBodyA = ground.findStationLocationInAnotherFrame(s, pointA_inGround, *_bodyA);
        pointB_inBodyB = ground.findStationLocationInAnotherFrame(s, pointB_inGround, *_bodyB);
    }
    else
    {
        pointA_inBodyA = pointA;
        pointB_inBodyB = pointB;
        pointA_inGround = _bodyA->findStationLocationInGround(s, pointA_inBodyA);
        pointB_inGround = _bodyB->findStationLocationInGround(s, pointB_inBodyB);
    }

    // Find the direction along which the actuator applies its force.
    // NOTE: this will fail if the points are coincident.
    const SimTK::Vec3 r = pointA_inGround - pointB_inGround;
    const SimTK::UnitVec3 direction(r); // normalize

    // Find the force magnitude and set it. Then form the force vector.
    double forceMagnitude;

    if (isActuationOverridden(s)) {
        forceMagnitude = computeOverrideActuation(s);
    } else {
       forceMagnitude = computeActuation(s);
    }
    setActuation(s, forceMagnitude);

    const SimTK::Vec3 force = forceMagnitude*direction;

    // Apply equal and opposite forces to the bodies.
    applyForceToPoint(s, *_bodyA, pointA_inBodyA, force, bodyForces);
    applyForceToPoint(s, *_bodyB, pointB_inBodyB, -force, bodyForces);

    // Get the relative velocity of the points in ground.
    SimTK::Vec3 velA_G =  _bodyA->findStationVelocityInGround(s, pointA_inBodyA);
    SimTK::Vec3 velB_G = _bodyB->findStationVelocityInGround(s, pointB_inBodyB);
    SimTK::Vec3 velAB_G = velA_G-velB_G;
    // Speed used to compute power is the speed along the line connecting 
    // the two bodies.
    setSpeed(s, ~velAB_G*direction);
}
//_____________________________________________________________________________
/**
 * Sets the actual Body references _bodyA and _bodyB
 */
void PointToPointActuator::extendConnectToModel(Model& model)
{
    Super::extendConnectToModel(model);

    if (get_bodyA().empty() || get_bodyB().empty())
        throw OpenSim::Exception(
            "PointToPointActuator::extendConnectToModel(): body name properties "
            "were not set.");

    // Look up the bodies by name in the Model, and record pointers to the
    // corresponding body objects.
    _bodyA = &updModel().updBodySet().get(get_bodyA());
    _bodyB = &updModel().updBodySet().get(get_bodyB());
}
