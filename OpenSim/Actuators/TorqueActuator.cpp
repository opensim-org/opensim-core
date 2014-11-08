/* -------------------------------------------------------------------------- *
 *                        OpenSim:  TorqueActuator.cpp                        *
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

/* 
 * Author: Ajay Seth
 */

//==============================================================================
// INCLUDES
//==============================================================================
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>

#include "TorqueActuator.h"

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
TorqueActuator::TorqueActuator()
{
    constructProperties();
}
//_____________________________________________________________________________
// Constructor with given body names.
TorqueActuator::TorqueActuator(const Body& bodyA, const Body& bodyB,
                   const SimTK::Vec3& axis, bool axisInGround)
{
    constructProperties();

    setBodyA(bodyA);
    setBodyB(bodyB);

    set_axis(axis);
    set_torque_is_global(axisInGround);
}

//_____________________________________________________________________________
// Construct and initialize properties.
void TorqueActuator::constructProperties()
{
    setAuthors("Ajay Seth, Matt DeMers");
    constructProperty_bodyA();
    constructProperty_bodyB();
    constructProperty_torque_is_global(true);
    constructProperty_axis(Vec3(0,0,1)); // z direction
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
void TorqueActuator::setBodyA(const Body& aBody)
{
    _bodyA = &aBody;
    set_bodyA(aBody.getName());
}
//_____________________________________________________________________________
/**
 * Set the generalized Body to which the equal and opposite Body actuation 
 * is applied.
 *
 * @param aBody Pointer to the generalized Body.
 */
void TorqueActuator::setBodyB(const Body& aBody)
{
    _bodyB = &aBody;
    set_bodyB(aBody.getName());
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
double TorqueActuator::getStress(const State& s) const
{
    return std::abs(getActuation(s) / getOptimalForce());
}
//_____________________________________________________________________________
/**
 * Compute all quantities necessary for applying the actuator force to the
 * model.
 */
double TorqueActuator::computeActuation(const State& s) const
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
 */
void TorqueActuator::computeForce(const State& s, 
                                  Vector_<SpatialVec>& bodyForces, 
                                  Vector& generalizedForces) const
{
    if(!_model) return;
    const SimbodyEngine& engine = getModel().getSimbodyEngine();

    const bool torqueIsGlobal = getTorqueIsGlobal();
    const Vec3& axis = getAxis();
    
    double actuation = 0;

    if (isActuationOverriden(s)) {
        actuation = computeOverrideActuation(s);
    } else {
        actuation = computeActuation(s);
    }
    setActuation(s, actuation);

    if(!_bodyA)
        return;
    
    setActuation(s, actuation);
    Vec3 torque = actuation * UnitVec3(axis);
    
    if (!torqueIsGlobal)
        engine.transform(s, *_bodyA, torque, engine.getGroundBody(), torque);
    
    applyTorque(s, *_bodyA, torque, bodyForces);

    // if bodyB is not specified, use the ground body by default
    if(_bodyB)
        applyTorque(s, *_bodyB, -torque, bodyForces);

    // get the angular velocity of the body in ground
    Vec3 omegaA(0), omegaB(0);
    engine.getAngularVelocity(s, *_bodyA, omegaA);
    engine.getAngularVelocity(s, *_bodyB, omegaB);
    // the "speed" is the relative angular velocity of the bodies
    // projected onto the torque axis.
    setSpeed(s, ~(omegaA-omegaB)*axis);
}
//_____________________________________________________________________________
/**
 * Sets the actual Body references _bodyA and _bodyB
 */
void TorqueActuator::extendConnectToModel(Model& model)
{
    Super::extendConnectToModel(model);

    if (get_bodyA().empty() || get_bodyB().empty())
        throw OpenSim::Exception(
            "TorqueActuator::extendConnectToModel(): body name properties "
            "were not set.");

    // Look up the bodies by name in the Model, and record pointers to the
    // corresponding body objects.
    _bodyA = model.updBodySet().get(get_bodyA());
    _bodyB = model.updBodySet().get(get_bodyB());
}

//==============================================================================
// XML
//==============================================================================
//-----------------------------------------------------------------------------
// UPDATE FROM XML NODE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Update this object based on its XML node.
 *
 * This method simply calls Object::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber) and then calls
 * a few methods in this class to ensure that variable members have been
 * set in a consistent manner.
 */
void TorqueActuator::
updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    int documentVersion = versionNumber;
    bool converting=false;
    if ( documentVersion < XMLDocument::getLatestVersion()){
        if (documentVersion<10905){
            // This used to be called "Force" back then
            XMLDocument::renameChildNode(aNode, "body_A", "bodyB"); // body_B -> body
            XMLDocument::renameChildNode(aNode, "body_B", "bodyA"); // direction_A -> direction
            XMLDocument::renameChildNode(aNode, "direction_A", "axis"); // direction_A -> direction
            converting = true;
        }
    }
    Super::updateFromXMLNode(aNode, versionNumber);
    if (converting) upd_axis() *= -1.0;
}   

