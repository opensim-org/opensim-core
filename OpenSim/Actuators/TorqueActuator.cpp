/* -------------------------------------------------------------------------- *
 *                        OpenSim:  TorqueActuator.cpp                        *
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

/* 
 * Author: Ajay Seth
 */

//==============================================================================
// INCLUDES
//==============================================================================
#include "TorqueActuator.h"
#include <OpenSim/Simulation/Model/Model.h>

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
TorqueActuator::TorqueActuator(const PhysicalFrame& bodyA, const PhysicalFrame& bodyB,
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

void TorqueActuator::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    // Cache the computed speed of the actuator
    this->_speedCV = addCacheVariable("speed", 0.0, SimTK::Stage::Velocity);
}

//==============================================================================
// GET AND SET
//==============================================================================
//-----------------------------------------------------------------------------
// BodyID
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/*
 * Set the PhysicalFrame to which the Body actuator is applied.

 */
void TorqueActuator::setBodyA(const PhysicalFrame& aBody)
{
    _bodyA = &aBody;
    set_bodyA(aBody.getName());
}
//_____________________________________________________________________________
/*
 * Set the PhysicalFrame to which the equal and opposite Body actuation 
 * is applied.
 */
void TorqueActuator::setBodyB(const PhysicalFrame& aBody)
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
    if (!_model) {
        return SimTK::NaN;
    }

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
void TorqueActuator::computeForce(
    const State& s,
    Vector_<SpatialVec>& bodyForces,
    Vector& generalizedForces) const
{
    if (!_model || !_bodyA) {
        return;
    }

    const bool torqueIsGlobal = getTorqueIsGlobal();
    const Vec3& axis          = getAxis();

    double actuation = isActuationOverridden(s) ? computeOverrideActuation(s)
                                                : computeActuation(s);
    setActuation(s, actuation);

    Vec3 torque = actuation * UnitVec3(axis);

    if (!torqueIsGlobal) {
        torque = _bodyA->expressVectorInGround(s, torque);
    }

    applyTorque(s, *_bodyA, torque, bodyForces);

    // if bodyB is not specified, use the ground body by default
    if (_bodyB) {
        applyTorque(s, *_bodyB, -torque, bodyForces);
    }
}

double TorqueActuator::getSpeed(const SimTK::State& s) const
{
    if (isCacheVariableValid(s, _speedCV)) {
        return getCacheVariableValue(s, _speedCV);
    }

    double speed = calcSpeed(s);

    updCacheVariableValue(s, _speedCV) = speed;
    markCacheVariableValid(s, _speedCV);
    return speed;
}

double TorqueActuator::calcSpeed(const SimTK::State& s) const
{
    if (!_model || !_bodyA) {
        return SimTK::NaN;
    }

    const bool torqueIsGlobal = getTorqueIsGlobal();
    const Vec3& axis          = SimTK::UnitVec3(getAxis());

    // get the angular velocity of the body in ground
    Vec3 omegaA = _bodyA->getVelocityInGround(s)[0];
    // if bodyB is not specified, use the ground body by default
    Vec3 omegaB =
        _bodyB ? _bodyB->getVelocityInGround(s)[0] : SimTK::Vec3{0., 0., 0.};

    // the "speed" is the relative angular velocity of the bodies
    // projected onto the torque axis.
    double speed = ~(omegaA - omegaB) * axis;
    return speed;
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

    // TODO: Replace this custom lookup with Sockets
    if(getModel().hasComponent<PhysicalFrame>(get_bodyA()))
        _bodyA = &getModel().getComponent<PhysicalFrame>(get_bodyA());
    else
        _bodyA = &getModel().getComponent<PhysicalFrame>("./bodyset/"+get_bodyA());

    if (getModel().hasComponent<PhysicalFrame>(get_bodyB()))
        _bodyB = &getModel().getComponent<PhysicalFrame>(get_bodyB());
    else
        _bodyB = &getModel().getComponent<PhysicalFrame>("./bodyset/" + get_bodyB());
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

