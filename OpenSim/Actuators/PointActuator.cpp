/* -------------------------------------------------------------------------- *
 *                        OpenSim:  PointActuator.cpp                         *
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


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>

#include "PointActuator.h"

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
PointActuator::PointActuator(const string& bodyName)
{
    setNull();
    constructProperties();

    if (!bodyName.empty())
        set_body(bodyName);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
// Set the data members of this actuator to their null values.
void PointActuator::setNull()
{
    setAuthors("Ajay Seth");
}

//_____________________________________________________________________________
// Allocate and initialize properties.
void PointActuator::constructProperties()
{
    constructProperty_body();
    constructProperty_point(Vec3(0)); // origin
    constructProperty_point_is_global(false);
    constructProperty_direction(Vec3(1,0,0)); // arbitrary
    constructProperty_force_is_global(false);
    constructProperty_optimal_force(1.0);
}

//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// BodyID
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
// Set the generalized Body to which the Body actuator is applied, and set
// the body name property to match.
void PointActuator::setBody(Body* aBody)
{
    _body = aBody;
    if(aBody)
        set_body(aBody->getName());
}
//_____________________________________________________________________________
// Get the generalized Body to which the Body actuator is applied.
Body* PointActuator::getBody() const
{
    return _body;
}

//-----------------------------------------------------------------------------
// OPTIMAL FORCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
//Set the optimal force of the force.

void PointActuator::setOptimalForce(double aOptimalForce)
{
    set_optimal_force(aOptimalForce);
}
//_____________________________________________________________________________
// Get the optimal force of the force.
double PointActuator::getOptimalForce() const
{
    return get_optimal_force();
}
//_____________________________________________________________________________
// Get the stress of the force. This would be the force or torque provided by 
// this actuator divided by its optimal force.
double PointActuator::getStress( const SimTK::State& s) const
{
    return std::abs(getActuation(s) / getOptimalForce()); 
}


//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute all quantities necessary for applying the actuator force to the
 * model.
 */
double PointActuator::computeActuation( const SimTK::State& s ) const
{
    if(!_model) return 0;

    // FORCE
    return getControl(s) * getOptimalForce();
}



//=============================================================================
// APPLICATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Apply the actuator force to BodyA and BodyB.
 */
void PointActuator::computeForce(const SimTK::State& s, 
                                 SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                                 SimTK::Vector& generalizedForces) const
{
    const SimbodyEngine& engine = getModel().getSimbodyEngine();

    if( !_model || !_body ) return;

    double force;

    if (isActuationOverriden(s)) {
        force = computeOverrideActuation(s);
    } else {
       force = computeActuation(s);
    }
    setActuation(s, force);

    
    Vec3 forceVec = force*SimTK::UnitVec3(get_direction());
    Vec3 lpoint = get_point();
    if (!get_force_is_global())
        engine.transform(s, *_body, forceVec, 
                         getModel().getGround(), forceVec);
    if (get_point_is_global())
        engine.transformPosition(s, getModel().getGround(), lpoint, 
                                 *_body, lpoint);
    applyForceToPoint(s, *_body, lpoint, forceVec, bodyForces);

    // get the velocity of the actuator in ground
    Vec3 velocity(0);
    engine.getVelocity(s, *_body, lpoint, velocity);

    // the speed of the point is the "speed" of the actuator used to compute 
    // power
    setSpeed(s, velocity.norm());
}
//_____________________________________________________________________________
/**
 * Sets the actual Body reference _body
 */
void PointActuator::extendConnectToModel(Model& model)
{
    Super::extendConnectToModel(model);

    string errorMessage;

    const string& bodyName = get_body();

    if (!model.updBodySet().contains(bodyName)) {
        errorMessage = "PointActuator: Unknown body (" + bodyName 
                        + ") specified in Actuator " + getName();
        throw OpenSim::Exception(errorMessage);
    }

    _body = &model.updBodySet().get(bodyName);
}


//=============================================================================
// XML
//=============================================================================
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
void PointActuator::
updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    int documentVersion = versionNumber;
    bool converting=false;
    if ( documentVersion < XMLDocument::getLatestVersion()){
        if (documentVersion<10905){
            // This used to be called "Force" back then
            XMLDocument::renameChildNode(aNode, "body_B", "body"); // body_B -> body
            XMLDocument::renameChildNode(aNode, "point_B", "point"); // point_B -> point
            XMLDocument::renameChildNode(aNode, "direction_A", "direction"); // direction_A -> direction
            set_force_is_global(true);
            converting = true;
        }
    }
    Super::updateFromXMLNode(aNode, versionNumber);
    if (converting) upd_direction() *= -1.0;

    if (_model && !get_body().empty()) {
        const std::string& bodyName = get_body();
        _body = &_model->updBodySet().get(bodyName);
    }
}   

