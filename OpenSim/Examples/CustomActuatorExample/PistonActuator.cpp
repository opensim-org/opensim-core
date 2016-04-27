/* -------------------------------------------------------------------------- *
 *                        OpenSim:  PistonActuator.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "PistonActuator.h"
#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace std;


//=============================================================================
// STATICS
//=============================================================================


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
PistonActuator::~PistonActuator()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 * @param aBodyNameA name of the first body to which the force is applied
 * @param aBodyNameB name of the second body to which the force is applied
 *
 */
PistonActuator::PistonActuator( string aBodyNameA, string aBodyNameB) :
    ScalarActuator(),
    _bodyA(NULL),
    _bodyB(NULL)
{
    constructProperties();

    if (_model) {
        _bodyA = &_model->updBodySet().get(get_bodyA());
        _bodyB = &_model->updBodySet().get(get_bodyB());
    } 
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this actuator to their null values.
 */
void PistonActuator::setNull()
{
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void PistonActuator::constructProperties()
{
    setAuthors("Matt S. DeMers");
    SimTK::Vec3 x(0.0, 0.0, 0.0);

    constructProperty_bodyA("");
    constructProperty_bodyB("");

    constructProperty_points_are_global(false);
    constructProperty_pointA(x);
    constructProperty_pointB(x);
    constructProperty_optimal_force(1.0);
}

//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// BodyID
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the generalized Body to which the Body actuator is applied.
 *
 * @param aBody Pointer to the generalized Body.
 */
void PistonActuator::setBodyA(Body* aBody)
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
void PistonActuator::setBodyB(Body* aBody)
{
    _bodyB = aBody;
    if(aBody)
        set_bodyB(aBody->getName());
}
//_____________________________________________________________________________
/**
 * Get the generalized Body to which the Body actuator
 * is applied.
 *
 * @return Pointer to the Body
 */
OpenSim::Body* PistonActuator::getBodyA() const
{
    return(_bodyA);
}
//_____________________________________________________________________________
/**
 * Get the generalized Body to which the equal and opposite Body actuation
 * is applied.
 *
 * @return Pointer to the Body
 */
OpenSim::Body* PistonActuator::getBodyB() const
{
    return(_bodyB);
}

//-----------------------------------------------------------------------------
// OPTIMAL FORCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the optimal force of the actuator.
 *
 * @param aOptimalForce Optimal force.
 */
void PistonActuator::setOptimalForce(double aOptimalForce)
{
    set_optimal_force(aOptimalForce);
}
//_____________________________________________________________________________
/**
 * Get the optimal force of the actuator.
 *
 * @return Optimal force.
 */
double PistonActuator::getOptimalForce() const
{
    return(get_optimal_force());
}
//_____________________________________________________________________________
/**
 * Get the stress of the force. This would be the force or torque provided by 
 * this actuator divided by its optimal force.
 * @return Stress.
 */
double PistonActuator::getStress( const SimTK::State& s) const
{
    return fabs(getActuation(s) / get_optimal_force());
}


//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute all quantities necessary for applying the actuator force to the
 * model.
 *
 * @param s current SimTK::State 
 */

double PistonActuator::computeActuation( const SimTK::State& s ) const
{
    if(!_model) return 0;

    // FORCE
    return ( getControl(s) * getOptimalForce() );
}



//=============================================================================
// APPLICATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Apply the actuator force to BodyA and BodyB.
 *
 * @param s current SimTK::State
 */
void PistonActuator::computeForce(const SimTK::State& s, 
                                    SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
                                    SimTK::Vector& generalizedForces) const
{
    if(!_model) return;
    const SimbodyEngine& engine = getModel().getSimbodyEngine();
    
    if(_bodyA ==NULL || _bodyB ==NULL)
        return;
    
    /* store _pointA and _pointB positions in the global frame.  If not
    ** already in the body frame, transform _pointA and _pointB into their
    ** respective body frames. */

    SimTK::Vec3 pointA_inGround, pointB_inGround;

    SimTK::Vec3 _pointA = get_pointA();
    SimTK::Vec3 _pointB = get_pointB();
    if (get_points_are_global())
    {
        pointA_inGround = _pointA;
        pointB_inGround = _pointB;
        engine.transformPosition(s, getModel().getGround(), _pointA, *_bodyA, _pointA);
        engine.transformPosition(s, getModel().getGround(), _pointB, *_bodyB, _pointB);
    }
    else
    {
        engine.transformPosition(s, *_bodyA, _pointA, getModel().getGround(), pointA_inGround);
        engine.transformPosition(s, *_bodyB, _pointB, getModel().getGround(), pointB_inGround);
    }

    // find the direction along which the actuator applies its force
    SimTK::Vec3 r = pointA_inGround - pointB_inGround;

    SimTK::UnitVec3 direction(r);

    // find the force magnitude and set it. then form the force vector
    double forceMagnitude = computeActuation(s);
    setActuation(s,  forceMagnitude );
    SimTK::Vec3 force = forceMagnitude*direction;

    // apply equal and opposite forces to the bodies
    applyForceToPoint(s, *_bodyA, _pointA, force, bodyForces);
    applyForceToPoint(s, *_bodyB, _pointB, -force, bodyForces);
}
//_____________________________________________________________________________
/**
 * extendConnectToModel() sets the actual Body references _bodyA and _bodyB
 */
void PistonActuator::
extendConnectToModel(Model& aModel)
{
    Super::extendConnectToModel( aModel);

    if (_model) {
        _bodyA = &_model->updBodySet().get(upd_bodyA());
        _bodyB = &_model->updBodySet().get(upd_bodyB());
    }
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
void PistonActuator::
updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    Actuator::updateFromXMLNode(aNode, versionNumber);
    setBodyA(_bodyA);
    setBodyB(_bodyB);
    setOptimalForce(upd_optimal_force());
}   
