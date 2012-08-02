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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied    *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/* 
 * Author: Matt DeMers
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include "PistonActuator.h"
#include <OpenSim/OpenSim.h>
/*
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
*/

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
	CustomActuator(),
	_bodyNameA(_propBodyNameA.getValueStr()),
	_bodyNameB(_propBodyNameB.getValueStr()),
	_pointsAreGlobal(_propPointsAreGlobal.getValueBool()),
	_pointA(_propPointA.getValueDblVec3()),
	_pointB(_propPointB.getValueDblVec3()),
	_optimalForce(_propOptimalForce.getValueDbl()),
	_bodyA(NULL),
	_bodyB(NULL)
{
	// NULL
	setNull();

	// MEMBER VARIABLES
	_bodyNameA = aBodyNameA;
	_bodyNameB = aBodyNameB;

	if (_model) {
		_bodyA = &_model->updBodySet().get(_bodyNameA);
		_bodyB = &_model->updBodySet().get(_bodyNameB);
	} 
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param anActuator actuator to be copied.
 */
PistonActuator::PistonActuator(const PistonActuator &anActuator) :
	CustomActuator(anActuator),
	_bodyNameA(_propBodyNameA.getValueStr()),
	_bodyNameB(_propBodyNameB.getValueStr()),
	_pointsAreGlobal(_propPointsAreGlobal.getValueBool()),
	_pointA(_propPointA.getValueDblVec3()),
	_pointB(_propPointB.getValueDblVec3()),
	_optimalForce(_propOptimalForce.getValueDbl()),
	_bodyA(NULL),
	_bodyB(NULL)
{
	setNull();
	copyData(anActuator);
}
//_____________________________________________________________________________
/**
 * Copy this actuator and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this actuator.
 */
Object* PistonActuator::
copy() const
{
	PistonActuator *force = new PistonActuator(*this);
	return force;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this actuator to their null values.
 */
void PistonActuator::
setNull()
{
	setType("PistonActuator");
	setupProperties();

	setNumStateVariables( 0);
}

void PistonActuator::initStateCache(SimTK::State& s, SimTK::SubsystemIndex subsystemIndex, Model& model )
{
    Actuator::initStateCache(s, subsystemIndex, model);

}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void PistonActuator::
setupProperties()
{
	SimTK::Vec3 x(0.0, 0.0, 0.0);

	_propBodyNameA.setName("bodyA");
	_propertySet.append( &_propBodyNameA );

	_propBodyNameB.setName("bodyB");
	_propertySet.append( &_propBodyNameB );

	_propPointsAreGlobal.setName("points_are_global");
	_propPointsAreGlobal.setValue(false);
	_propertySet.append( &_propPointsAreGlobal );

	_propPointA.setName("pointA");
	_propPointA.setValue(x);
	_propertySet.append( &_propPointA );

	_propPointB.setName("pointB");
	_propPointB.setValue(x);
	_propertySet.append( &_propPointB );

	_propOptimalForce.setName("optimal_force");
	_propOptimalForce.setValue(1.0);
	_propertySet.append( &_propOptimalForce );
}

//_____________________________________________________________________________
/**
 * Copy the member data of the specified actuator.
 * @param aPistonActuator PistonActuator providing the data to be copied
 */
void PistonActuator::
copyData(const PistonActuator &aPistonActuator)
{
	// MEMBER VARIABLES
	_bodyNameA = aPistonActuator._bodyNameA;
	_bodyNameB = aPistonActuator._bodyNameB;
	_pointsAreGlobal = aPistonActuator._pointsAreGlobal;
	_pointA = aPistonActuator._pointA;
	_pointB = aPistonActuator._pointB;

	setOptimalForce(aPistonActuator.getOptimalForce());
	setBodyA(aPistonActuator.getBodyA());
	setBodyB(aPistonActuator.getBodyB());
}


//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return  aBodyID ID (or number, or index) of the generalized Body.
 */
PistonActuator& PistonActuator::
operator=(const PistonActuator &aPistonActuator)
{
	// BASE CLASS
	Actuator::operator =(aPistonActuator);

	copyData(aPistonActuator);

	return(*this);
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
		_bodyNameA = aBody->getName();
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
		_bodyNameB = aBody->getName();
}
//_____________________________________________________________________________
/**
 * Get the generalized Body to which the Body actuator
 * is applied.
 *
 * @return Pointer to the Body
 */
Body* PistonActuator::getBodyA() const
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
Body* PistonActuator::getBodyB() const
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
	_optimalForce = aOptimalForce;
}
//_____________________________________________________________________________
/**
 * Get the optimal force of the actuator.
 *
 * @return Optimal force.
 */
double PistonActuator::getOptimalForce() const
{
	return(_optimalForce);
}
//_____________________________________________________________________________
/**
 * Get the stress of the force.
 *
 * @return Stress.
 */
double PistonActuator::getStress( const SimTK::State& s) const
{
	return fabs(getForce(s)/_optimalForce); 
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
	if(_model==NULL) return 0;

	// FORCE
	return ( getControl(s) * _optimalForce );
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
	if(_model==NULL) return;
	const SimbodyEngine& engine = getModel().getSimbodyEngine();
	
	if(_bodyA ==NULL || _bodyB ==NULL)
		return;
	
	/* store _pointA and _pointB positions in the global frame.  If not
	** alread in the body frame, transform _pointA and _pointB into their
	** respective body frames. */

	SimTK::Vec3 pointA_inGround, pointB_inGround;

	if (_pointsAreGlobal)
	{
		pointA_inGround = _pointA;
		pointB_inGround = _pointB;
		engine.transformPosition(s, engine.getGroundBody(), _pointA, *_bodyA, _pointA);
		engine.transformPosition(s, engine.getGroundBody(), _pointB, *_bodyB, _pointB);
	}
	else
	{
		engine.transformPosition(s, *_bodyA, _pointA, engine.getGroundBody(), pointA_inGround);
		engine.transformPosition(s, *_bodyB, _pointB, engine.getGroundBody(), pointB_inGround);
	}

	// find the dirrection along which the actuator applies its force
	SimTK::Vec3 r = pointA_inGround - pointB_inGround;

	SimTK::UnitVec3 direction(r);

	// find the force magnitude and set it. then form the force vector
	double forceMagnitude = computeActuation(s);
	setForce(s,  forceMagnitude );
	SimTK::Vec3 force = forceMagnitude*direction;

	// appy equal and opposite forces to the bodies
	applyForceToPoint(s, *_bodyA, _pointA, force, bodyForces);
	applyForceToPoint(s, *_bodyB, _pointB, -force, bodyForces);
}
//_____________________________________________________________________________
/**
 * setup sets the actual Body references _bodyA and _bodyB
 */
void PistonActuator::
setup(Model& aModel)
{
	Actuator::setup( aModel);

	if (_model) {
		_bodyA = &_model->updBodySet().get(_bodyNameA);
		_bodyB = &_model->updBodySet().get(_bodyNameB);
	}
}


//=============================================================================
// CHECK
//=============================================================================
//_____________________________________________________________________________
/**
 * Check that this point actuator actuator is valid.
 *
 * @return True if valid, false if invalid.
 */
bool PistonActuator::check() const
{
	if(!Actuator::check()) return(false);

	// BodyID
	if( _bodyA != NULL) {
		printf("PistonActuator.check: ERROR- %s actuates ",
			getName().c_str());
		printf("an invalid Body (%s).\n", _bodyNameA.c_str());
		return(false);
	}
	return(true);
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
 * This method simply calls Object::updateFromXMLNode() and then calls
 * a few methods in this class to ensure that variable members have been
 * set in a consistent manner.
 */
void PistonActuator::
updateFromXMLNode()
{
	Actuator::updateFromXMLNode();
	setBodyA(_bodyA);
	setBodyB(_bodyB);
	setOptimalForce(_optimalForce);
}	

/** 
 * Methods to query a Force for the value actually applied during simulation
 * The names of the quantities (column labels) is returned by this first function
 * getRecordLabels()
 */
OpenSim::Array<std::string> PistonActuator::getRecordLabels() const {
	OpenSim::Array<std::string> labels("");
	labels.append(getName());
	return labels;
}
/**
 * Given SimTK::State object extract all the values necessary to report forces, application location
 * frame, etc. used in conjunction with getRecordLabels and should return same size Array
 */
OpenSim::Array<double> PistonActuator::getRecordValues(const SimTK::State& state) const {
	OpenSim::Array<double> values(1);
	values.append(getForce(state));
	return values;
};

