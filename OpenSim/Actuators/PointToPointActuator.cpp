// PointToPointActuator.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2009, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* 
 * Author: Matt DeMers
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include "PointToPointActuator.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>

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
PointToPointActuator::~PointToPointActuator()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 * @param aBodyNameA name of the first body to which the force is applied
 * @param aBodyNameB name of the second body to which the force is applied
 *
 */
PointToPointActuator::PointToPointActuator( string aBodyNameA, string aBodyNameB) :
	Actuator(),
	_bodyNameA(_propBodyNameA.getValueStr()),
	_bodyNameB(_propBodyNameB.getValueStr()),
	_pointsAreGlobal(_propPointsAreGlobal.getValueBool()),
	_pointA(_propPointA.getValueDblVec()),
	_pointB(_propPointB.getValueDblVec()),
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
PointToPointActuator::PointToPointActuator(const PointToPointActuator &anActuator) :
	Actuator(anActuator),
	_bodyNameA(_propBodyNameA.getValueStr()),
	_bodyNameB(_propBodyNameB.getValueStr()),
	_pointsAreGlobal(_propPointsAreGlobal.getValueBool()),
	_pointA(_propPointA.getValueDblVec()),
	_pointB(_propPointB.getValueDblVec()),
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
Object* PointToPointActuator::
copy() const
{
	PointToPointActuator *force = new PointToPointActuator(*this);
	return force;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this actuator to their null values.
 */
void PointToPointActuator::
setNull()
{
	setType("PointToPointActuator");
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void PointToPointActuator::
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
 * @param aPointToPointActuator PointToPointActuator providing the data to be copied
 */
void PointToPointActuator::
copyData(const PointToPointActuator &aPointToPointActuator)
{
	// MEMBER VARIABLES
	_bodyNameA = aPointToPointActuator._bodyNameA;
	_bodyNameB = aPointToPointActuator._bodyNameB;
	_pointsAreGlobal = aPointToPointActuator._pointsAreGlobal;
	_pointA = aPointToPointActuator._pointA;
	_pointB = aPointToPointActuator._pointB;

	setOptimalForce(aPointToPointActuator.getOptimalForce());
	setBodyA(aPointToPointActuator.getBodyA());
	setBodyB(aPointToPointActuator.getBodyB());
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
PointToPointActuator& PointToPointActuator::
operator=(const PointToPointActuator &aPointToPointActuator)
{
	// BASE CLASS
	Actuator::operator =(aPointToPointActuator);

	copyData(aPointToPointActuator);

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
void PointToPointActuator::setBodyA(Body* aBody)
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
void PointToPointActuator::setBodyB(Body* aBody)
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
Body* PointToPointActuator::getBodyA() const
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
Body* PointToPointActuator::getBodyB() const
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
void PointToPointActuator::setOptimalForce(double aOptimalForce)
{
	_optimalForce = aOptimalForce;
}
//_____________________________________________________________________________
/**
 * Get the optimal force of the actuator.
 *
 * @return Optimal force.
 */
double PointToPointActuator::getOptimalForce() const
{
	return(_optimalForce);
}
//_____________________________________________________________________________
/**
 * Get the stress of the force.
 *
 * @return Stress.
 */
double PointToPointActuator::getStress( const SimTK::State& s) const
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

double PointToPointActuator::computeActuation( const SimTK::State& s ) const
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
void PointToPointActuator::computeForce(const SimTK::State& s, 
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

	SimTK::Vec3 pointA_inGround, pointB_inGround, pointA_inBodyA, pointB_inBodyB;

	if (_pointsAreGlobal)
	{
		pointA_inGround = _pointA;
		pointB_inGround = _pointB;
		engine.transformPosition(s, engine.getGroundBody(), pointA_inGround, *_bodyA, pointA_inBodyA);
		engine.transformPosition(s, engine.getGroundBody(), pointB_inGround, *_bodyB, pointB_inBodyB);
	}
	else
	{
		pointA_inBodyA = _pointA;
		pointB_inBodyB = _pointB;
		engine.transformPosition(s, *_bodyA, pointA_inBodyA, engine.getGroundBody(), pointA_inGround);
		engine.transformPosition(s, *_bodyB, pointB_inBodyB, engine.getGroundBody(), pointB_inGround);
	}

	// find the dirrection along which the actuator applies its force
	SimTK::Vec3 r = pointA_inGround - pointB_inGround;

	SimTK::UnitVec3 direction(r);

	// find the force magnitude and set it. then form the force vector
	double forceMagnitude;

    if( isForceOverriden(s) ) {
       forceMagnitude = computeOverrideForce(s);
    } else {
       forceMagnitude = computeActuation(s);
    }
    setForce(s,  forceMagnitude );

	SimTK::Vec3 force = forceMagnitude*direction;

	// appy equal and opposite forces to the bodies
	applyForceToPoint(s, *_bodyA, pointA_inBodyA, force, bodyForces);
	applyForceToPoint(s, *_bodyB, pointB_inBodyB, -force, bodyForces);

	// get the velocity of the actuator in ground
	SimTK::Vec3 velA_G(0), velB_G(0), velAB_G(0);
	engine.getVelocity(s, *_bodyA, pointA_inBodyA, velA_G);
	engine.getVelocity(s, *_bodyB, pointB_inBodyB, velB_G);
	velAB_G = velA_G-velB_G;
	// speed used to comput power is the speed along the line connecting the two bodies
	setSpeed(s, ~velAB_G*direction);
}
//_____________________________________________________________________________
/**
 * setup sets the actual Body references _bodyA and _bodyB
 */
void PointToPointActuator::setup(Model& aModel)
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
bool PointToPointActuator::check() const
{
	// BodyID
	if( _bodyA != NULL) {
		printf("PointToPointActuator.check: ERROR- %s actuates ",
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
void PointToPointActuator::
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
OpenSim::Array<std::string> PointToPointActuator::getRecordLabels() const {
	OpenSim::Array<std::string> labels("");
	labels.append(getName());
	return labels;
}
/**
 * Given SimTK::State object extract all the values necessary to report forces, application location
 * frame, etc. used in conjunction with getRecordLabels and should return same size Array
 */
OpenSim::Array<double> PointToPointActuator::getRecordValues(const SimTK::State& state) const {
	OpenSim::Array<double> values(1);
	values.append(getForce(state));
	return values;
};

