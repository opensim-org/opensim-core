// GeneralizedForce.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include "GeneralizedForce.h"
#include "Model.h"
#include "AbstractDynamicsEngine.h"
#include "CoordinateSet.h"
#include "SpeedSet.h"



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
GeneralizedForce::~GeneralizedForce()
{
	if(_utmp!=NULL) { delete[] _utmp;  _utmp=NULL; }
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
GeneralizedForce::GeneralizedForce(string aQName) :
	AbstractActuator(),
	_qName(_propQName.getValueStr()),
	_optimalForce(_propOptimalForce.getValueDbl()),
	_q(NULL),
	_u(NULL)
{
	// NULL
	setNull();

	// MEMBER VARIABLES
	_qName = aQName;

	if (_model) {
		_q = _model->getDynamicsEngine().getCoordinateSet()->get(_qName);
		_u = _model->getDynamicsEngine().getSpeedSet()->get(AbstractSpeed::getSpeedName(_qName));
	}
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aForce Force to be copied.
 */
GeneralizedForce::GeneralizedForce(const GeneralizedForce &aGenForce) :
	AbstractActuator(aGenForce),
	_qName(_propQName.getValueStr()),
	_optimalForce(_propOptimalForce.getValueDbl()),
	_q(NULL),
	_u(NULL)
{
	setNull();
	copyData(aGenForce);
}
//_____________________________________________________________________________
/**
 * Copy this actuator and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this actuator.
 */
Object* GeneralizedForce::
copy() const
{
	GeneralizedForce *force = new GeneralizedForce(*this);
	return force;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this actuator to their null values.
 */
void GeneralizedForce::
setNull()
{
	setType("GeneralizedForce");
	setupProperties();

	// APPLIES FORCE
	_appliesForce = true;

	setNumControls(1); setNumStates(0); setNumPseudoStates(0);
	bindControl(0, _excitation, "excitation");

	// MEMBER VARIABLES
	_utmp = NULL;
	_excitation = 0.0;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void GeneralizedForce::
setupProperties()
{
	_propQName.setName("coordinate"); // TODOAUG: was "qid"
	_propertySet.append( &_propQName );

	_propOptimalForce.setName("optimal_force");
	_propOptimalForce.setValue(1.0);
	_propertySet.append( &_propOptimalForce );
}

//_____________________________________________________________________________
/**
 * Copy the member data of the specified actuator.
 */
void GeneralizedForce::
copyData(const GeneralizedForce &aGenForce)
{
	// MEMBER VARIABLES
	setQ(aGenForce.getQ());
	setOptimalForce(aGenForce.getOptimalForce());
	_excitation = aGenForce._excitation;
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
 * @return  aQID ID (or number, or index) of the generalized coordinate.
 */
GeneralizedForce& GeneralizedForce::
operator=(const GeneralizedForce &aGenForce)
{
	// BASE CLASS
	AbstractActuator::operator =(aGenForce);

	copyData(aGenForce);

	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// QID
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the generalized coordinate to which the generalized force is applied.
 *
 * @param aQ ID Pointer to the generalized coordinate.
 */
void GeneralizedForce::
setQ(AbstractCoordinate* aQ)
{
	_q = aQ;
	if (aQ)
		_qName = aQ->getName();
}
//_____________________________________________________________________________
/**
 * Get the generalized coordinate to which the generalized force
 * is applied.
 *
 * @return Pointer to the coordinate
 */
AbstractCoordinate* GeneralizedForce::
getQ() const
{
	return(_q);
}

//-----------------------------------------------------------------------------
// OPTIMAL FORCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the optimal force of the force.
 *
 * @param aOptimalForce Optimal force.
 */
void GeneralizedForce::
setOptimalForce(double aOptimalForce)
{
	_optimalForce = aOptimalForce;
}
//_____________________________________________________________________________
/**
 * Get the optimal force of the force.
 *
 * @return Optimal force.
 */
double GeneralizedForce::
getOptimalForce() const
{
	return(_optimalForce);
}
//_____________________________________________________________________________
/**
 * Get the stress of the force.
 *
 * @return Stress.
 */
double GeneralizedForce::
getStress() const
{
	return fabs(_force/_optimalForce); 
}


//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute all quantities necessary for applying the actuator force to the
 * model.
 */
void GeneralizedForce::
computeActuation()
{
	if(_model==NULL) return;

	// SPEED
	if (_u) _speed = _u->getValue();

	// FORCE
	double force = _excitation * _optimalForce;
	setForce(force);
}


//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 */
ActuatorSet *GeneralizedForce::
CreateActuatorSetOfGeneralizedForcesForModel(Model *aModel,double aOptimalForce,bool aIncludeLockedAndConstrainedCoordinates)
{
	ActuatorSet *as = new ActuatorSet();
	CoordinateSet *cs = aModel->getDynamicsEngine().getCoordinateSet();
	for(int i=0; i<cs->getSize(); i++) {
		if(!aIncludeLockedAndConstrainedCoordinates && (cs->get(i)->getLocked() || cs->get(i)->getConstrained())) continue;
		GeneralizedForce *actuator = new GeneralizedForce();
		actuator->setQ(cs->get(i));
		actuator->setName(cs->get(i)->getName());
		actuator->setOptimalForce(aOptimalForce);
		as->append(actuator);
	}
	as->setup(aModel);
	return as;
}

//=============================================================================
// APPLICATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Apply the actuator force to BodyA and BodyB.
 */
void GeneralizedForce::
apply()
{
	if(_model==NULL) return;
	if(isQValid()) _model->getDynamicsEngine().applyGeneralizedForce(*_q,_force);
}
//_____________________________________________________________________________
/**
 * setup sets the actual Coordinate reference _q
 */
void GeneralizedForce::
setup(Model* aModel)
{
	AbstractActuator::setup(aModel);
	if (_model) {
		_q = _model->getDynamicsEngine().getCoordinateSet()->get(_qName);
		_u = _model->getDynamicsEngine().getSpeedSet()->get(AbstractSpeed::getSpeedName(_qName));
	}
}


//=============================================================================
// CHECK
//=============================================================================
//_____________________________________________________________________________
/**
 * Check that this generalized force actuator is valid.
 *
 * @return True if valid, false if invalid.
 */
bool GeneralizedForce::
check() const
{
	if(!AbstractActuator::check()) return(false);

	// QID
	if(!isQValid()) {
		printf("GeneralizedForce.check: ERROR- %s actuates ",
			getName().c_str());
		printf("an invalid generalized coordinate (%s).\n", _qName.c_str());
		return(false);
	}

	return(true);
}
//_____________________________________________________________________________
/**
 * Is the.
 */
bool GeneralizedForce::
isQValid() const
{
	if (_model == NULL || _q == NULL || _u == NULL)
		return false;

	return true;
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
void GeneralizedForce::
updateFromXMLNode()
{
	AbstractActuator::updateFromXMLNode();
	setQ(_q);
	setOptimalForce(_optimalForce);
}	

