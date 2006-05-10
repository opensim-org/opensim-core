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
#include <OpenSim/Tools/IO.h>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/PropertyInt.h>
#include "GeneralizedForce.h"




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
GeneralizedForce::GeneralizedForce(int aQID,int aNX,int aNY,int aNYP) :
	Actuator(aNX,aNY,aNYP),
	_qID(_propQID.getValueInt())
{
	// NULL
	setNull();

	// MEMBER VARIABLES
	_qID = aQID;
}
//_____________________________________________________________________________
/**
 * Construct the actuator from an XML element.
 *
 * @param aElement XML element.
 * @param aNX Number of controls.
 * @param aNY Number of states.
 * @param aNYP Number of pseudo-states.
 */
GeneralizedForce::
GeneralizedForce(DOMElement *aElement,int aNX,int aNY,int aNYP):
	Actuator(aNX,aNY,aNYP,aElement),
	_qID(_propQID.getValueInt())
{
	setNull();
	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aForce Force to be copied.
 */
GeneralizedForce::GeneralizedForce(const GeneralizedForce &aGenForce) :
	Actuator(aGenForce),
	_qID(_propQID.getValueInt())
{
	setNull();
	setQID(aGenForce.getQID());
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
	Actuator *act = new GeneralizedForce(*this);
	return(act);
}
//_____________________________________________________________________________
/**
 * Copy this actuator and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * GeneralizedForce::GeneralizedForce(DOMElement*,int,int) in order to establish the
 * relationship of the GeneralizedForce object with the XML node.  Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this GeneralizedForce object.  Finally, the data members of the copy are
 * updated using GeneralizedForce::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this actuator.
 */
Object* GeneralizedForce::
copy(DOMElement *aElement) const
{
	GeneralizedForce *act = new
		GeneralizedForce(aElement,getNX(),getNY(),getNYP());
	*act = *this;
	act->updateFromXMLNode();
	return(act);
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

	// MEMBER VARIABLES
	_utmp = NULL;
	_qID = -1;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void GeneralizedForce::
setupProperties()
{
	_propQID.setName("qid");
	_propQID.setValue(-1);
	_propertySet.append( &_propQID );
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
	Actuator::operator =(aGenForce);

	// MEMBER VARIABLES
	setQID(aGenForce.getQID());

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
 * @param aQID ID (or number, or index) of the generalized coordinate.
 */
void GeneralizedForce::
setQID(int aQID)
{
	_qID = aQID;
}
//_____________________________________________________________________________
/**
 * Get the ID of the generalized coordinate to which the generalized force
 * is applied.
 *
 * @param aID Body ID.
 */
int GeneralizedForce::
getQID() const
{
	return(_qID);
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
	_speed = _model->getSpeed(_qID);

	// FORCE
	double force = getControl(0) * _optimalForce;
	setForce(force);
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
	if(isQIDValid()) _model->applyGeneralizedForce(_qID,_force);
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
	if(!Actuator::check()) return(false);

	// QID
	if(!isQIDValid()) {
		printf("GeneralizedForce.check: ERROR- %s actuates ",
			getName().c_str());
		printf("an invalid generalized coordinate (%d).\n",getQID());
		return(false);
	}

	return(true);
}
//_____________________________________________________________________________
/**
 * Is the.
 */
bool GeneralizedForce::
isQIDValid() const
{
	if(_model==NULL) return(false);
	if((_qID>=0)&&(_qID<_model->getNU())) return(true);
	return(false);
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
	Actuator::updateFromXMLNode();
	setQID(_qID);
	setOptimalForce(_optimalForce);
}	

