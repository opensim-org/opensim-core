// ContactForceSet.cpp
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
#include <OpenSim/Tools/PropertyObjArray.h>
#include "Actuator.h"
#include "ContactForceSet.h"


//=============================================================================
// STATICS
//=============================================================================


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________


using namespace OpenSim;
/**
 * Destructor.
 */
ContactForceSet::~ContactForceSet()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
ContactForceSet::ContactForceSet()
{
	// NULL
	setNull();
}
//_____________________________________________________________________________
/**
 * Construct an actuator from file.
 *
 * @param aFileName Name of the file.
 */
ContactForceSet::ContactForceSet(const char *aFileName) :
	ActuatorSet(aFileName)
{
	setNull();
	updateFromXMLNode();
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this actuator to their null values.
 */
void ContactForceSet::
setNull()
{
	// TYPE
	setType("ContactForceSet");

	// PROPERTIES
	setupSerializedMembers();
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ContactForceSet::
setupSerializedMembers()
{
}


//=============================================================================
// SET OPERATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Append a contact object on to the contact force set.  A copy is NOT made.
 *
 * @param aContact Contact force to be appended.
 * @return True if successful; false otherwise.
 */
bool ContactForceSet::
append(ContactForce *aContact)
{
	return( ActuatorSet::append(aContact) );
}
//_____________________________________________________________________________
/**
 * Set the contact force at an index.  A copy is not made.  The contact force
 * previously set a the index is removed and, if the set is the memory owner,
 * deleted.
 *
 * @param aIndex Array index where the contact object is to be stored.  aIndex
 * should be in the range 0 <= aIndex <= getSize();
 * @param aContact Contact force to be set.
 * @return True if successful; false otherwise.
 */
bool ContactForceSet::
set(int aIndex,ContactForce *aContact)
{
	return( ActuatorSet::set(aIndex,aContact) );
}


//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * For each contact, compute all necessary quantities.
 */
void ContactForceSet::
computeContact()
{
	int i;
	Actuator *act;
	for(i=0;i<getSize();i++) {
		act = get(i);
		if(act!=NULL) act->computeActuation();
	}
}
//_____________________________________________________________________________
/**
 * Update the pseudostates of all contact forces.  Pseudostates are quantities
 * that are not integrated but that depend on the time history of a
 * simulation (e.g., spring set points).
 */
void ContactForceSet::
updatePseudoStates()
{
	int i;
	int size = getSize();
	Actuator *contact;
	for(i=0;i<size;i++) {
		contact = get(i);
		if(contact!=NULL) contact->updatePseudoStates();
	}
}


//=============================================================================
// APPLICATION
//=============================================================================
//_____________________________________________________________________________
/**
 * For each contact, apply the force(s) (or torque(s)) to the model.
 */
void ContactForceSet::
apply()
{
	int i;
	Actuator *act;
	for(i=0;i<getSize();i++) {
		act = get(i);
		if(act!=NULL) act->apply();
	}
}


//=============================================================================
// CHECK
//=============================================================================
//_____________________________________________________________________________
/**
 * Check that all contacts are valid.
 */
bool ContactForceSet::
check() const
{
	bool status=true;

	// LOOP THROUGH CONTACTS
	int i;
	Actuator *act;
	for(i=0;i<getSize();i++) {
		act = get(i);
		if(act==NULL) continue;
		if(!act->check()) status = false;
	}

	return(status);
}
