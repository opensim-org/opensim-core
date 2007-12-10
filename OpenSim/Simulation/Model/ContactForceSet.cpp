// ContactForceSet.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include "AbstractActuator.h"
#include "ContactForce.h"
#include "ContactForceSet.h"


//=============================================================================
// STATICS
//=============================================================================


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________


using namespace OpenSim;
using namespace std;

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
ContactForceSet::ContactForceSet():
ActuatorSet()
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
ContactForceSet::ContactForceSet(const string &aFileName) :
	ActuatorSet(aFileName, false)
{
	setNull();
	updateFromXMLNode();
	// TODO: Do we need to call constructMaps again here, since ActuatorSet did not call updateFromXMLNode
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aContactForceSet ContactForceSet to be copied.
 */
ContactForceSet::ContactForceSet(const ContactForceSet &aContactForceSet) :
	ActuatorSet(aContactForceSet)
{
	setNull();

	// Class Members
	copyData(aContactForceSet);
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

	// MODEL
	_model = NULL;
}
//_____________________________________________________________________________
/**
 * Copy this ContactForceSet and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this ContactForceSet.
 */
Object* ContactForceSet::copy() const
{
	ContactForceSet *contactSet = new ContactForceSet(*this);
	return(contactSet);
}
//_____________________________________________________________________________
/**
 * Copy the member variables of the ContactForceSet.
 *
 * @param aContactForceSet contact set to be copied
 */
void ContactForceSet::copyData(const ContactForceSet &aContactForceSet)
{
	ActuatorSet::copyData(aContactForceSet);
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
 */
bool ContactForceSet::
append(ContactForceSet &aContactForceSet, bool aAllowDuplicateNames)
{
	return ActuatorSet::append(aContactForceSet, aAllowDuplicateNames);
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
	AbstractActuator *act;
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
	AbstractActuator *contact;
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
	AbstractActuator *act;
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
	AbstractActuator *act;
	for(i=0;i<getSize();i++) {
		act = get(i);
		if(act==NULL) continue;
		if(!act->check()) status = false;
	}

	return(status);
}
