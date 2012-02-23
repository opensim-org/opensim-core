// ForceSet.cpp
// Authors: Ajay Seth, Jack Middleton
/*
 * Copyright (c)  2010, Stanford University. All rights reserved. 
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


//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <algorithm>
#include "ForceSet.h"
#include "Model.h"
#include "Muscle.h"
#include "SimTKsimbody.h"

using namespace std;
using namespace OpenSim;

#ifndef SWIG
template class OSIMSIMULATION_API ModelComponentSet<Force>;
#endif


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
ForceSet::~ForceSet()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
ForceSet::ForceSet()
{
	setNull();
}

ForceSet::ForceSet(Model& model) : 
ModelComponentSet<Force>(model)
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Construct an actuator set from file.
 *
 * @param aFileName Name of the file.
 */
ForceSet::ForceSet(Model& model, const std::string &aFileName, bool aUpdateFromXMLNode) :
	ModelComponentSet<Force>(model, aFileName, false)
{
	setNull();

	if(aUpdateFromXMLNode)
		updateFromXMLDocument();
}


//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aForceSet ForceSet to be copied.
 */
ForceSet::ForceSet(const ForceSet &aForceSet) :
	ModelComponentSet<Force>(aForceSet)
{
	setNull();

	// Class Members
	copyData(aForceSet);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this ForceSet to their null values.
 */
void ForceSet::setNull()
{
	// TYPE
	setType("ForceSet");
	// NAME
	//setName("ForceSet");

	// PROPERTIES
	setupSerializedMembers();

	_actuators.setMemoryOwner(false);

	_muscles.setMemoryOwner(false);
}

//_____________________________________________________________________________
/**
 * Copy this ForceSet and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this ForceSet.
 */
Object* ForceSet::copy() const
{
	ForceSet *actSet = new ForceSet(*this);
	return(actSet);
}

//_____________________________________________________________________________
/**
 * Copy the member variables of the ForceSet.
 *
 * @param aAbsForceSet actuator set to be copied
 */
void ForceSet::copyData(const ForceSet &aAbsForceSet)
{
    // ACTUATORS
    _actuators = aAbsForceSet._actuators;
	_actuators.setMemoryOwner(false);
	_muscles = aAbsForceSet._muscles;
	_muscles.setMemoryOwner(false);
}

//_____________________________________________________________________________
/**
 * Set up the serialized member variables.
 */
void ForceSet::setupSerializedMembers()
{
}

void ForceSet::setup(Model& aModel)
{
	// BASE CLASS
	ModelComponentSet<Force>::setup(aModel);

	updateActuators();
	updateMuscles();
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
ForceSet& ForceSet::operator=(const ForceSet &aAbsForceSet)
{
	// BASE CLASS
	Set<Force>::operator=(aAbsForceSet);

	// Class Members
	copyData(aAbsForceSet);

	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================

//-----------------------------------------------------------------------------
// ACTUATOR
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Remove an actuator from the actuator set.
 *
 * @param aIndex Index of the actuator to be removed.
 * @return True if the remove was successful; false otherwise.
 */
bool ForceSet::remove(int aIndex)
{
	bool success = Set<Force>::remove(aIndex);

	updateActuators();
	updateMuscles();

	return(success);
}

//_____________________________________________________________________________
/**
 * Append an actuator on to the set.  A copy of the specified actuator
 * is not made.
 *
 * This method overrides the method in Set<Force> so that several
 * internal variables of the actuator set can be updated.
 *
 * @param aActuator Pointer to the actuator to be appended.
 * @return True if successful; false otherwise.
 */
bool ForceSet::
append(Force *aForce)
{
	bool success = ModelComponentSet<Force>::append(aForce);

	if((success)&&(_model!=NULL)) {
		updateActuators();
		updateMuscles();
	}

	return(success);
}
//_____________________________________________________________________________
/**
 * Append an actuator on to the set.  A copy of the specified actuator
 * is made.
 *
 * This method overrides the method in Set<Force> so that several
 * internal variables of the actuator set can be updated.
 *
 * @param aActuator reference to the actuator to be appended.
 * @return True if successful; false otherwise.
 */
bool ForceSet::
append(Force &aForce)
{
	bool success = ModelComponentSet<Force>::append(aForce);


	if((success)&&(_model!=NULL)) {
		updateActuators();
	    updateMuscles();
	}

	return(success);
}
//_____________________________________________________________________________
/**
 * Append actuators from an actuator set to this set.  Copies of the actuators are not made.
 *
 * This method overrides the method in Set<Force> so that several
 * internal variables of the actuator set can be updated.
 *
 * @param aForceSet The set of actuators to be appended.
 * @param aAllowDuplicateNames If true, all actuators will be appended; If false, don't append actuators whose
 * name already exists in this model's actuator set.
 * @return True if successful; false otherwise.
 */
bool ForceSet::append(ForceSet &aForceSet, bool aAllowDuplicateNames)
{
	bool success = true;
	for(int i=0;i<aForceSet.getSize() && success;i++) {
		bool nameExists = false;
		if(!aAllowDuplicateNames) {
			std::string name = aForceSet.get(i).getName();
			for(int j=0;j<getSize();j++) {
				if(get(j).getName() == name) {
					nameExists = true;
					break;
				}
			}
		}
		if(!nameExists) {
			if(!ModelComponentSet<Force>::append(&aForceSet.get(i))) 
				success = false;
		}
	}

	if(success) {
		updateActuators();
		updateMuscles();
	}

	return(success);
}
//_____________________________________________________________________________
/**
 * Set the actuator at an index.  A copy of the specified actuator is NOT made.
 * The actuator previously set a the index is removed (and deleted).
 *
 * This method overrides the method in Set<Force> so that several
 * internal variables of the actuator set can be updated.
 *
 * @param aIndex Array index where the actuator is to be stored.  aIndex
 * should be in the range 0 <= aIndex <= getSize();
 * @param aActuator Pointer to the actuator to be set.
 * @return True if successful; false otherwise.
 */
bool ForceSet::set(int aIndex,Force *aActuator)
{
	bool success = ModelComponentSet<Force>::set(aIndex,aActuator);

	if(success) {
		updateActuators();
		updateMuscles();
	}

	return(success);
}

bool ForceSet::insert(int aIndex, Force *aForce)
{
	bool success = ModelComponentSet<Force>::insert(aIndex, aForce);

	if(success) {
		updateActuators();
		updateMuscles();
	}

	return(success);
}

//_____________________________________________________________________________
/**
 * Get the list of Actuators.
 */
const Set<Actuator>& ForceSet::getActuators() const
{
	return _actuators;
}

Set<Actuator>& ForceSet::updActuators() 
{
	updateActuators();
    return _actuators;
}
//_____________________________________________________________________________
/**
 * Rebuild the list of Actuators.
 */
void ForceSet::updateActuators()
{
    _actuators.setSize(0);
    for (int i = 0; i < getSize(); ++i)
    {
        Actuator* act = dynamic_cast<Actuator*>(&get(i));
        if (act != NULL)  _actuators.append(act);
    }
}

//=============================================================================
//_____________________________________________________________________________
/**
 * Get the list of Muscles.
 */
const Set<Muscle>& ForceSet::getMuscles() const
{
    return _muscles;
}
Set<Muscle>& ForceSet::updMuscles() 
{
	updateMuscles();
    return _muscles;
}
//_____________________________________________________________________________
/**
 * Rebuild the list of Muscles.
 */
void ForceSet::updateMuscles()
{
    _muscles.setSize(0);
    for (int i = 0; i < getSize(); ++i)
    {
        Muscle* m = dynamic_cast<Muscle*>(&get(i));
        if (m != NULL)  _muscles.append(m);
    }
}

//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________


//_____________________________________________________________________________
/**
 * Get the names of the states of the actuators.
 *
 * @param rNames Array of names.
 */
void ForceSet::
getStateVariableNames(OpenSim::Array<std::string> &rNames) const
{
	for(int i=0;i<getSize();i++) {
		Actuator *act = dynamic_cast<Actuator*>(&get(i)); 
       
		if(act) {
            rNames.append(act->getStateVariableNames());
        }
	}
}


//=============================================================================
// CHECK
//=============================================================================
//_____________________________________________________________________________
/**
 * Check that all actuators are valid.
 */
bool ForceSet::
check() const
{
	bool status=true;

	// LOOP THROUGH ACTUATORS
	Actuator *act;
	int size = getSize();
	for(int i=0;i<size;i++) {
		act = dynamic_cast<Actuator *>(&get(i));
		if(!act) continue;
	}

	return(status);
}
