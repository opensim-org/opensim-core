// ObjectGroup.cpp
// Author: Peter Loan
/*
 * Copyright (c)  2007, Stanford University. All rights reserved. 
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
#include "ObjectGroup.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
ObjectGroup::ObjectGroup() :
   Object(),
	_memberNames(_memberNamesProp.getValueStrArray())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Constructor taking the group name but no member names.
 */
ObjectGroup::ObjectGroup(const string& aName) :
   Object(),
	_memberNames(_memberNamesProp.getValueStrArray())
{
	setName(aName);
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
ObjectGroup::~ObjectGroup()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aGroup Group to be copied.
 */
ObjectGroup::ObjectGroup(const ObjectGroup &aGroup) :
   Object(aGroup),
	_memberNames(_memberNamesProp.getValueStrArray())
{
	setupProperties();
	copyData(aGroup);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one ObjectGroup to another.
 *
 * @param aGroup ObjectGroup to be copied.
 */
void ObjectGroup::copyData(const ObjectGroup &aGroup)
{
   _memberNames = aGroup._memberNames;
	_memberObjects = aGroup._memberObjects; // TODO: this copies pointers... but as long as call setup afterwards it should be okay
}

//_____________________________________________________________________________
/**
 * Set the data members of this ObjectGroup to their null values.
 */
void ObjectGroup::setNull()
{
	_memberObjects.setSize(0);
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ObjectGroup::setupProperties()
{
	_memberNamesProp.setName("members");
	_propertySet.append(&_memberNamesProp);
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
ObjectGroup& ObjectGroup::operator=(const ObjectGroup &aGroup)
{
	// BASE CLASS
	Object::operator=(aGroup);

	copyData(aGroup);

	return(*this);
}

//_____________________________________________________________________________
/**
 * Check if the group contains an object with a certain name.
 *
 * @param aName the name of the object.
 * @return Boolean indicating whether or not the group contains the object.
 */
bool ObjectGroup::contains(const string& aName) const
{
	for(int i=0; i<_memberObjects.getSize(); i++)
		if(_memberObjects[i] && _memberObjects[i]->getName()==aName)
			return true;
	return false;
}

//_____________________________________________________________________________
/**
 * Add an object to the group.
 *
 * @param aObject pointer to the object.
 */
void ObjectGroup::add(Object* aObject)
{
	if (aObject != NULL) {
		// check if object is already a member of this group
		if (_memberObjects.findIndex(aObject) != -1) return;

		_memberObjects.append(aObject);
		_memberNames.append(aObject->getName());
	}
}

//_____________________________________________________________________________
/**
 * Remove an object from the group.
 *
 * @param aObject pointer to the object.
 */
void ObjectGroup::remove(const Object* aObject)
{
	if (aObject != NULL)
	{
		int index = _memberObjects.findIndex(const_cast<Object*>(aObject));
		if(index >= 0) {
			_memberObjects.remove(index);
			_memberNames.remove(index);
		}
	}
}

//_____________________________________________________________________________
/**
 * Replace an object in the group with another object.
 *
 * @param aOldObject pointer to the old object.
 * @param aNewObject pointer to the new object.
 */
void ObjectGroup::replace(const Object* aOldObject, Object* aNewObject)
{
	if (aOldObject != NULL && aNewObject != NULL)
	{
		int index = _memberObjects.findIndex(const_cast<Object*>(aOldObject));
		if(index >= 0) {
			_memberObjects.get(index) = aNewObject;
			_memberNames.get(index) = aNewObject->getName();
		}
	}
}

//_____________________________________________________________________________
/**
 * Set up the group, after the member names have been deserialized.
 * For each member name, if the name is the name of an object in
 * aObject (the objects in the set that this group belongs to), then
 * store a pointer to the object in the same index in _memberObjects
 * as the name is in _memberNames. If the member name does not correspond
 * to an object in aObjects, remove the name from _memberNames.
 *
 * @param aObjects list of objects that are in the set that this group belongs to.
 */
void ObjectGroup::setup(ArrayPtrs<Object>& aObjects)
{
	_memberObjects.setSize(0); // clear existing contents
	for (int i=0; i<_memberNames.getSize();) {
        int index = aObjects.getIndex(_memberNames.get(i));
		if (index > -1) {
			_memberObjects.insert(i, aObjects.get(index));
			i++;
		} else {
			_memberNames.remove(i);
		}
	}
}
