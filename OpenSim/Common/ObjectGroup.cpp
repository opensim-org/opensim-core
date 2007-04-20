// ObjectGroup.cpp
// Author: Peter Loan
/*
 * Copyright (c) 2007, Stanford University. All rights reserved. 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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
ObjectGroup::ObjectGroup(string& aName) :
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

//_____________________________________________________________________________
/**
 * Copy this muscle group and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this group.
 */
Object* ObjectGroup::copy() const
{
	ObjectGroup *grp = new ObjectGroup(*this);
	return(grp);
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
	_memberObjects = aGroup._memberObjects;
	// because ArrayPtrs::operator= sets MemoryOwner to true
	_memberObjects.setMemoryOwner(false);
}

//_____________________________________________________________________________
/**
 * Set the data members of this ObjectGroup to their null values.
 */
void ObjectGroup::setNull()
{
	_memberObjects.setSize(0);
	_memberObjects.setMemoryOwner(false);

	setType("ObjectGroup");
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
	int i;
	for (i = 0; i < _memberObjects.getSize(); i++)
		if (_memberObjects[i]->getName() == aName)
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
		int i;
		for (i=0; i<_memberObjects.getSize(); i++) {
			if (aObject == _memberObjects.get(i)) {
				// object is already a member of this group
				return;
			}
		}

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
		int i;
		for (i=0; i<_memberObjects.getSize(); i++) {
			if (aObject == _memberObjects.get(i)) {
				_memberObjects.remove(i);
				_memberNames.remove(i);
			}
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
	int i;
	for (i=0; i<_memberNames.getSize();) {
		Object* obj = aObjects.get(_memberNames.get(i));
		if (obj != NULL) {
			_memberObjects.insert(i, obj);
			i++;
		} else {
			_memberNames.remove(i);
		}
	}
}

void ObjectGroup::peteTest() const
{
	int i;
	cout << "Group " << getName() << ":" << endl;
	for (i = 0; i < _memberNames.getSize(); i++)
		cout << "  " << i+1 << " " << _memberNames.get(i) << " (obj= " << _memberObjects.get(i)->getName() << ")" << endl;
}
