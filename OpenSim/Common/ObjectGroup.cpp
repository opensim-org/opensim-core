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
	return _memberObjects.getIndex(aName) != -1;
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
		if (_memberObjects.getIndex(aObject) != -1) return;

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
		for (int i=0; i<_memberObjects.getSize(); i++) {
			if (aObject == _memberObjects.get(i)) {
				_memberObjects.remove(i);
				_memberNames.remove(i);
			}
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
		for (int i=0; i<_memberObjects.getSize(); i++) {
			if (aOldObject == _memberObjects.get(i)) {
				_memberObjects.remove(i);
				_memberNames.remove(i);
				_memberObjects.insert(i, aNewObject);
				_memberNames.insert(i, aNewObject->getName());
				break;
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
	_memberObjects.setSize(0); // clear existing contents
	for (int i=0; i<_memberNames.getSize();) {
		Object* obj = aObjects.get(_memberNames.get(i));
		if (obj != NULL) {
			_memberObjects.insert(i, obj);
			i++;
		} else {
			_memberNames.remove(i);
		}
	}
}
