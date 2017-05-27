/* -------------------------------------------------------------------------- *
 *                         OpenSim:  ObjectGroup.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

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
void ObjectGroup::add(const Object* aObject)
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
void ObjectGroup::replace(const Object* aOldObject, const Object* aNewObject)
{
    if (aOldObject != NULL && aNewObject != NULL)
    {
        int index = _memberObjects.findIndex(const_cast<Object*>(aOldObject));
        if(index >= 0) {
            _memberObjects.updElt(index) = aNewObject;
            _memberNames.updElt(index) = aNewObject->getName();
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
void ObjectGroup::setupGroup(ArrayPtrs<Object>& aObjects)
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
