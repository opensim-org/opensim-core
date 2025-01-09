/* -------------------------------------------------------------------------- *
 *                        OpenSim:  PropertyGroup.cpp                         *
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
#include "PropertyGroup.h"

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
PropertyGroup::
PropertyGroup() :
    _properties(NULL)
{
    setNull();
}

//_____________________________________________________________________________
/**
 * Constructor taking the group name but no member names.
 */
PropertyGroup::
PropertyGroup(string& aName) :
    _properties(NULL)
{
    setName(aName);
    setNull();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
PropertyGroup::
~PropertyGroup()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aGroup Group to be copied.
 */
PropertyGroup::
PropertyGroup(const PropertyGroup &aGroup) :
    _properties(NULL)
{
    copyData(aGroup);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one PropertyGroup to another.
 *
 * @param aGroup PropertyGroup to be copied.
 */
void PropertyGroup::
copyData(const PropertyGroup &aGroup)
{
    _name = aGroup._name;
    _properties = aGroup._properties;
}

//_____________________________________________________________________________
/**
 * Clear the group (remove all property members).
 */
void PropertyGroup::
clear()
{
    _properties.setSize(0);
    _properties.trim();
}

//_____________________________________________________________________________
/**
 * Set the data members of this PropertyGroup to their null values.
 */
void PropertyGroup::
setNull()
{
}

//_____________________________________________________________________________
/**
 * Construct and return a copy of this object.
 *
 * The object is allocated using the new operator, so the caller is
 * responsible for deleting the returned object.
 *
 * @return Copy of this object.
 */
PropertyGroup* PropertyGroup::clone() const
{
    PropertyGroup *propertyGroup = new PropertyGroup(*this);
    return(propertyGroup);
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
PropertyGroup& PropertyGroup::
operator=(const PropertyGroup &aGroup)
{
    copyData(aGroup);

    return(*this);
}

//_____________________________________________________________________________
/**
 * Less than operator.
 *
 * @param aGroup Group with which to evaluate less than.
 * @return True if number of properties in this group is less than the number
 *         in aGroup.
 */
bool PropertyGroup::
operator<(const PropertyGroup& aGroup) const
{
    if (_properties.getSize() < aGroup.getProperties().getSize())
        return true;
    else
        return false;
}
//_____________________________________________________________________________
/**
 * Equality operator.
 *
 * @param aGroup Group with which to evaluate less than.
 * @return True if the two groups have the same number of properties.
 */
bool PropertyGroup::
operator==(const PropertyGroup& aGroup) const
{
    if (_properties.getSize() == aGroup.getProperties().getSize())
        return true;
    else
        return false;
}

//_____________________________________________________________________________
/**
 * Check if the group contains an object with a certain name.
 *
 * @param aName the name of the object.
 * @return Boolean indicating whether or not the group contains the object.
 */
bool PropertyGroup::
contains(const string& aName) const
{
    for (int i = 0; i < _properties.getSize(); i++)
        if (_properties.get(i)->getName() == aName)
            return true;

    return false;
}

//_____________________________________________________________________________
/**
 * Add a property, if it is not already in the group.
 *
 */
void PropertyGroup::
add(Property_Deprecated* aProperty)
{
    if (_properties.findIndex(aProperty) < 0)
        _properties.append(aProperty);
}

//_____________________________________________________________________________
/**
 * Remove a property.
 *
 */
void PropertyGroup::
remove(Property_Deprecated* aProperty)
{
    int index = getPropertyIndex(aProperty);
    if (index >= 0)
        _properties.remove(index);
}

//_____________________________________________________________________________
/**
 * Get a property by index.
 *
 */
Property_Deprecated* PropertyGroup::
get(int aIndex)
{
    if (aIndex >= 0 && aIndex < _properties.getSize())
        return _properties.get(aIndex);

    return NULL;
}

//_____________________________________________________________________________
/**
 * Get the index of a property.
 *
 */
int PropertyGroup::
getPropertyIndex(Property_Deprecated* aProperty) const
{
    for (int i = 0; i < _properties.getSize(); i++)
        if (_properties.get(i) == aProperty)
            return i;

    return -1;
}
