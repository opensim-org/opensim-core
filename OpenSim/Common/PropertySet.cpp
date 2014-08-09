/* -------------------------------------------------------------------------- *
 *                         OpenSim:  PropertySet.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
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

/* Note: This code was originally developed by Realistic Dynamics Inc.
 * Author: Frank C. Anderson
 */


//============================================================================
// INCLUDES
//============================================================================
#include "PropertySet.h"
#include "PropertyInt.h"



using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Constructor.
 */
PropertySet::PropertySet()
{
    _array.setMemoryOwner(false);
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aSet Set of properties to be copied.
 */
PropertySet::PropertySet(const PropertySet &aSet)
{
    _array = aSet._array;
    _array.setMemoryOwner(false);

}


//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// OUTPUT (<<)
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Implementation of the output operator.
 *
 * @param aOut Output stream.
 * @param aArray Array to be output.
 * @return Reference to the output stream.
ostream&
operator<<(ostream &aOut,const PropertySet &aSet)
{
	aOut << "\nProperty Set:\n";

	int i;
	for(i=0;i<aSet.getSize();i++)  {
		aOut << aSet.get(i) << "\n";
	}

	return(aOut);
}
 */


//=============================================================================
// EMPTY?
//=============================================================================
//_____________________________________________________________________________
/**
 * Determine whether or not this property set is empty.
 *
 * @returns True if empty; false otherwise.
 */
bool PropertySet::
isEmpty() const
{
    if(_array.getSize()<=0) return(true);
    return(false);
}


//=============================================================================
// SIZE
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the number of properties in the set, or, equivalently, the size of
 * the property set.
 *
 * @returns Size or number of properties in this set.
 */
int PropertySet::
getSize() const
{
    return(_array.getSize());
}


//=============================================================================
// GET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get a reference to a property by index.
 *
 * @param aIndex Index of the property to get.
 * @throws Exception if the index is out of bounds.
 */
Property_Deprecated* PropertySet::
get(int aIndex) throw(Exception)
{
    // NO SUCH PROPERTY - THROW EXCEPTION
    if((aIndex<0)||(aIndex>=_array.getSize())) {
        string msg = "PropertySet.get(int): Index is out of bounds.";
        throw Exception(msg,__FILE__,__LINE__);
    }

    return(_array[aIndex]);
}
//_____________________________________________________________________________
/**
 * Get a constant reference to a property by index.
 *
 * @param aIndex Index of the property to get.
 * @throws Exception if the index is out of bounds.
 */
const Property_Deprecated* PropertySet::
get(int aIndex) const
{
    // NO SUCH PROPERTY - THROW EXCEPTION
    if((aIndex<0)||(aIndex>=_array.getSize())) {
        string msg = "PropertySet.get(int): Index is out of bounds.";
        throw Exception(msg,__FILE__,__LINE__);
    }

    return(_array[aIndex]);
}
//_____________________________________________________________________________
/**
 * Get the first property in the set that has a specified name.
 *
 * @param aName Name of the property to get.
 * @throws Exception if there is no such property.
 */
Property_Deprecated* PropertySet::
get(const string &aName) throw(Exception)
{
    int i;
    PropertyInt prop(aName,0);
    for(i=0; i<_array.getSize(); i++) {
        if((*_array[i]) == prop) return(_array[i]);
    }

    // NO SUCH PROPERTY - THROW EXCEPTION
    string msg = "PropertySet.get: No property named " + aName;
    throw Exception(msg,__FILE__,__LINE__);
}
//_____________________________________________________________________________
/**
 * Get the first property in the set that has a specified name.
 *
 * @param aName Name of the property to get.
 * @throws Exception if there is no such property.
 */
const Property_Deprecated* PropertySet::
get(const string &aName) const
{
    int i;
    PropertyInt prop(aName,0);
    for(i=0; i<_array.getSize(); i++) {
        if((*_array[i]) == prop) return(_array[i]);
    }

    // NO SUCH PROPERTY - THROW EXCEPTION
    string msg = "PropertySet.get: No property named " + aName;
    throw Exception(msg,__FILE__,__LINE__);
}
//_____________________________________________________________________________
/**
 * If the set contains a property with the specified name, return it.
 *
 * @param aName Name of the property to get.
 * @return Pointer to the property.
 */
const Property_Deprecated* PropertySet::
contains(const string& aName) const
{
    int i;
    PropertyInt prop(aName,0);
    for(i=0; i<_array.getSize(); i++) {
        if((*_array[i]) == prop) return(_array[i]);
    }
    return NULL;
}
//_____________________________________________________________________________
/**
 * If the set contains a property with the specified name, return it.
 *
 * @param aName Name of the property to get.
 * @return Pointer to the property.
 */
Property_Deprecated* PropertySet::
contains(const string& aName)
{
    int i;
    PropertyInt prop(aName,0);
    for(i=0; i<_array.getSize(); i++) {
        if((*_array[i]) == prop) return(_array[i]);
    }
    return NULL;
}


//=============================================================================
// APPEND
//=============================================================================
//_____________________________________________________________________________
/**
 * Append a property to the set.
 *
 * @param aProperty Property_Deprecated to be appended.  Note that a copy is NOT made.
 */
void PropertySet::
append(Property_Deprecated *aProperty)
{
    _array.append(aProperty);
}

//_____________________________________________________________________________
/**
 * Append a property to the set, and put it in the specified group.
 *
 * @param aProperty Property_Deprecated to be appended.  Note that a copy is NOT made.
 */
void PropertySet::
append(Property_Deprecated *aProperty, const string& aName)
{
    _array.append(aProperty);

    // Add the property to the specified group, creating the group if necessary.
    PropertyGroup* group = addGroup(aName);
    group->add(aProperty);
}

//=============================================================================
// REMOVE
//=============================================================================
//_____________________________________________________________________________
/**
 * Remove the first property in the set that has a specified name.
 *
 * @param aName Name of the property to remove.
 * @throws Exception if there is no such property.
 */
void PropertySet::
remove(const string &aName)
{
    int i;
    PropertyInt prop(aName,0);
    for(i=0; i<_array.getSize(); i++) {
        if((*_array[i]) == prop) {
            // Found a match, so remove the property from all the groups,
            // and then remove the property.
            int j;
            for (j=0; j <_propertyGroups.getSize(); j++) {
                _propertyGroups.get(j)->remove(_array[i]);
            }
            _array.remove(i);
            return;
        }
    }

    // NO SUCH PROPERTY - THROW EXCEPTION
    string msg = "PropertySet.get: No property named " + aName;
    throw Exception(msg,__FILE__,__LINE__);
}


//=============================================================================
// CLEAR
//=============================================================================
//_____________________________________________________________________________
/**
 * Clear this property set of all properties and groups.
 */
void PropertySet::
clear()
{
    // Remove all the properties.
    _array.setSize(0);
    _array.trim();

    // Remove all the groups.
    _propertyGroups.setSize(0);
    _propertyGroups.trim();
}

//_____________________________________________________________________________
/**
 * Add an empty group to the set, if it is not already in there.
 */
PropertyGroup* PropertySet::
addGroup(string aGroupName)
{
    int index = _propertyGroups.getIndex(aGroupName);
    if (index == -1) {
        PropertyGroup* group = new PropertyGroup(aGroupName);
        _propertyGroups.append(group);
        return group;
    }
    return _propertyGroups.get(index);
}

//_____________________________________________________________________________
/**
 * Add a property to a group. The group will be created if it does not already
 * exist, but the property must already be in the PropertySet for it to be
 * added to the group.
 */
void PropertySet::
addPropertyToGroup(std::string aGroupName, std::string aPropertyName)
{
    Property_Deprecated* prop = _array.get(aPropertyName);
    if (prop)
    {
        PropertyGroup* group = _propertyGroups.get(aGroupName);
        if (group == NULL)
            group = addGroup(aGroupName);
        group->add(prop);
    }
}

//_____________________________________________________________________________
/**
 * Add a property to a group. The group and the property must already be in the
 * PropertySet.
 */
void PropertySet::
addPropertyToGroup(PropertyGroup* aGroup, std::string aPropertyName)
{
    // Make sure the group and property are in the PropertySet before adding the
    // property to the group.
    Property_Deprecated* prop = _array.get(aPropertyName);
    if (prop && aGroup)
    {
        int i;
        for (i = 0; i < _propertyGroups.getSize(); i++) {
            if (_propertyGroups.get(i) == aGroup) {
                aGroup->add(prop);
            }
        }
    }
}

//_____________________________________________________________________________
/**
 * Add a property to a group. The group and the property must already be in the
 * PropertySet.
 */
void PropertySet::
addPropertyToGroup(PropertyGroup* aGroup, Property_Deprecated* aProperty)
{
    // Make sure the group and property are in the PropertySet before adding the
    // property to the group.
    int index = _array.getIndex(aProperty);
    if (index >= 0 && aGroup)
    {
        int i;
        for (i = 0; i < _propertyGroups.getSize(); i++) {
            if (_propertyGroups.get(i) == aGroup) {
                aGroup->add(_array[index]);
            }
        }
    }
}

//_____________________________________________________________________________
/**
 * Add a property to a group. The group will be created if it does not already
 * exist, but the property must already be in the PropertySet for it to be
 * added to the group.
 */
void PropertySet::
addPropertyToGroup(std::string aGroupName, Property_Deprecated* aProperty)
{
    int index = _array.getIndex(aProperty);
    if (index >= 0)
    {
        PropertyGroup* group = _propertyGroups.get(aGroupName);
        if (group == NULL)
            group = addGroup(aGroupName);
        group->add(aProperty);
    }
}

//_____________________________________________________________________________
/**
 * Get the group containing the passed-in property.
 */
PropertyGroup* PropertySet::
getGroupContaining(Property_Deprecated* aProperty)
{
    int i;
    for (i = 0; i < _propertyGroups.getSize(); i++) {
        if (_propertyGroups[i]->getPropertyIndex(aProperty) >= 0)
            return _propertyGroups.get(i);
    }

    return NULL;
}

//_____________________________________________________________________________
/**
 * Get the index of the group containing the passed-in property.
 */
int PropertySet::
getGroupIndexContaining(Property_Deprecated* aProperty)
{
    int i;
    for (i = 0; i < _propertyGroups.getSize(); i++) {
        if (_propertyGroups.get(i)->getPropertyIndex(aProperty) >= 0)
            return i;
    }

    return -1;
}
