// PropertyGroup.cpp
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
PropertyGroup* PropertyGroup::
copy() const
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
add(Property* aProperty)
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
remove(Property* aProperty)
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
Property* PropertyGroup::
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
getPropertyIndex(Property* aProperty) const
{
	for (int i = 0; i < _properties.getSize(); i++)
		if (_properties.get(i) == aProperty)
			return i;

	return -1;
}
