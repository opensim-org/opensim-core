// PropertyGroup.cpp
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
