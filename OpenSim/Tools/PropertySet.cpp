// PropertySet.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

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
Property* PropertySet::
get(int aIndex)
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
const Property* PropertySet::
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
Property* PropertySet::
get(const string &aName)
{
	int i;
	PropertyInt prop(aName,0);
	for(i=0;i<_array.getSize();i++) {
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
const Property* PropertySet::
get(const string &aName) const
{
	int i;
	PropertyInt prop(aName,0);
	for(i=0;i<_array.getSize();i++) {
		if((*_array[i]) == prop) return(_array[i]);
	}
		
	// NO SUCH PROPERTY - THROW EXCEPTION
	string msg = "PropertySet.get: No property named " + aName;
	throw Exception(msg,__FILE__,__LINE__);
}


//=============================================================================
// APPEND
//=============================================================================
//_____________________________________________________________________________
/**
 * Append a property to the set.
 *
 * @param aProperty Property to be appended.  Note that a copy is NOT made.
 */
void PropertySet::
append(Property *aProperty)
{
	_array.append(aProperty);
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
	for(i=0;i<_array.getSize();i++) {
		if((*_array[i]) == prop) {
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
 * Clear this property set of all properties.
 */
void PropertySet::
clear()
{
	_array.setSize(0);
	_array.trim();
}
