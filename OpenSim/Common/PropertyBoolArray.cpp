// PropertyBoolArray.cpp
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
#include "PropertyBoolArray.h"




using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Constructor.
 */
PropertyBoolArray::
PropertyBoolArray(const string &aName,
	const Array<bool> &aArray) :
	Property(Property::BoolArray,aName), _array(0)
{
	_array = aArray;
}
//_____________________________________________________________________________
/**
 * Default Constructor.
 */
PropertyBoolArray::
PropertyBoolArray() :
	Property(Property::BoolArray,"BoolArrayPropertyName"), _array(0)
{
}
//_____________________________________________________________________________
/**
 * Constructor.
 */
PropertyBoolArray::
PropertyBoolArray(const string &aName,
	int aSize,const bool aArray[]) :
	Property(Property::BoolArray,aName), _array(0)
{
	if(aSize<=0) return;
	if(aArray==NULL) return;
	_array.append(aSize,aArray);
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aProperty Property to be copied.
 */
PropertyBoolArray::PropertyBoolArray(const PropertyBoolArray &aProperty) :
	Property(aProperty), _array(0)
{
	_array = aProperty._array;
}
//_____________________________________________________________________________
/**
 * Construct and return a copy of this property.
 * The property is allocated using the new operator, so the caller is
 * responsible for deleting the returned object.
 *
 * @return Copy of this property.
 */
Property* PropertyBoolArray::copy() const
{
	Property *property = new PropertyBoolArray(*this);
	return(property);
}


//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assign this property to another.
 *
 * @param aProperty Property to which to assign this property.
 * @return Reference to this property.
 */
PropertyBoolArray& PropertyBoolArray::
operator=(const PropertyBoolArray &aProperty)
{
	Property::operator =(aProperty);
	_array = aProperty._array;
	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// TYPE AS STRING
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the type of this property as a string.
 *
 * @return Type of the property.
 */
const char* PropertyBoolArray::
getTypeAsString() const
{
	return("BoolArray");
}

//-----------------------------------------------------------------------------
// VALUE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the value of this property.
 *
 * @param aArray Array to which this property is to be assigned.
 */
void PropertyBoolArray::
setValue(const Array<bool>& aArray)
{
	_array = aArray;
}
//_____________________________________________________________________________
/**
 * Set the value of this property.
 *
 * @param aSize Size of the specified array.
 * @param aValue Array to which this property is to be assigned.
 */
void PropertyBoolArray::
setValue(int aSize,const bool aArray[])
{
	_array.setSize(0);
	if(aSize<=0) return;
	if(aArray==NULL) return;
	_array.append(aSize,aArray);
}
//_____________________________________________________________________________
/**
 * Get a reference to the value of this property.  Note that the returned
 * reference can be used to change the value of this property.
 *
 * @return Reference to the value of this property.
 */
Array<bool>& PropertyBoolArray::
getValueBoolArray()
{
	return(_array);
}
//_____________________________________________________________________________
/**
 * Get a constant reference to the value of this property.
 *
 * @return Reference to the value of this property.
 */
const Array<bool>& PropertyBoolArray::
getValueBoolArray() const
{
	return(_array);
}
//_____________________________________________________________________________
/**
 * Get a constantString represeting the value of this property.
 *
 * @return Constant String represeting the value of this property.
 */
const string &PropertyBoolArray::
toString()
{
	string str = "(";
	for(int i=0; i < _array.getSize(); i++)
		str += (i>0?" ":"") + (_array[i]?string("True"):string("False"));
	str += ")";
	_valueString = str;
	return (_valueString);
}
