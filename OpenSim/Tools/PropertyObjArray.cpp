// PropertyObjArray.cpp
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
#include "PropertyObjArray.h"
#include "Object.h"




using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 *
 * Construct an empty object-array property without a name.
 */
PropertyObjArray::
PropertyObjArray() :
	Property(Property::ObjArray,"")
{
	_array.setSize(0);
}
//_____________________________________________________________________________
/**
 * Constructor.
 *
 * Construct an empty object-array property with a specified name.
 *
 * @param aName Name of the object array.
 */
PropertyObjArray::
PropertyObjArray(const string &aName) :
	Property(Property::ObjArray,aName)
{
	_array.setSize(0);
}
//_____________________________________________________________________________
/**
 * Constructor.
 *
 * Construct an object-array property with a specified name initialized to
 * a specified array of objects.  Note that copies of the objects in the
 * specified array are made.
 *
 * @param aName Name of the object array.
 * @param aArray Array used to initialize the object array.
 */
PropertyObjArray::
PropertyObjArray(const string &aName,const ArrayPtrs<Object> &aArray) :
	Property(Property::ObjArray,aName)
{
	_array = aArray;
}
//_____________________________________________________________________________
/**
 * Constructor.
 */
PropertyObjArray::
PropertyObjArray(const string &aName,
	int aSize,const Object **aArray) :
	Property(Property::ObjArray,aName)
{
	if(aSize<=0) return;
	if(aArray==NULL) return;
	for(int i=0;i<aSize;i++) {
		_array.append( aArray[i]->copy() );
	}
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aProperty Property to be copied.
 */
PropertyObjArray::PropertyObjArray(const PropertyObjArray &aProperty) :
	Property(aProperty)
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
Property* PropertyObjArray::copy() const
{
	Property *property = new PropertyObjArray(*this);
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
PropertyObjArray& PropertyObjArray::
operator=(const PropertyObjArray &aProperty)
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
const char* PropertyObjArray::
getTypeAsString() const
{
	return("ObjArray");
}

//-----------------------------------------------------------------------------
// VALUE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the value of this property.
 *
 * @param aArray Array to which this property is to be assigned.  Note
 * that copies of the objects in the specified array are NOT made.
 */
void PropertyObjArray::
setValue(const ArrayPtrs<Object>& aArray)
{
	_array = aArray;
}
//_____________________________________________________________________________
/**
 * Set the value of this property.
 *
 * @param aSize Size of the specified array.
 * @param aValue Array to which this property is to be assigned.  Note
 * that copies of the objects in the specified array are NOT made.
 */
void PropertyObjArray::
setValue(int aSize,Object** aArray)
{
	_array.setSize(0);
	if(aSize<=0) return;
	if(aArray==NULL) return;
	for(int i=0;i<aSize;i++) {
		_array.append(aArray[i]);
		aArray++;
	}
}
//_____________________________________________________________________________
/**
 * Get a reference to the value of this property.  Note that the returned
 * reference can be used to change the value of this property.
 *
 * @return Reference to the value of this property.
 */
ArrayPtrs<Object>& PropertyObjArray::
getValueObjArray()
{
	return(_array);
}
//_____________________________________________________________________________
/**
 * Get a constant reference to the value of this property.
 *
 * @return Reference to the value of this property.
 */
const ArrayPtrs<Object>& PropertyObjArray::
getValueObjArray() const
{
	return(_array);
}
//_____________________________________________________________________________
/**
 * Get a constant String represeting the value of this property.
 *
 * @return Constant String represeting the value of this property.
 */
const string &PropertyObjArray::
toString()
{
	_valueString = "(Array of objects)";
	return (_valueString);
}
