// PropertyDblArray.cpp
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
#include "PropertyDblArray.h"




using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Constructor.
 */
PropertyDblArray::
PropertyDblArray(const string &aName,
	const Array<double> &aArray) :
	Property(Property::DblArray,aName), _array(0.0)
{
	_array = aArray;
}
//_____________________________________________________________________________
/**
 * Default Constructor.
 */
PropertyDblArray::
PropertyDblArray() :
	Property(Property::DblArray,"BoolArrayPropertyName"), _array(0.0)
{

}
//_____________________________________________________________________________
/**
 * Constructor.
 */
PropertyDblArray::
PropertyDblArray(const string &aName,
	int aSize,const double aArray[]) :
	Property(Property::DblArray,aName), _array(0.0)
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
PropertyDblArray::PropertyDblArray(const PropertyDblArray &aProperty) :
	Property(aProperty), _array(0.0)
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
Property* PropertyDblArray::copy() const
{
	Property *property = new PropertyDblArray(*this);
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
PropertyDblArray& PropertyDblArray::
operator=(const PropertyDblArray &aProperty)
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
const char* PropertyDblArray::
getTypeAsString() const
{
	return("DblArray");
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
void PropertyDblArray::
setValue(const Array<double>& aArray)
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
void PropertyDblArray::
setValue(int aSize,const double aArray[])
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
Array<double>& PropertyDblArray::
getValueDblArray()
{
	return(_array);
}
//_____________________________________________________________________________
/**
 * Get a constant reference to the value of this property.
 *
 * @return Reference to the value of this property.
 */
const Array<double>& PropertyDblArray::
getValueDblArray() const
{
	return(_array);
}
//_____________________________________________________________________________
/**
 * Get value at specific index.
 *
 * @return Reference to the value of this property at specified index.
 */
double& PropertyDblArray::getDblValueAt(int aIndex) const
{
	return _array[aIndex];
}
//_____________________________________________________________________________
/**
 * Set value at specific index.
 *
 * 
 */
void PropertyDblArray::setDblValueAt(int aIndex, const double aDouble) const
{
	_array[aIndex] = aDouble;
}
//_____________________________________________________________________________
/**
 * Get the size of Array used by property.
 *
 * @return size of Array used by property.
 */
const int PropertyDblArray::getArraySize() const
{
	return _array.getSize();
}
//_____________________________________________________________________________
/**
 * Get a constant String represeting the value of this property.
 *
 * @return Constant String represeting the value of this property.
 */
const string &PropertyDblArray::
toString()
{
	string str = "(";
	char dbl[32];
	for(int i=0; i < _array.getSize(); i++){
		sprintf (dbl, "%lf\n", _array[i]);
		str += dbl;
	}
	str += ")";
	_valueString = str;
	return (_valueString);
}
