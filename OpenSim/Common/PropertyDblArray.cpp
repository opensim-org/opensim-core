// PropertyDblArray.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


//============================================================================
// INCLUDES
//============================================================================
#include "PropertyDblArray.h"
#include <cstdio>




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
	Property_Deprecated(Property_Deprecated::DblArray,aName), _array(0.0)
{
	_array = aArray;
}
//_____________________________________________________________________________
/**
 * Default Constructor.
 */
PropertyDblArray::
PropertyDblArray() :
	Property_Deprecated(Property_Deprecated::DblArray,"BoolArrayPropertyName"), _array(0.0)
{

}
//_____________________________________________________________________________
/**
 * Constructor.
 */
PropertyDblArray::
PropertyDblArray(const string &aName,
	int aSize,const double aArray[]) :
	Property_Deprecated(Property_Deprecated::DblArray,aName), _array(0.0)
{
	if(aSize<=0) return;
	if(aArray==NULL) return;
	_array.append(aSize,aArray);
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aProperty Property_Deprecated to be copied.
 */
PropertyDblArray::PropertyDblArray(const PropertyDblArray &aProperty) :
	Property_Deprecated(aProperty), _array(0.0)
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
PropertyDblArray* PropertyDblArray::clone() const
{
	PropertyDblArray *property = new PropertyDblArray(*this);
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
 * @param aProperty Property_Deprecated to which to assign this property.
 * @return Reference to this property.
 */
PropertyDblArray& PropertyDblArray::
operator=(const PropertyDblArray &aProperty)
{
	Property_Deprecated::operator =(aProperty);
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
std::string PropertyDblArray::
getTypeName() const
{
	return("double");
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
 * Get a constant String representing the value of this property.
 *
 * @return Constant String represeting the value of this property.
 */
string PropertyDblArray::
toString() const
{
	string str = "(";
	char dbl[256];
	for(int i=0; i < _array.getSize(); i++){
		sprintf(dbl, "%g", _array[i]);
		str += (i>0?" ":"") + string(dbl);
	}
	str += ")";
	return str;
}
