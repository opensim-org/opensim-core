/* -------------------------------------------------------------------------- *
 *                       OpenSim:  PropertyDblArray.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include "PropertyDblArray.h"
#include "Array.h"
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

void PropertyDblArray::assign(const AbstractProperty& that) {
    try {
        *this = dynamic_cast<const PropertyDblArray&>(that);
    } catch(const std::bad_cast&) {
        OPENSIM_THROW(InvalidArgument,
                      "Unsupported type. Expected: " + this->getTypeName() +
                      " | Received: " + that.getTypeName());
    }
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
 * @return Constant String representing the value of this property.
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
