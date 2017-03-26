/* -------------------------------------------------------------------------- *
 *                      OpenSim:  PropertyBoolArray.cpp                       *
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
    Property_Deprecated(Property_Deprecated::BoolArray,aName), _array(0)
{
    _array = aArray;
}
//_____________________________________________________________________________
/**
 * Default Constructor.
 */
PropertyBoolArray::
PropertyBoolArray() :
    Property_Deprecated(Property_Deprecated::BoolArray,"BoolArrayPropertyName"), _array(0)
{
}
//_____________________________________________________________________________
/**
 * Constructor.
 */
PropertyBoolArray::
PropertyBoolArray(const string &aName,
    int aSize,const bool aArray[]) :
    Property_Deprecated(Property_Deprecated::BoolArray,aName), _array(0)
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
PropertyBoolArray::PropertyBoolArray(const PropertyBoolArray &aProperty) :
    Property_Deprecated(aProperty), _array(0)
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
PropertyBoolArray* PropertyBoolArray::clone() const
{
    PropertyBoolArray *prop = new PropertyBoolArray(*this);
    return prop;
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
PropertyBoolArray& PropertyBoolArray::
operator=(const PropertyBoolArray &aProperty)
{
    Property_Deprecated::operator =(aProperty);
    _array = aProperty._array;
    return(*this);
}

void PropertyBoolArray::assign(const AbstractProperty& that) {
    try {
        *this = dynamic_cast<const PropertyBoolArray&>(that);
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
std::string PropertyBoolArray::
getTypeName() const
{
    return "bool";
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
 * Get a constantString representing the value of this property.
 *
 * @return Constant String representing the value of this property.
 */
string PropertyBoolArray::
toString() const
{
    string str = "(";
    for(int i=0; i < _array.getSize(); i++)
        str += (i>0?" ":"") + (_array[i]?string("True"):string("False"));
    str += ")";
    return str;
}
