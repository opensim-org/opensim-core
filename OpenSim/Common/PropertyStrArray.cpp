/* -------------------------------------------------------------------------- *
 *                       OpenSim:  PropertyStrArray.cpp                       *
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
#include "PropertyStrArray.h"
#include "Array.h"



using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Constructor.
 */
PropertyStrArray::
PropertyStrArray(const string &aName,
    const Array<string> &aArray) :
    Property_Deprecated(Property_Deprecated::StrArray,aName), _array("")
{
    _array = aArray;
}
//_____________________________________________________________________________
/**
 * Default Constructor.
 */
PropertyStrArray::
PropertyStrArray() :
    Property_Deprecated(Property_Deprecated::StrArray,"StrArrayPropertyName"), _array("")
{
}
//_____________________________________________________________________________
/**
 * Constructor.
 */
PropertyStrArray::
PropertyStrArray(const string &aName,
    int aSize,const string *aArray) :
    Property_Deprecated(Property_Deprecated::StrArray,aName), _array("")
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
PropertyStrArray::PropertyStrArray(const PropertyStrArray &aProperty) :
    Property_Deprecated(aProperty), _array("")
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
PropertyStrArray* PropertyStrArray::clone() const
{
    PropertyStrArray *property = new PropertyStrArray(*this);
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
PropertyStrArray& PropertyStrArray::
operator=(const PropertyStrArray &aProperty)
{
    Property_Deprecated::operator =(aProperty);
    _array = aProperty._array;
    return(*this);
}

void PropertyStrArray::assign(const AbstractProperty& that) {
    try {
        *this = dynamic_cast<const PropertyStrArray&>(that);
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
std::string PropertyStrArray::
getTypeName() const
{
    return "string";
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
void PropertyStrArray::
setValue(const Array<string>& aArray)
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
void PropertyStrArray::
setValue(int aSize,const string *aArray)
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
Array<string>& PropertyStrArray::
getValueStrArray()
{
    return(_array);
}
//_____________________________________________________________________________
/**
 * Get a constant reference to the value of this property.
 *
 * @return Reference to the value of this property.
 */
const Array<string>& PropertyStrArray::
getValueStrArray() const
{
    return(_array);
}
//_____________________________________________________________________________
/**
 * Get a constant String representing the value of this property.
 *
 * @return Constant String representing the value of this property.
 */
string PropertyStrArray::
toString() const
{
    string str = "(";
    for(int i=0; i < _array.getSize(); i++)
        str += (i>0?" ":"") + _array[i];
    str += ")";
    return str;
}
