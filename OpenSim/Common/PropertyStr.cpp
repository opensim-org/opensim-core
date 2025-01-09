/* -------------------------------------------------------------------------- *
 *                         OpenSim:  PropertyStr.cpp                          *
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
#include "PropertyStr.h"




using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Constructor.
 */
PropertyStr::
PropertyStr(const string &aName,const string &aValue) :
    Property_Deprecated(Property_Deprecated::Str,aName)
{
    _value = aValue;
    setAllowableListSize(1,1);
}
//_____________________________________________________________________________
/**
 * Default Constructor.
 */
PropertyStr::
PropertyStr() :
    Property_Deprecated(Property_Deprecated::Str,"StringPropertyName")
{
    _value = PropertyStr::getDefaultStr();
    setAllowableListSize(1,1);
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aProperty Property_Deprecated to be copied.
 */
PropertyStr::PropertyStr(const PropertyStr &aProperty) :
    Property_Deprecated(aProperty)
{
    _value = aProperty.getValueStr();
}
//_____________________________________________________________________________
/**
 * Construct and return a copy of this property.
 * The property is allocated using the new operator, so the caller is
 * responsible for deleting the returned object.
 *
 * @return Copy of this property.
 */
PropertyStr* PropertyStr::clone() const
{
    PropertyStr *property = new PropertyStr(*this);
    return(property);
}

void PropertyStr::assign(const AbstractProperty& that) {
    try {
        *this = dynamic_cast<const PropertyStr&>(that);
    } catch(const std::bad_cast&) {
        OPENSIM_THROW(InvalidArgument,
                      "Unsupported type. Expected: " + this->getTypeName() +
                      " | Received: " + that.getTypeName());
    }
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
PropertyStr& PropertyStr::
operator=(const PropertyStr &aProperty)
{
    Property_Deprecated::operator =(aProperty);
    _value = aProperty.getValueStr();
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
std::string PropertyStr::
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
 * @param aValue Value to which this property is assigned.
 */
void PropertyStr::
setValue(const string &aValue)
{
    _value = aValue;
}
//_____________________________________________________________________________
/**
 * Get a reference to the value of this property.  Note that the returned
 * reference can be used to change the value of this property.
 *
 * @return Reference to the value of this property.
 */
string& PropertyStr::
getValueStr()
{
    return(_value);
}
//_____________________________________________________________________________
/**
 * Get a constant reference to the value of this property.
 *
 * @return Reference to the value of this property.
 */
const string& PropertyStr::
getValueStr() const
{
    return(_value);
}
//_____________________________________________________________________________
/**
 * Get a constant String representing the value of this property.
 *
 * @return Constant String representing the value of this property.
 */
string PropertyStr::
toString() const
{
    return _value;
}
const std::string& PropertyStr::getDefaultStr() {
    static const std::string defaultValue = "Unassigned";
    return defaultValue;
}
