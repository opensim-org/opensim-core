/* -------------------------------------------------------------------------- *
 *                     OpenSim:  Property_Deprecated.cpp                      *
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
#include "Property_Deprecated.h"



using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Property_Deprecated::
Property_Deprecated() : AbstractProperty()
{
    setNull();
}
//_____________________________________________________________________________
/**
 * Constructor.
 */
Property_Deprecated::
Property_Deprecated(PropertyType aType, const string &aName) 
:   AbstractProperty()
{
    setNull();
    setName(aName);
    _propertyType = aType;
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aProperty Property_Deprecated to be copied.
 */
Property_Deprecated::Property_Deprecated(const Property_Deprecated &aProperty)
:   AbstractProperty(aProperty)
{
    setNull();
    *this = aProperty;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set member variables to their null values.
 */
void Property_Deprecated::
setNull()
{
    _propertyType = Property_Deprecated::None;
    _matchName = false;
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
Property_Deprecated& Property_Deprecated::
operator=(const Property_Deprecated& aProperty)
{
    AbstractProperty::operator=(aProperty);
    _propertyType = aProperty._propertyType;
    _matchName = aProperty._matchName;

    return *this;
}

//-----------------------------------------------------------------------------
// EQUALITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Determine if two properties are equal.
 *
 * Two properties are equal if their names are the same; the types do not
 * need to be the same.
 *
 * @param aProperty Property_Deprecated for which to make the equality test.
 * @return True if the specified property and this property are equal, false
 * otherwise.
 */
bool Property_Deprecated::
operator==(const Property_Deprecated &aProperty) const
{
    if(getName() != aProperty.getName()) return(false);
    return(true);
}

//-----------------------------------------------------------------------------
// LESS THAN
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Determine if this property is less than another.
 *
 * This property is less than another if the name of this string is less
 * than the name of the other property.
 *
 * @param aProperty Property_Deprecated for which to make the less than test.
 * @return True if this property is less than the other, false otherwise.
 */
bool Property_Deprecated::
operator<(const Property_Deprecated &aProperty) const
{
    return(getName() < aProperty.getName());
}

//-----------------------------------------------------------------------------
// OUTPUT (<<)
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Implementation of the output operator.
 * The output consists of the type and name:\n\n
 *
 * @param aOut Output stream.
 * @param aProperty Property_Deprecated to be output.
 * @return Reference to the output stream.
ostream& operator<<(ostream &aOut,const Property_Deprecated &aProperty)
{
    aOut << aProperty.getTypeName() << " " << aProperty.getName();
    return(aOut);
}
 */


//=============================================================================
// GET AND SET
//=============================================================================

//-----------------------------------------------------------------------------
// TYPE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the type of this property.
 *
 * @param aType Type of this object represented as a string.  In most all
 * cases, the type should be the name of the class.
 */
void Property_Deprecated::
setType(PropertyType aType)
{
    _propertyType = aType;
}
//_____________________________________________________________________________
/**
 * Get the type of this property.
 *
 * @return Type of the property.
 */
Property_Deprecated::PropertyType Property_Deprecated::
getType() const
{
    return _propertyType;
}




