// Property.cpp
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
#include "Property.h"




using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Property::
Property()
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Constructor.
 */
Property::
Property(PropertyType aType,const string &aName)
{
	setNull();
	_type = aType;
	_name = aName;
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aProperty Property to be copied.
 */
Property::Property(const Property &aProperty)
{
	setNull();
	*this = aProperty;
	_comment = aProperty.getComment();
}
//_____________________________________________________________________________
/**
 * Construct and return a copy of this property.
 *
 * The property is allocated using the new operator, so the caller is
 * responsible for deleting the returned object.
 *
 * @return Copy of this property.
 *
Property* Property::copy() const
{
	Property *property = new Property(*this);
	return(property);
}
*/

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set member variables to their null values.
 */
void Property::
setNull()
{
	_type = Property::None;
	_name = "unknown";
	_useDefault = false;
	_minArraySize = 0;
	_maxArraySize = INT_MAX;
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
Property& Property::
operator=(const Property &aProperty)
{
	setType(aProperty.getType());
	setName(aProperty.getName());
	setUseDefault(aProperty.getUseDefault());
	_comment = aProperty.getComment();
	_minArraySize = aProperty._minArraySize;
	_maxArraySize = aProperty._maxArraySize;
	return(*this);
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
 * @param aProperty Property for which to make the equality test.
 * @return True if the specified property and this property are equal, false
 * otherwise.
 */
bool Property::
operator==(const Property &aProperty) const
{
	if(_name != aProperty.getName()) return(false);
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
 * @param aProperty Property for which to make the less than test.
 * @return True if this property is less than the other, false otherwise.
 */
bool Property::
operator<(const Property &aProperty) const
{
	return(_name < aProperty.getName());
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
 * @param aProperty Property to be output.
 * @return Reference to the output stream.
ostream& operator<<(ostream &aOut,const Property &aProperty)
{
	aOut << aProperty.getTypeAsString() << " " << aProperty.getName();
	return(aOut);
}
 */


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
const char* Property::
getTypeAsString() const
{
	return("None");
}

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
void Property::
setType(PropertyType aType)
{
	_type = aType;
}
//_____________________________________________________________________________
/**
 * Get the type of this property.
 *
 * @return Type of the property.
 */
Property::PropertyType Property::
getType() const
{
	return(_type);
}

//-----------------------------------------------------------------------------
// NAME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the name of this property.
 *
 * @param aName Name to which to set this property.
 */
void Property::
setName(const string &aName)
{
	_name = aName;
}
//_____________________________________________________________________________
/**
 * Get the name of this property.
 *
 * @retrun Name of this property
 */
const string& Property::
getName() const
{
	return(_name);
}

//-----------------------------------------------------------------------------
// USE DEFAULT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set whether or not this property uses a "default" property for its value.
 *
 * True means that this property should have the same value as
 * some default property.  False means that the value of this property is
 * independent of the value of any other property.
 *
 * @param aTrueFalse True means this property uses another property for its
 * value.  False means this property does not use another property for its
 * value.
 */
void Property::
setUseDefault(bool aTrueFalse)
{
	_useDefault = aTrueFalse;
}
//_____________________________________________________________________________
/**
 * Get whether or not this property uses a "default" property for its value.
 *
 * True means that this property should have the same value as
 * some default property.  False means that the value of this property is
 * independent of the value of any other property.
 *
 * @return True if this property uses another property for its value; false if
 * this property does not use another property for its value.
 */
bool Property::
getUseDefault() const
{
	return(_useDefault);
}



