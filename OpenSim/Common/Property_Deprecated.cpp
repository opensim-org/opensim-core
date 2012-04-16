// Property_Deprecated.cpp
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
#include "Property_Deprecated.h"
#include <climits>



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
const char* Property_Deprecated::
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




