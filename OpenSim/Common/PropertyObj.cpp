// PropertyObj.cpp
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
#include "PropertyObj.h"




using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
PropertyObj::~PropertyObj()
{
	if(_value!=NULL) { delete _value;  _value=NULL; }
}
//_____________________________________________________________________________
/**
 * Constructor.
 */
PropertyObj::
PropertyObj(const string &aName,const Object &aValue) :
	Property_Deprecated(Property_Deprecated::Obj,aName)
{
	setName(aName);
	_value = aValue.clone();
	_value->setName(aName);
    setAllowableListSize(1,1);
}
//_____________________________________________________________________________
/**
 * Default Constructor.
 */
PropertyObj::
PropertyObj() :
	Property_Deprecated(Property_Deprecated::Obj,"Object")
{
	_value = 0;
    setAllowableListSize(1,1);
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aProperty Property_Deprecated to be copied.
 */
PropertyObj::PropertyObj(const PropertyObj &aProperty) :
	Property_Deprecated(aProperty)
{
	_value = aProperty.getValueObj().clone();
}
//_____________________________________________________________________________
/**
 * Construct and return a copy of this property.
 * The property is allocated using the new operator, so the caller is
 * responsible for deleting the returned object.
 *
 * @return Copy of this property.
 */
PropertyObj* PropertyObj::clone() const
{
	PropertyObj *property = new PropertyObj(*this);
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
PropertyObj& PropertyObj::
operator=(const PropertyObj &aProperty)
{
	Property_Deprecated::operator =(aProperty);
	if(_value!=NULL) { delete _value;  _value=NULL; }
	_value = aProperty.getValueObj().clone();
	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================

//-----------------------------------------------------------------------------
// VALUE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get a reference to the value of this property.  Note that the returned
 * reference can be used to change the value of this property.
 *
 * @return Reference to the value of this property.
 */
Object& PropertyObj::
getValueObj()
{
	return((*_value));
}
//_____________________________________________________________________________
/**
 * Get a constant reference to the value of this property.
 *
 * @return Contant reference to the value of this property.
 */
const Object& PropertyObj::
getValueObj() const
{
	return((*_value));
}
//_____________________________________________________________________________
/**
 * Get a constant String represeting the value of this property.
 *
 * @return Constant String represeting the value of this property.
 */
string PropertyObj::
toString() const
{
	return "(Object)";
}
