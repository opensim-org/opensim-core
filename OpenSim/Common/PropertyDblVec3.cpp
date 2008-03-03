// PropertyDblVec3.cpp
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

/*  Author: Ayman Habib 
 */


//============================================================================
// INCLUDES
//============================================================================
#include "PropertyDblVec3.h"




using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Constructor.
 */
PropertyDblVec3::
PropertyDblVec3(const string &aName,
	const SimTK::Vec3& aVec3) :
	Property(Property::DblVec3,aName),
	_vec(aVec3)
{
}
//_____________________________________________________________________________
/**
 * Constructor.
 */
PropertyDblVec3::
PropertyDblVec3(const string &aName,
	const Array<double> &aArray) :
	Property(Property::DblVec3,aName)
{
	assert(aArray.getSize()==3);
	for(int i=0; i<3; i++)
		_vec[i] = aArray[i];
}
//_____________________________________________________________________________
/**
 * Default Constructor.
 */
PropertyDblVec3::
PropertyDblVec3() :
	Property(Property::DblVec3,"DblVec3PropertyName")
{

}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aProperty Property to be copied.
 */
PropertyDblVec3::PropertyDblVec3(const PropertyDblVec3 &aProperty) :
	Property(aProperty)
{
	_vec = aProperty._vec;
}
//_____________________________________________________________________________
/**
 * Construct and return a copy of this property.
 * The property is allocated using the new operator, so the caller is
 * responsible for deleting the returned object.
 *
 * @return Copy of this property.
 */
Property* PropertyDblVec3::copy() const
{
	Property *property = new PropertyDblVec3(*this);
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
PropertyDblVec3& PropertyDblVec3::
operator=(const PropertyDblVec3 &aProperty)
{
	Property::operator =(aProperty);
	_vec = aProperty._vec;
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
const char* PropertyDblVec3::
getTypeAsString() const
{
	return("DblVec3");
}

//-----------------------------------------------------------------------------
// VALUE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the value of this property.
 *
 * @param aArray Array to which this property is to be assigned.
 *
void PropertyDblVec3::
setValue(const Array<double>& aArray)
{
	for(int i=0;i<3; i++)
		_vec[i] = aArray[i];
}
//_____________________________________________________________________________
/**
 * Set the value of this property.
 *
 * @param aArray Array to which this property is to be assigned.
 */
void PropertyDblVec3::
setValue(const SimTK::Vec3 &aVec3)
{
	_vec = aVec3;
}
//_____________________________________________________________________________
/**
 * Get a reference to the value of this property.  Note that the returned
 * reference can be used to change the value of this property.
 *
 * @return Reference to the value of this property.
 */
SimTK::Vec3& PropertyDblVec3::
getValueDblVec3()
{
	return(_vec);
}
//_____________________________________________________________________________
/**
 * Get a constant reference to the value of this property.
 *
 * @return Reference to the value of this property.
 */
const SimTK::Vec3& PropertyDblVec3::
getValueDblVec3() const
{
	return(_vec);
}
//_____________________________________________________________________________
/**
 * Get a constant String represeting the value of this property.
 *
 * @return Constant String represeting the value of this property.
 */
const string &PropertyDblVec3::
toString()
{
	string str = "(";
	char dbl[256];
	for(int i=0; i < _vec.size(); i++){
		sprintf(dbl, "%g", _vec[i]);
		str += (i>0?" ":"") + string(dbl);
	}
	str += ")";
	_valueString = str;
	return (_valueString);
}
