// PropertyTransform.cpp
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
#include "PropertyTransform.h"




using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Constructor.
 */
PropertyTransform::
PropertyTransform(const string &aName,
	const SimTK::Transform& aTransform) :
PropertyDblArray(aName, OpenSim::Array<double>(0., 6)),
	_transform(aTransform)
{
	setType(Transform);
	getRotationsAndTranslationsAsArray6(&_array[0]);
}
//_____________________________________________________________________________
/**
 * Constructor.
 */
PropertyTransform::
PropertyTransform(const string &aName,
	const Array<double> &aArray) :
PropertyDblArray(aName, aArray)
{
	setType(Transform);
	assert(aArray.getSize()==6);
	_transform.updR().setRotationToBodyFixedXYZ(SimTK::Vec3::getAs(&aArray[0]));
	_transform.updP() = SimTK::Vec3::getAs(&aArray[3]);
}
//_____________________________________________________________________________
/**
 * Default Constructor.
 */
PropertyTransform::
PropertyTransform() :
	PropertyDblArray("TransformPropertyName", OpenSim::Array<double>(0., 6))
{
	setType(Transform);
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aProperty Property_Deprecated to be copied.
 */
PropertyTransform::PropertyTransform(const PropertyTransform &aProperty) :
	PropertyDblArray(aProperty)
{
	_transform = aProperty._transform;
}
//_____________________________________________________________________________
/**
 * Construct and return a copy of this property.
 * The property is allocated using the new operator, so the caller is
 * responsible for deleting the returned object.
 *
 * @return Copy of this property.
 */
PropertyTransform* PropertyTransform::copy() const
{
	PropertyTransform *property = new PropertyTransform(*this);
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
PropertyTransform& PropertyTransform::
operator=(const PropertyTransform &aProperty)
{
	PropertyDblArray::operator =(aProperty);
	_transform = aProperty._transform;
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
const char* PropertyTransform::
getTypeAsString() const
{
	return("Transform");
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
void PropertyTransform::
setValue(const Array<double>& aArray)
{
	for(int i=0;i<3; i++)
		_transform[i] = aArray[i];
}
//_____________________________________________________________________________
/**
 * Set the value of this property.
 *
 * @param aArray Array to which this property is to be assigned.
 */
void PropertyTransform::
setValue(const SimTK::Transform &aTransform)
{
	_transform = aTransform;
	getRotationsAndTranslationsAsArray6(&_array[0]);
}
void PropertyTransform::
setValue(int aSize,const double aArray[])
{
	assert(aSize==6);
	PropertyDblArray::setValue(aSize, aArray);
	_transform.updR().setRotationToBodyFixedXYZ(SimTK::Vec3::getAs(&aArray[0]));
	_transform.updP() = SimTK::Vec3::getAs(&aArray[3]);
}
//_____________________________________________________________________________
/**
 * Get a reference to the value of this property.  Note that the returned
 * reference can be used to change the value of this property.
 *
 * @return Reference to the value of this property.
 */
SimTK::Transform& PropertyTransform::
getValueTransform()
{
	return(_transform);
}
//_____________________________________________________________________________
/**
 * Get a constant reference to the value of this property.
 *
 * @return Reference to the value of this property.
 */
const SimTK::Transform& PropertyTransform::
getValueTransform() const
{
	return(_transform);
}
void PropertyTransform::getRotationsAndTranslationsAsArray6(double aArray[]) const
{
	SimTK::Vec3 translations = _transform.p();
	SimTK::Vec3 rotations = _transform.R().convertRotationToBodyFixedXYZ();
	int i=0;
	for(i=0; i<3; i++){
		aArray[i] = rotations[i];
		aArray[i+3] = translations[i];
	}
}
//_____________________________________________________________________________
/**
 * Get a constant String represeting the value of this property.
 *
 * @return Constant String represeting the value of this property.
 */
string PropertyTransform::
toString() const
{
	string str = "(";
	char pad[256];
	double rawData[6];
	getRotationsAndTranslationsAsArray6(rawData);
	sprintf(pad, "%g %g %g %g %g %g", rawData[0], rawData[1], rawData[2], rawData[3], rawData[4], rawData[5]);
	str += string(pad);
	str += ")";
	return str;
}
