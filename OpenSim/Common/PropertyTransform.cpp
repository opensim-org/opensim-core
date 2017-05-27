/* -------------------------------------------------------------------------- *
 *                      OpenSim:  PropertyTransform.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
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
    setAllowableListSize(6);
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
    setAllowableListSize(6);
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
    setAllowableListSize(6);
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
PropertyTransform* PropertyTransform::clone() const
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
std::string PropertyTransform::
getTypeName() const
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
*/
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
 * Get a constant String representing the value of this property.
 *
 * @return Constant String representing the value of this property.
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
