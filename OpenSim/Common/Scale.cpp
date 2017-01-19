/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Scale.cpp                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include "Scale.h"



using namespace OpenSim;
using namespace std;
using SimTK::Vec3;

//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
Scale::~Scale(void)
{
}
//_____________________________________________________________________________
/**
 * Default constructor of an Scale
 */
Scale::Scale():
_scaleFactors(_propScaleFactors.getValueDblVec()),
_segmentName(_propSegmentName.getValueStr()),
_apply(_propApply.getValueBool())
{
    setNull();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param an Scale to copy
 */
Scale::Scale(const Scale &aScale) :
Object(aScale),
_scaleFactors(_propScaleFactors.getValueDblVec()),
_segmentName(_propSegmentName.getValueStr()),
_apply(_propApply.getValueBool())
{
    setNull();

    // ASSIGN
    *this = aScale;
}
//_____________________________________________________________________________
/**
 * Constructor of a scaleSet from a file.
 */
Scale::Scale(const string& scaleFileName):
Object(scaleFileName, false),
_scaleFactors(_propScaleFactors.getValueDblVec()),
_segmentName(_propSegmentName.getValueStr()),
_apply(_propApply.getValueBool())
{
    setNull();
    updateFromXMLDocument();
}

//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assign this object to the values of another.
 *
 * @return Reference to this object.
 */
Scale& Scale::
operator=(const Scale &aScale)
{
    // BASE CLASS
    _segmentName = aScale.getSegmentName();
    aScale.getScaleFactors(_scaleFactors);
    _apply = aScale.getApply();

    return(*this);
}


void Scale::setNull()
{
    setName("");
    setupProperties();
}
//_____________________________________________________________________________
/**
 * Set up the serialized member variables.  
 */
void Scale::
setupProperties()
{
    Vec3 one3(1.0); 

    // scale factors
    _propScaleFactors.setName("scales");
    _propScaleFactors.setValue(one3);
    //_propScaleFactors.setAllowableListSize(3);
    _propertySet.append( &_propScaleFactors );

    // segment name
    _propSegmentName.setName("segment");
    _propSegmentName.setValue("unnamed_segment");
    _propertySet.append( &_propSegmentName );

    // whether or not to apply the scale
    _propApply.setName("apply");
    _propApply.setValue(true);
    _propertySet.append(&_propApply);
}
//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get segment name
 */
const std::string& Scale::
getSegmentName() const
{
    return _segmentName;
}
//_____________________________________________________________________________
/**
 * Set the value of scale factors
 */
void Scale::
getScaleFactors(SimTK::Vec3& aScaleFactors) const
{
    aScaleFactors = _scaleFactors;
}

//_____________________________________________________________________________
/**
 * Set segment name
 */
void Scale::
setSegmentName(const string& aSegmentName)
{
    _segmentName = aSegmentName;
}
//_____________________________________________________________________________
/**
 * Set scale factors
 */
void Scale::
setScaleFactors(const SimTK::Vec3& aScaleFactors)
{
    _scaleFactors = aScaleFactors;
}
