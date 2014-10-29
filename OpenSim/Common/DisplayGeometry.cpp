/* -------------------------------------------------------------------------- *
 *                       OpenSim:  DisplayGeometry.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

/*  
 * Author:  Ayman Habib
 */
#include "DisplayGeometry.h"

using namespace std;
using namespace OpenSim;

//=============================================================================
//=============================================================================
/**
 * A class for representing one DisplayGeometry of an object, e.g. one bone
 *
 * @author Ayman Habib
 * @version 1.0
 */
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
DisplayGeometry::DisplayGeometry() :
_geometryFile(_propGeometryFile.getValueStr()),
_color(_propColor.getValueDblVec()),
_textureFile(_propTextureFile.getValueStr()),
_propTransform(PropertyTransform("", SimTK::Transform())),
_transform(_propTransform.getValueTransform()),
_scaleFactors(_propScaleFactors.getValueDblVec()),
_displayPreference((DisplayPreference&)_propDisplayPreference.getValueInt()),
_opacity(_propOpacity.getValueDbl())
{
    setNull();
}

DisplayGeometry::DisplayGeometry(const std::string &aFileName) :
_geometryFile(_propGeometryFile.getValueStr()),
_color(_propColor.getValueDblVec()),
_textureFile(_propTextureFile.getValueStr()),
_propTransform(PropertyTransform("", SimTK::Transform())),
_transform(_propTransform.getValueTransform()),
_scaleFactors(_propScaleFactors.getValueDblVec()),
_displayPreference((DisplayPreference&)_propDisplayPreference.getValueInt()),
_opacity(_propOpacity.getValueDbl())
{
    setNull();

    _geometryFile=aFileName;
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aGeometryPiece DisplayGeometry to be copied.
 */
DisplayGeometry::DisplayGeometry(const DisplayGeometry &aGeometryPiece) :
Object(aGeometryPiece),
_geometryFile(_propGeometryFile.getValueStr()),
_color(_propColor.getValueDblVec()),
_textureFile(_propTextureFile.getValueStr()),
_propTransform(PropertyTransform("", SimTK::Transform())),
_transform(_propTransform.getValueTransform()),
_scaleFactors(_propScaleFactors.getValueDblVec()),
_displayPreference((DisplayPreference&)_propDisplayPreference.getValueInt()),
_opacity(_propOpacity.getValueDbl())
{
    setNull();
    //setupProperties();
    copyData(aGeometryPiece);

}
//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void DisplayGeometry::
setNull()
{
    setName("");

    setupProperties();

    // Assign default values
    _color = 1.0;
    _textureFile="";
    _transform=SimTK::Transform();
    _scaleFactors = 1.0;
    _displayPreference = GouraudShaded;
    _opacity = 1.0;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void DisplayGeometry::setupProperties()
{

    _propGeometryFile.setName("geometry_file");
    _propGeometryFile.setComment("Name of geometry file .vtp, .stl, .obj");
    _propertySet.append(&_propGeometryFile);

    _propColor.setName("color");
    _propColor.setComment("Color used to display the geometry when visible");
    _propertySet.append(&_propColor);

    _propTextureFile.setName("texture_file");
    _propTextureFile.setComment("Name of texture file .jpg, .bmp");
    _propertySet.append(&_propTextureFile);

    _propTransform.setName("transform");
    _propTransform.setComment("in body transform specified as 3 rotations (rad) followed by 3 translations rX rY rZ tx ty tz");
    _propertySet.append(&_propTransform);

    _propScaleFactors.setName("scale_factors");
    _propScaleFactors.setComment("Three scale factors for display purposes: scaleX scaleY scaleZ");
    _propertySet.append(&_propScaleFactors);

    _propDisplayPreference.setName("display_preference");
    _propDisplayPreference.setComment("Display Pref. 0:Hide 1:Wire 3:Flat 4:Shaded");
    _propertySet.append(&_propDisplayPreference);

    _propOpacity.setName("opacity");
    _propOpacity.setComment("Display opacity between 0.0 and 1.0");
    _propertySet.append(&_propOpacity);

}

void DisplayGeometry::copyData(const DisplayGeometry &aGeometryPiece)
{
    _geometryFile=aGeometryPiece._geometryFile;
    _color=aGeometryPiece._color;
    _textureFile=aGeometryPiece._textureFile;
    _transform=aGeometryPiece._transform;
    _scaleFactors=aGeometryPiece._scaleFactors;
    _displayPreference=aGeometryPiece._displayPreference;
    _opacity = aGeometryPiece._opacity;
}

void DisplayGeometry::setRotationsAndTRanslations(double aArray[])
{
    _propTransform.setValue(6, aArray);
}
//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
DisplayGeometry& DisplayGeometry::operator=(const DisplayGeometry &aDisplayGeometry)
{
    Object::operator=(aDisplayGeometry);
    copyData(aDisplayGeometry);
    return(*this);
}
