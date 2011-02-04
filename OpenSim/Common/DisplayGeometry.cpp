// DisplayGeometry.cpp
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

	setType("DisplayGeometry");
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

Object* DisplayGeometry::copy() const
{
	DisplayGeometry *geometry = new DisplayGeometry(*this);
	return(geometry);

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
