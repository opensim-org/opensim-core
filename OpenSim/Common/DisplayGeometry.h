#ifndef _DisplayGeometry_h_
#define _DisplayGeometry_h_
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  DisplayGeometry.h                         *
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
 * Author:  
 */

#include "osimCommonDLL.h"
#include "PropertyDbl.h"
#include "PropertyStr.h"
#include "PropertyDblArray.h"
#include "PropertyDblVec.h"
#include "PropertyTransform.h"
#include "PropertyInt.h"
#include "Object.h"
#include "Set.h"

namespace OpenSim { 

//=============================================================================
//=============================================================================
/**
 * A class for representing the DisplayGeometry properties of an object.
 *
 * @author Ayman Habib
 * @version 1.0
 */
class OSIMCOMMON_API DisplayGeometry: public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(DisplayGeometry, Object);

//=============================================================================
// DATA
//=============================================================================
public:
	enum  DisplayPreference {
			None=0,
			WireFrame=1,
			SolidFill=2,
			FlatShaded=3,
			GouraudShaded=4
	};
protected:
	/** Name of geomtry file .vtp, .stl, .obj are supported */
	PropertyStr		_propGeometryFile;
	std::string&	_geometryFile;

	/** A list of 3 colors */
	PropertyDblVec3	_propColor;
	SimTK::Vec3&	_color;

	/** Opacity */
	PropertyDbl	_propOpacity;
	double&	_opacity;

	/** Texture file */
	PropertyStr		_propTextureFile;
	std::string&	_textureFile;

	/** Transform relative to owner's frame */
	PropertyTransform	_propTransform;
	SimTK::Transform&	_transform;

	/** A list of 3 scale factors in X, Y, Z directions */
	PropertyDblVec3	_propScaleFactors;
	SimTK::Vec3&	_scaleFactors;

	/** DisplayPreference 0 up to 4 */
	PropertyInt	_propDisplayPreference;
	DisplayPreference& _displayPreference;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	DisplayGeometry();
	DisplayGeometry(const std::string &aFileName);
	DisplayGeometry(const DisplayGeometry &aDisplayGeometry);
	virtual ~DisplayGeometry() {}

private:
	void setNull();
	void copyData(const DisplayGeometry &aDisplayGeometry);

protected:
	// Serialization support
	virtual void setupProperties();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	DisplayGeometry& operator=(const DisplayGeometry &aObject);
	bool operator==(const DisplayGeometry &aObject) const { return (_geometryFile== aObject._geometryFile); };
#endif
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	const std::string&	getGeometryFile() const { return _geometryFile; };
	void setGeometryFile(const std::string&	aGeometryFile) { _geometryFile=aGeometryFile; };

	SimTK::Vec3&	getColor() const { return _color; };
	void getColor(double aColor[3]) 
	{
		for(int i=0; i< 3; i++) aColor[i] = _color[i];
	}
	void setColor(const SimTK::Vec3&	aColor) { _color=aColor; };
	void setColor(double aColor[3])
	{
		for(int i=0; i< 3; i++) _color[i] = aColor[i];
	}

	const std::string&	getTextureFile() const { return _textureFile; };
	void setTextureFile(const std::string&	aTextureFile) { _textureFile=aTextureFile; };

	SimTK::Transform&	getTransform() const { return _transform; };
	void setTransform(const SimTK::Transform&	aTransform) { _transform=aTransform; };
	void setRotationsAndTRanslations(double aArray[]);
	void getRotationsAndTranslationsAsArray6(double aArray[]) const
	{
			SimTK::Vec3 translations = _transform.p();
			SimTK::Vec3 rotations = _transform.R().convertRotationToBodyFixedXYZ();
			int i=0;
			for(i=0; i<3; i++){
				aArray[i] = rotations[i];
				aArray[i+3] = translations[i];
			}
	}
	SimTK::Vec3&	getScaleFactors() const { return _scaleFactors; };
	void getScaleFactors(double aScaleFactors[]) const {
		SimTK::Vec3::updAs(aScaleFactors) = getScaleFactors();
	} 
	void setScaleFactors(const double aScaleFactors[]){
		setScaleFactors(SimTK::Vec3::getAs(aScaleFactors));
	}
	void setScaleFactors(const SimTK::Vec3&	aScaleFactors) { _scaleFactors=aScaleFactors; };

	DisplayPreference getDisplayPreference() const { return _displayPreference; };
	void setDisplayPreference(const DisplayPreference& aPreference) { _displayPreference=aPreference; };

	double getOpacity() const { return _opacity; };
	void setOpacity(const double& aOpacity) { _opacity=aOpacity; };

//=============================================================================
};	// END of class DisplayGeometry


}; //namespace
//=============================================================================
//=============================================================================

#endif //__DisplayGeometry_h__
