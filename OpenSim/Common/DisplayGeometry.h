#ifndef _DisplayGeometry_h_
#define _DisplayGeometry_h_
// DisplayGeometry.h
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
