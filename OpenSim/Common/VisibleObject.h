#ifndef _VisibleObject_h_
#define _VisibleObject_h_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  VisibleObject.h                          *
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


// INCLUDES
#include "osimCommonDLL.h"
#include "Object.h"
#include "PropertyObj.h"
#include "PropertyDblArray.h"
#include "PropertyDblVec.h"
#include "PropertyTransform.h"
#include "PropertyBool.h"
#include "Geometry.h"
#include "DisplayGeometry.h"
#include "SimTKcommon.h"
#include "GeometrySet.h"

#ifndef SWIG
using SimTK::Transform;
#endif

namespace OpenSim { 

class Geometry;
class GeometrySet;
// CONSTANTS

//=============================================================================
//=============================================================================
/**
 * Class VisibleObject is intended to be used as the base class for all
 * Visible objects that subclass Object.  It provides a common object from which
 * to derive and also some basic functionality, such as maintaining geometry
 *
 * @version 1.0
 * @author Ayman Habib
 */

class OSIMCOMMON_API VisibleObject: public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(VisibleObject, Object);

//=============================================================================
// DATA
//=============================================================================
private:
	// PROPERTIES
	/** Name of geometry(s) */
	PropertyObj			_propGeometrySet;
	OpenSim::GeometrySet&	_geometrySet;	

	/** Scale factors for geometry. */
	PropertyDblVec3		_propScaleFactors;
	SimTK::Vec3&		_scaleFactors;

	/** Transform of Geometries relative to owner frame */
	PropertyTransform	_transformProp;
	Transform&			_transform; // transform relative to _owner's frame. 

	/** Whether to show frame at the origin of the geometry owner */
	PropertyBool		_propShowAxes;
	bool&				_showAxes;

	/** DisplayPreference 0 up to 4 if geometry contains pieces their preferences take precedence*/
	PropertyInt	_propDisplayPreference;
	DisplayGeometry::DisplayPreference& _displayPreference;

protected:
	Object*				_owner;	// Actual object that owns this VisibleObject

	Array<Geometry*>   _allGeometry;
	// In general these can use the Observable mechanism but that would be slow 
	// for display purposes.
	ArrayPtrs<VisibleObject>	_dependents;
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	VisibleObject();
	VisibleObject(const std::string &aFileName);
	VisibleObject(const VisibleObject &aVisibleObject);
	virtual ~VisibleObject();

private:
	void setNull();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	VisibleObject& operator=(const VisibleObject &aObject);
	virtual bool operator==(const VisibleObject &aObject) const;
#endif
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:
	// OWNER OBJECT
	void setOwner(Object *aObject)
	{
		_owner = aObject;
	}

	Object* getOwner()
	{
		return _owner;
	}
	// GEOMETRY FILES
	void setNumGeometryFiles(int n);
	void setGeometryFileName(int i, const std::string &aGeometryFileName);
	const int getNumGeometryFiles() const;
	const std::string& getGeometryFileName(int i) const;

	const GeometrySet& getGeometrySet() const { 
		return (_geometrySet); 
	};

    GeometrySet& updGeometrySet() {
        return (_geometrySet);
    };
	// TRANSFORM
	const SimTK::Transform& getTransform() const {
		return _transform;
	}
	SimTK::Transform& updTransform() {
		return _transform;
	}
	void getRotationsAndTranslationsAsArray6(double aArray[]) const;
	void getTransformAsDouble16(double flatList[]) {
		double* matStart = &_transform.toMat44()[0][0];
		for (int i=0; i<16; i++) flatList[i]=matStart[i];
	}

	void setTransform(const SimTK::Transform& aTransform) {
		_transform = aTransform;
	}
	void translate(const SimTK::Vec3& t) {
		_transform.updP() = _transform.p()+t;
	}

	// Scale
	void setScaleFactors(const SimTK::Vec3& aScaleFactors);
	void getScaleFactors(SimTK::Vec3& aScaleFactors) const;
	// A variation that uses raw arrays for use from GUI only.
	void setScaleFactors(const double aScaleFactors[]){
		setScaleFactors(SimTK::Vec3::getAs(aScaleFactors));
	}
	void getScaleFactors(double aScaleFactors[]) const {
		getScaleFactors(SimTK::Vec3::updAs(aScaleFactors));
	} 

	// Axes
	bool getShowAxes() const { return _showAxes; };
	void setShowAxes(const bool showAxes) { _showAxes = showAxes; };

	// DisplayPreference
	DisplayGeometry::DisplayPreference getDisplayPreference() const;
	void setDisplayPreference(const DisplayGeometry::DisplayPreference& aPreference);

	// DEPENDENTS
	void addDependent(VisibleObject *aChild){ 
		if (!hasDependent(aChild))
			_dependents.append(aChild);
	};
	bool hasDependent(VisibleObject *aChild){ // Check if dependency already added
		for(int i=0; i < _dependents.getSize(); i++){
			if (_dependents.get(i)==aChild)
				return true; 
		}
		return false;
	};
	void removeDependent(VisibleObject *aChild){ _dependents.remove(aChild); };
	int countDependents(){ return _dependents.getSize(); };
	VisibleObject *getDependent(int i) { 
		return _dependents.get(i); 
	};
	// GEOMETRY ITEMS
	void addGeometry(Geometry* aGeometry)
	{
		_allGeometry.append(aGeometry);
	};
	void removeGeometry(Geometry* aGeometry)
	{
		_allGeometry.remove(_allGeometry.findIndex(aGeometry));
	};
	// Release resources allocated by the display support code.
	void freeGeometry()
	{
		for (int i=_allGeometry.getSize()-1; i >= 0; i--)
			delete _allGeometry.get(i);
		_allGeometry.setSize(0);
	}
	Geometry* getGeometry(int i)
	{
		return _allGeometry.get(i);
	}
	int countGeometry() const
	{
		return _allGeometry.getSize();
	};
	const Geometry* getDefaultGeometry()
	{
		return AnalyticSphere::createSphere(0.1);
	}

	//--------------------------------------------------------------------------
	// XML
	//--------------------------------------------------------------------------
	void setupProperties();

	virtual void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1);

	// updateGeometry is the method used to update geometry that can change (e.g. muscles
	// changing geometry during motion).
	// Clients/subclasses should not override this method unless their geometry is changing.
	// This call forces the "recomputation" of geometry aside from display implications.
protected:
	virtual void updateGeometry() {};

//=============================================================================
};	// END of class VisibleObject

}; //namespace
//=============================================================================
//=============================================================================

#endif //__VisibleObject_h__
