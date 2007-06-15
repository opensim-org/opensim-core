#ifndef _VisibleObject_h_
#define _VisibleObject_h_
// VisibleObject.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/*  
 * Author:  
 */


// INCLUDES
#include "osimCommonDLL.h"
#include "Object.h"
#include "Transform.h"
#include "VisibleProperties.h"
#include "PropertyStrArray.h"
#include "PropertyObj.h"
#include "PropertyDblArray.h"
#include "Geometry.h"

namespace OpenSim { 

class AbstractDynamicsEngine;

class Geometry;
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

class OSIMCOMMON_API VisibleObject: public Object
{

//=============================================================================
// DATA
//=============================================================================
public:
private:
	// PROPERTIES
	/** Name of geometry file name(s) */
	PropertyStrArray	_propGeometryFileNames;
	Array<std::string>&	_geometryFileNames;	

	Array<Geometry*>   _allGeometry;
	/** Object that represents shading, material, colors */
	PropertyObj		_propVisibleProp;
	VisibleProperties&	_visibleProp;

	/** Scale factors for geometry. unserialized */
	PropertyDblArray	_propScaleFactors;
	Array<double>&		_scaleFactors;

	// In general these can use the Observable mechanism but that would be slow 
	// for display purposes.
	ArrayPtrs<VisibleObject>	_dependents;

protected:
	Object*				_owner;	// Actual object that owns this VisibleObject
	Transform			_transform; // transform relative to _owner's frame. unserialized
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
	virtual Object* copy() const;
	virtual void setup(AbstractDynamicsEngine *aEngine) { }

private:
	void setNull();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	VisibleObject& operator=(const VisibleObject &aObject);
	virtual bool operator==(const VisibleObject &aObject);
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
	// VISIBLE PROPERTIES
	void setVisibleProperties(const VisibleProperties &aVisibleProperties);
	VisibleProperties& getVisibleProperties();
	// TRANSFORM
	void setTransform(const Transform &aTransform);
	virtual Transform& getTransform();

	void setScaleFactors(const double aScaleFactors[3]);
	void getScaleFactors(double aScaleFactors[3]) const;

	// More interfaces for transform as other interfaces (Angles based) are needed
	void rotateRadians(const double rR[3]);
	void rotateRadians(const double rR[3], const Transform::RotationOrder order);
	void rotateRadiansX(const double rR);
	void rotateRadiansY(const double rR);
	void rotateRadiansZ(const double rR);
	void rotateRadiansAxis(const double rR, const double axis[3]);
	void rotateDegrees(const double rR[3]);
	void rotateDegrees(const double rR[3], const Transform::RotationOrder order);
	void rotateDegreesX(const double rR);
	void rotateDegreesY(const double rR);
	void rotateDegreesZ(const double rR);
	void rotateDegreesAxis(const double rR, const double axis[3]);
	void translate(const double t[3]);
	// DEPENDENTS
	void addDependent(VisibleObject *aChild){ _dependents.append(aChild); };
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
	}
	Geometry* getGeometry(int i)
	{
		return _allGeometry.get(i);
	}
	int countGeometry()
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
