#ifndef _VisibleObject_h_
#define _VisibleObject_h_
// VisibleObject.h
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


// INCLUDES
#include "osimCommonDLL.h"
#include "Object.h"
#include "Transform.h"
#include "VisibleProperties.h"
#include "PropertyStrArray.h"
#include "PropertyObj.h"
#include "PropertyDblArray.h"
#include "PropertyDblVec3.h"
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
	PropertyDblVec3	_propScaleFactors;
	SimTK::Vec3&		_scaleFactors;

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

	void setScaleFactors(const SimTK::Vec3& aScaleFactors);
	void getScaleFactors(SimTK::Vec3& aScaleFactors) const;
	// A variation that uses raw arrays for use from GUI only.
	void setScaleFactors(const double aScaleFactors[]){
		setScaleFactors(SimTK::Vec3::getAs(aScaleFactors));
	}
	void getScaleFactors(double aScaleFactors[]) const {
		getScaleFactors(SimTK::Vec3::updAs(aScaleFactors));
	}
	// More interfaces for transform as other interfaces (Angles based) are needed
	void rotateRadians(const double rR[3]);
	void rotateRadians(const double rR[3], const Transform::RotationOrder order);
	void rotateRadiansX(const double rR);
	void rotateRadiansY(const double rR);
	void rotateRadiansZ(const double rR);
	void rotateRadiansAxis(const double rR, const SimTK::Vec3& axis);
	void rotateDegrees(const double rR[3]);
	void rotateDegrees(const double rR[3], const Transform::RotationOrder order);
	void rotateDegreesX(const double rR);
	void rotateDegreesY(const double rR);
	void rotateDegreesZ(const double rR);
	void rotateDegreesAxis(const double rR, const SimTK::Vec3& axis);
	void translate(const SimTK::Vec3& t);
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
		_allGeometry.setSize(0);
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
