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
#include "rdToolsDLL.h"
#include "Object.h"
#include "Transform.h"
#include "VisibleProperties.h"
#include "PropertyStrArray.h"
#include "PropertyObj.h"
#include "PropertyDblArray.h"
#include "Geometry.h"

namespace OpenSim { 

class XMLNode;
class XMLDocument;

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

class RDTOOLS_API VisibleObject: public Object
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
	Array<double>		_scaleFactors;

	// In general these can use the Observable mechanism but that would be slow for display purposes.
	ArrayPtrs<VisibleObject>	_dependents;

protected:
	Object*				_owner;	// Actual object that owns this VisibleObject, for debugging only as of now
	Transform			_transform; /** transform relative to _owner's frame. unserialized */
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	VisibleObject();
	VisibleObject(const std::string &aFileName);
	VisibleObject(DOMElement *aNode);
	VisibleObject(const VisibleObject &aVisibleObject);
	virtual ~VisibleObject();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;
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

	void setOwner(Object *aObject)
	{
		_owner = aObject;
	}

	Object* getOwner()
	{
		return _owner;
	}
	void setNumGeometryFiles(int n);
	void setGeometryFileName(int i, const std::string &aGeometryFileName);
	const int getNumGeometryFiles() const;
	const std::string& getGeometryFileName(int i) const;

	void setVisibleProperties(const VisibleProperties &aVisibleProperties);
	VisibleProperties& getVisibleProperties();

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
	
	void addDependent(VisibleObject *aChild){ _dependents.append(aChild); };
	void removeDependent(VisibleObject *aChild){ _dependents.remove(aChild); };
	int countDependents(){ return _dependents.getSize(); };
	VisibleObject *getDependent(int i) { 
		return _dependents.get(i); 
	};
	void addGeometry(Geometry* aGeometry)
	{
		_allGeometry.append(aGeometry);
	};
	const Geometry* getGeometry(int i)
	{
		return _allGeometry.get(i);
	}
	int countGeometry()
	{
		return _allGeometry.getSize();
	};
	const Geometry* getDefaultGeometry()
	{
		return AnalyticGeometry::createSphere(0.1);
	}
	//--------------------------------------------------------------------------
	// XML
	//--------------------------------------------------------------------------
	void setupProperties();
	
private:
	void generateDocument();


//=============================================================================
};	// END of class VisibleObject

}; //namespace
//=============================================================================
//=============================================================================

#endif //__VisibleObject_h__
