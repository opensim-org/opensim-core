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
#include "PropertyStrArray.h"
#include "PropertyObj.h"
#include "PropertyDblArray.h"
#include "PropertyDblVec3.h"
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

class OSIMCOMMON_API VisibleObject: public Object
{

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
	virtual Object* copy() const;

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

	const GeometrySet& getGeometrySet() const { 
		return (_geometrySet); 
	};
	// TRANSFORM
	const SimTK::Transform& getTransform() {
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

	virtual void updateFromXMLNode();

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
