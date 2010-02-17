#ifndef __PathPoint_h__
#define __PathPoint_h__

// PathPoint.h
// Author: Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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


// INCLUDE
#include <assert.h>
#include <iostream>
#include <string>
#include <math.h>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/Geometry.h>
#include <OpenSim/Common/VisibleObject.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyDblVec3.h>
#include <OpenSim/Common/PropertyStr.h>

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

namespace OpenSim {

class Body;
class Model;
class GeometryPath;
class SimbodyEngine;
class WrapObject;

//=============================================================================
//=============================================================================
/**
 * A class implementing a path point.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API PathPoint : public Object  
{

//=============================================================================
// DATA
//=============================================================================
private:

protected:

   const Model* _model;

   PropertyDblVec3 _locationProp;
   SimTK::Vec3 &_location;

	PropertyStr _bodyNameProp;
   std::string &_bodyName;

	// Support for Display
	PropertyObj _displayerProp;
	VisibleObject &_displayer;

	/* const*/ OpenSim::Body *_body; // Not const anymore since the body's displayer is not const

	GeometryPath* _path; // the path that owns this location point

	/** A temporary kluge until the default mechanism is working */
	static Geometry *_defaultGeometry;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	PathPoint();
	PathPoint(const PathPoint &aPoint);
	virtual ~PathPoint();
	virtual Object* copy() const;

#ifndef SWIG
	PathPoint& operator=(const PathPoint &aPoint);
#endif
   void copyData(const PathPoint &aPoint);
	virtual void init(const PathPoint& aPoint);

	const SimTK::Vec3& getLocation() const { return _location; }
	SimTK::Vec3& getLocation()  { return _location; }

	const double& getLocationCoord(int aXYZ) const { assert(aXYZ>=0 && aXYZ<=2); return _location[aXYZ]; }
	void setLocationCoord(int aXYZ, double aValue) { assert(aXYZ>=0 && aXYZ<=2); _location[aXYZ]=aValue; }
	// A variant that uses basic types for use by GUI
#ifndef SWIG
	void setLocation( const SimTK::State& s, const SimTK::Vec3& aLocation);
	void setLocation( const SimTK::State& s, int aCoordIndex, double aLocation);
	void setLocation( const SimTK::State& s, double pt[]){ // A variant that uses basic types for use by GUI
		setLocation(s,SimTK::Vec3::updAs(pt));
	}
	void setBody(OpenSim::Body& aBody);
	void changeBodyPreserveLocation(const SimTK::State& s, OpenSim::Body& aBody);
#endif
	OpenSim::Body& getBody() const { return *_body; }
	const std::string& getBodyName() const { return _bodyName; }
	GeometryPath* getPath() const { return _path; }

   virtual void scale(const SimTK::State& s, const SimTK::Vec3& aScaleFactors);
	virtual WrapObject* getWrapObject() const { return NULL; }
#ifndef SWIG
	virtual bool isActive(const SimTK::State& s) const { return true; }
	virtual void setup(const Model& aModel, GeometryPath& aPath);
	virtual void update(const SimTK::State& s) { }
	virtual void getVelocity(const SimTK::State& s, SimTK::Vec3& aVelocity);
#endif
	// Visible Object Support
	virtual const VisibleObject* getDisplayer() const { return &_displayer; }
	virtual VisibleObject*	updDisplayer() { return &_displayer; };
	virtual void updateGeometry();

	// Utility
	static PathPoint* makePathPointOfType(PathPoint* aPoint, const std::string& aNewTypeName);
	static void deletePathPoint(PathPoint* aPoint) { if (aPoint) delete aPoint; }

	OPENSIM_DECLARE_DERIVED(PathPoint, Object);
protected:

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class PathPoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __PathPoint_h__
