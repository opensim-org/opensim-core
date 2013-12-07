#ifndef __PathPoint_h__
#define __PathPoint_h__
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  PathPoint.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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
#include <OpenSim/Common/PropertyDblVec.h>
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
class OSIMSIMULATION_API PathPoint : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(PathPoint, Object);

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
	VisibleObject _displayer;

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

#ifndef SWIG
	PathPoint& operator=(const PathPoint &aPoint);
#endif
   void copyData(const PathPoint &aPoint);
	virtual void init(const PathPoint& aPoint);

#ifndef SWIG
	const SimTK::Vec3& getLocation() const { return _location; }
#endif
	SimTK::Vec3& getLocation()  { return _location; }

	const double& getLocationCoord(int aXYZ) const { assert(aXYZ>=0 && aXYZ<=2); return _location[aXYZ]; }
	void setLocationCoord(int aXYZ, double aValue) { assert(aXYZ>=0 && aXYZ<=2); _location[aXYZ]=aValue; }
	// A variant that uses basic types for use by GUI

	void setLocation( const SimTK::State& s, const SimTK::Vec3& aLocation);
	void setLocation( const SimTK::State& s, int aCoordIndex, double aLocation);
	void setLocation( const SimTK::State& s, double pt[]){ // A variant that uses basic types for use by GUI
		setLocation(s,SimTK::Vec3::updAs(pt));
	}
	void setBody(OpenSim::Body& aBody);
	void changeBodyPreserveLocation(const SimTK::State& s, OpenSim::Body& aBody);

	OpenSim::Body& getBody() const { return *_body; }
	const std::string& getBodyName() const { return _bodyName; }
	GeometryPath* getPath() const { return _path; }

    virtual void scale(const SimTK::State& s, const SimTK::Vec3& aScaleFactors);
	virtual WrapObject* getWrapObject() const { return NULL; }

	virtual bool isActive(const SimTK::State& s) const { return true; }
	virtual void connectToModelAndPath(const Model& aModel, GeometryPath& aPath);
	virtual void update(const SimTK::State& s) { }

	// get the relative velocity of the path point with resepct to the body
	// it is connected to.
	virtual void getVelocity(const SimTK::State& s, SimTK::Vec3& aVelocity);
	// get the partial of the point location w.r.t. to the coordinates (Q)
	// it is dependent on.
	virtual SimTK::Vec3 getdPointdQ(const SimTK::State& s) const
	    { return SimTK::Vec3(0); }

	// Visible Object Support
	virtual const VisibleObject* getDisplayer() const { return &_displayer; }
	virtual VisibleObject*	updDisplayer() { return &_displayer; };
	virtual void updateGeometry();

	// Utility
	static PathPoint* makePathPointOfType(PathPoint* aPoint, const std::string& aNewTypeName);
	static void deletePathPoint(PathPoint* aPoint) { if (aPoint) delete aPoint; }

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
