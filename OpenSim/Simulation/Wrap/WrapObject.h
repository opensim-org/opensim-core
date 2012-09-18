#ifndef __WrapObject_h__
#define __WrapObject_h__
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  WrapObject.h                           *
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
#include <iostream>
#include <string>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/VisibleObject.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyStr.h>
#include "SimTKsimbody.h"

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

namespace OpenSim {

class VisibleObject;
class Body;
class PathPoint;
class PathWrap;
class WrapResult;
class Model;

//=============================================================================
//=============================================================================
/**
 * An abstract class that specifies the interface for a wrapping
 * object.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API WrapObject : public Object {
OpenSim_DECLARE_ABSTRACT_OBJECT(WrapObject, Object);

//=============================================================================
// DATA
//=============================================================================
public:
	enum WrapQuadrant
	{
		allQuadrants,
		negativeX,
		positiveX,
		negativeY,
		positiveY,
		negativeZ,
		positiveZ
	};

	enum WrapAction
	{
		noWrap,          // the path segment did not intersect the wrap object
		insideRadius,    // one or both path points are inside the wrap object
		wrapped,         // successful wrap, but may not be 'best' path
		mandatoryWrap    // successful wrap that must be used (e.g., both tangent
	};                  // points are on the constrained side of the wrap object)

protected:

	PropertyDblArray _xyzBodyRotationProp;
	Array<double>& _xyzBodyRotation;

	PropertyDblVec3 _translationProp;
	SimTK::Vec3 & _translation;

	PropertyBool _activeProp;
	bool& _active;

	PropertyStr _quadrantNameProp;
	std::string& _quadrantName;

	// Support for Display
	PropertyObj _displayerProp;
	VisibleObject &_displayer;

	WrapQuadrant _quadrant;
	int _wrapAxis;
	int _wrapSign;

	Body* _body;

	SimTK::Transform _pose;
	const Model* _model;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
    /** Display Preference to apply to the contact geometry.  **/
    OpenSim_DECLARE_PROPERTY(display_preference, int,
        "Display Pref. 0:Hide 1:Wire 3:Flat 4:Shaded");

    /** Display Color to apply to the contact geometry.  **/
    OpenSim_DECLARE_LIST_PROPERTY_SIZE(color, double, 3,
        "Display Color");

    WrapObject();
	WrapObject(const WrapObject& aWrapObject);
	virtual ~WrapObject();

#ifndef SWIG
	WrapObject& operator=(const WrapObject& aWrapObject);
#endif
   void copyData(const WrapObject& aWrapObject);

	virtual void scale(const SimTK::Vec3& aScaleFactors);
	virtual void connectToModelAndBody(Model& aModel, Body& aBody);

	Body& getBody() const { return *_body; }
	const double* getXYZBodyRotation() const { return &_xyzBodyRotation[0]; }
	const double* getTranslation() const { return &_translation[0]; }
	bool getActive() const { return _active; }
	bool getActiveUseDefault() const { return _activeProp.getValueIsDefault(); }
	const char* getQuadrantName() const { return _quadrantName.c_str(); }
	bool getQuadrantNameUseDefault() const { return _quadrantNameProp.getValueIsDefault(); }
	void setQuadrantName(const std::string& aName);
        const SimTK::Transform& getTransform() const { return _pose; }
	virtual const char* getWrapTypeName() const = 0;
	virtual std::string getDimensionsString() const { return ""; } // TODO: total SIMM hack!
#ifndef SWIG
	int wrapPathSegment( const SimTK::State& s, PathPoint& aPoint1, PathPoint& aPoint2,
		const PathWrap& aPathWrap, WrapResult& aWrapResult) const;
	virtual int wrapLine(const SimTK::State& s, SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2,
		const PathWrap& aPathWrap, WrapResult& aWrapResult, bool& aFlag) const = 0;
#endif
	// Visible Object Support
	virtual VisibleObject* getDisplayer() const { return &_displayer; };
	virtual void updateGeometry() {};

protected:
	void setupProperties();
	void setupQuadrant();
	void setGeometryQuadrants(AnalyticGeometry *aGeometry) const;
private:
	void setNull();
    void constructProperties();
//=============================================================================
};	// END of class WrapObject
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __WrapObject_h__


