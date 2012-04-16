#ifndef __WrapObject_h__
#define __WrapObject_h__

// WrapObject.h
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
	WrapObject();
	WrapObject(const WrapObject& aWrapObject);
	virtual ~WrapObject();

#ifndef SWIG
	WrapObject& operator=(const WrapObject& aWrapObject);
#endif
   void copyData(const WrapObject& aWrapObject);

	virtual void scale(const SimTK::Vec3& aScaleFactors);
	virtual void setup(Model& aModel, Body& aBody);

	Body& getBody() const { return *_body; }
	const double* getXYZBodyRotation() const { return &_xyzBodyRotation[0]; }
	const double* getTranslation() const { return &_translation[0]; }
	bool getActive() const { return _active; }
	bool getActiveUseDefault() const { return _activeProp.getUseDefault(); }
	const char* getQuadrantName() const { return _quadrantName.c_str(); }
	bool getQuadrantNameUseDefault() const { return _quadrantNameProp.getUseDefault(); }
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
//=============================================================================
};	// END of class WrapObject
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __WrapObject_h__


