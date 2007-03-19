#ifndef __AbstractWrapObject_h__
#define __AbstractWrapObject_h__

// AbstractWrapObject.h
// Author: Peter Loan
/*
 * Copyright (c) 2006, Stanford University. All rights reserved. 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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
#include <OpenSim/Common/Transform.h>

namespace OpenSim {

class VisibleObject;
class AbstractBody;
class AbstractDynamicsEngine;
class MusclePoint;
class MuscleWrap;
class WrapResult;

//=============================================================================
//=============================================================================
/**
 * An abstract class that specifies the interface for a muscle wrapping
 * object.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API AbstractWrapObject : public Object
{

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
		noWrap,          // the muscle line did not intersect the wrap object
		insideRadius,    // one or both muscle points are inside the wrap object
		wrapped,         // successful wrap, but may not be 'best' path
		mandatoryWrap    // successful wrap that must be used (e.g., both tangent
	};                  // points are on the constrained side of the wrap object)

protected:

	PropertyDblArray _xyzBodyRotationProp;
	Array<double>& _xyzBodyRotation;

	PropertyDblArray _translationProp;
	Array<double>& _translation;

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

	AbstractBody* _body;

	Transform _pose;
	Transform _inversePose;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	AbstractWrapObject();
	AbstractWrapObject(const AbstractWrapObject& aWrapObject);
	virtual ~AbstractWrapObject();
	virtual Object* copy() const = 0;
#ifndef SWIG
	AbstractWrapObject& operator=(const AbstractWrapObject& aWrapObject);
#endif
   void copyData(const AbstractWrapObject& aWrapObject);

	virtual void scale(Array<double>& aScaleFactors) = 0;
	virtual void setup(AbstractDynamicsEngine* aEngine, AbstractBody* aBody);

	AbstractBody* getBody() const { return _body; }
	const double* getXYZBodyRotation() const { return &_xyzBodyRotation[0]; }
	const double* getTranslation() const { return &_translation[0]; }
	bool getActive() const { return _active; }
	bool getActiveUseDefault() const { return _activeProp.getUseDefault(); }
	const char* getQuadrantName() const { return _quadrantName.c_str(); }
	bool getQuadrantNameUseDefault() const { return _quadrantNameProp.getUseDefault(); }
	void setQuadrantName(const std::string& aName);
	virtual const char* getWrapTypeName() const = 0;
	virtual std::string getDimensionsString() const { return ""; } // TODO: total SIMM hack!
	int wrapMuscleSegment(MusclePoint& aPoint1, MusclePoint& aPoint2,
		const MuscleWrap& aMuscleWrap, WrapResult& aWrapResult) const;
	virtual int wrapLine(Array<double>& aPoint1, Array<double>& aPoint2,
		const MuscleWrap& aMuscleWrap, WrapResult& aWrapResult, bool& aFlag) const = 0;

	// Visible Object Support
	virtual VisibleObject* getDisplayer() { return &_displayer; };
	virtual void updateGeometry() {};

	virtual void peteTest() const;

protected:
	void setupProperties();
	void setupQuadrant();
	void setGeometryQuadrants(AnalyticGeometry *aGeometry) const;
private:
	void setNull();
//=============================================================================
};	// END of class AbstractWrapObject
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __AbstractWrapObject_h__


