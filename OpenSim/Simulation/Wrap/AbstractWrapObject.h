#ifndef __AbstractWrapObject_h__
#define __AbstractWrapObject_h__

// AbstractWrapObject.h
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
	virtual VisibleObject* getDisplayer() const { return &_displayer; };
	virtual void updateGeometry() {};

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


