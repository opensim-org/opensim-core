// AbstractWrapObject.cpp
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "AbstractWrapObject.h"
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/Model/AbstractBody.h>
#include <OpenSim/Simulation/Model/MusclePoint.h>
#include "WrapResult.h"
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Common/VisibleObject.h>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Mtx.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
AbstractWrapObject::AbstractWrapObject() :
	Object(),
   _xyzBodyRotation(_xyzBodyRotationProp.getValueDblArray()),
   _translation(_translationProp.getValueDblVec3()),
	_active(_activeProp.getValueBool()),
	_quadrantName(_quadrantNameProp.getValueStr()),
	_displayerProp(PropertyObj("", VisibleObject())),
   _displayer((VisibleObject&)_displayerProp.getValueObj())

{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
AbstractWrapObject::~AbstractWrapObject()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aWrapObject AbstractWrapObject to be copied.
 */
AbstractWrapObject::AbstractWrapObject(const AbstractWrapObject& aWrapObject) :
	Object(aWrapObject),
   _xyzBodyRotation(_xyzBodyRotationProp.getValueDblArray()),
   _translation(_translationProp.getValueDblVec3()),
	_active(_activeProp.getValueBool()),
	_quadrantName(_quadrantNameProp.getValueStr()),
	_displayerProp(PropertyObj("", VisibleObject())),
   _displayer((VisibleObject&)_displayerProp.getValueObj())
{
	setNull();
	setupProperties();
	copyData(aWrapObject);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this AbstractWrapObject to their null values.
 */
void AbstractWrapObject::setNull()
{
	setType("AbstractWrapObject");

	_quadrant = allQuadrants;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void AbstractWrapObject::setupProperties()
{
	const double defaultRotations[] = {0.0, 0.0, 0.0};
	_xyzBodyRotationProp.setName("xyz_body_rotation");
	_xyzBodyRotationProp.setValue(3, defaultRotations);
	_propertySet.append(&_xyzBodyRotationProp);

	const SimTK::Vec3 defaultTranslations(0.0);
	_translationProp.setName("translation");
	_translationProp.setValue(defaultTranslations);
	//_translationProp.setAllowableArraySize(3);
	_propertySet.append(&_translationProp);

	_activeProp.setName("active");
	_activeProp.setValue(true);
	_propertySet.append(&_activeProp);

	_quadrantNameProp.setName("quadrant");
	_quadrantNameProp.setValue("Unassigned");
	_propertySet.append(&_quadrantNameProp);

	_displayerProp.setName("display");
	_propertySet.append(&_displayerProp);

}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this wrap object.
 * @param aBody body containing this wrap object.
 */
void AbstractWrapObject::setup(AbstractDynamicsEngine* aEngine, AbstractBody* aBody)
{
   _body = aBody;

	setupQuadrant();

	// Form a transform matrix containing _xyzBodyRotation and _translation
	_pose.setIdentity();
	_pose.rotateXBodyFixed(_xyzBodyRotation[0], Transform::Radians);
	_pose.rotateYBodyFixed(_xyzBodyRotation[1], Transform::Radians);
	_pose.rotateZBodyFixed(_xyzBodyRotation[2], Transform::Radians);
	_pose.translate(_translation);

	/* Invert the forward transform and store the inverse. */
	Mtx::Invert(4, _pose.getMatrix(), _inversePose.getMatrix());

	// Object is visible (has displayer) and depends on body it's attached to.
	_body->getDisplayer()->addDependent(getDisplayer());
	_displayer.setTransform( _pose);
	_displayer.setOwner(this);
}
//_____________________________________________________________________________
/**
 * set quadrants for the geometric object representing the wrap object
 * This has to be done after geometry object creation so it's not 
 * part of AbstractWrapObject::setup
 */
void AbstractWrapObject::setGeometryQuadrants(AnalyticGeometry *aGeometry) const
{
	// The following code should be moved to the base class AbstractWrapObject
	bool	quads[] = {true, true, true, true, true, true};

	if (_quadrant != allQuadrants){
		// Turn off half wrap object
		if (_wrapSign==1)
			quads[2*_wrapAxis]=false;
		else
			quads[2*_wrapAxis+1]=false;
	}
	aGeometry->setQuadrants(quads);
}
//_____________________________________________________________________________
/**
 * Copy data members from one AbstractWrapObject to another.
 *
 * @param aWrapObject AbstractWrapObject to be copied.
 */
void AbstractWrapObject::copyData(const AbstractWrapObject& aWrapObject)
{
	_xyzBodyRotation = aWrapObject._xyzBodyRotation;
	_translation = aWrapObject._translation;
	_active = aWrapObject._active;
	_quadrantName = aWrapObject._quadrantName;
	_quadrant = aWrapObject._quadrant;
	_displayer = aWrapObject._displayer;
}

//_____________________________________________________________________________
/**
 * Set the name of the quadrant, and call setupQuadrant() to determine the
 * appropriate values of _quadrant, _wrapAxis, and _wrapSign.
 *
 * @param aName The name of the quadrant (e.g., "+x", "-y").
 */
void AbstractWrapObject::setQuadrantName(const string& aName)
{
	_quadrantName = aName;

	setupQuadrant();
}

//_____________________________________________________________________________
/**
 * Determine the appropriate values of _quadrant, _wrapAxis, and _wrapSign,
 * based on the name of the quadrant. This should be called in setup() and
 * whenever the quadrant name changes.
 */
void AbstractWrapObject::setupQuadrant()
{
	if (_quadrantName == "-x" || _quadrantName == "-X") {
		_quadrant = negativeX;
		_wrapAxis = 0;
		_wrapSign = -1;
	} else if (_quadrantName == "x" || _quadrantName == "+x" || _quadrantName == "X" || _quadrantName == "+X") {
		_quadrant = positiveX;
		_wrapAxis = 0;
		_wrapSign = 1;
	} else if (_quadrantName == "-y" || _quadrantName == "-Y") {
		_quadrant = negativeY;
		_wrapAxis = 1;
		_wrapSign = -1;
	} else if (_quadrantName == "y" || _quadrantName == "+y" || _quadrantName == "Y" || _quadrantName == "+Y") {
		_quadrant = positiveY;
		_wrapAxis = 1;
		_wrapSign = 1;
	} else if (_quadrantName == "-z" || _quadrantName == "-Z") {
		_quadrant = negativeZ;
		_wrapAxis = 2;
		_wrapSign = -1;
	} else if (_quadrantName == "z" || _quadrantName == "+z" || _quadrantName == "Z" || _quadrantName == "+Z") {
		_quadrant = positiveZ;
		_wrapAxis = 2;
		_wrapSign = 1;
	} else if (_quadrantName == "all" || _quadrantName == "ALL" || _quadrantName == "All") {
		_quadrant = allQuadrants;
		_wrapSign = 0;
	} else if (_quadrantName == "Unassigned") {  // quadrant was not specified in wrap object definition; use default
		_quadrant = allQuadrants;
		_quadrantName = "all";
		_wrapSign = 0;
	} else {  // quadrant was specified incorrectly in wrap object definition; throw an exception
		string errorMessage = "Error: quadrant for wrap object " + getName() + " was specified incorrectly.";
		throw Exception(errorMessage);
	}
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
AbstractWrapObject& AbstractWrapObject::operator=(const AbstractWrapObject& aWrapObject)
{
	// BASE CLASS
	Object::operator=(aWrapObject);

	return(*this);
}

//=============================================================================
// WRAPPING
//=============================================================================
//_____________________________________________________________________________
/**
 * Calculate the wrapping of one muscle segment over one wrap object.
 *
 * @param aPoint1 The first muscle attachment point
 * @param aPoint2 The second muscle attachment point
 * @param aMuscleWrap An object holding the parameters for this muscle/wrap-object pairing
 * @param aWrapResult The result of the wrapping (tangent points, etc.)
 * @return The status, as a WrapAction enum
 */
int AbstractWrapObject::wrapMuscleSegment(MusclePoint& aPoint1, MusclePoint& aPoint2,
														const MuscleWrap& aMuscleWrap, WrapResult& aWrapResult) const
{
   int return_code = noWrap;
	bool p_flag;
	Vec3 pt1(0.0);
	Vec3 pt2(0.0);

	// Convert the muscle points from the frames of the bodies they are attached
	// to to the frame of the wrap object's body
	getBody()->getDynamicsEngine()->transformPosition(*aPoint1.getBody(), aPoint1.getAttachment(), *getBody(), pt1);
	getBody()->getDynamicsEngine()->transformPosition(*aPoint2.getBody(), aPoint2.getAttachment(), *getBody(), pt2);

	// Convert the muscle points from the frame of the wrap object's body
	// into the frame of the wrap object
	_inversePose.transformPoint(pt1);
	_inversePose.transformPoint(pt2);

	return_code = wrapLine(pt1, pt2, aMuscleWrap, aWrapResult, p_flag);

   if (p_flag == true && return_code > 0)
   {
		// Convert the tangent points from the frame of the wrap object to the
		// frame of the wrap object's body
		_pose.transformPoint(aWrapResult.r1);
		_pose.transformPoint(aWrapResult.r2);

		// Convert the surface points (between the tangent points) from the frame of
		// the wrap object to the frame of the wrap object's body
		int i;
		for (i = 0; i < aWrapResult.wrap_pts.getSize(); i++)
			_pose.transformPoint(aWrapResult.wrap_pts.get(i).get());
   }

   return return_code;
}
