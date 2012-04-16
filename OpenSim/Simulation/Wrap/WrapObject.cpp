// WrapObject.cpp
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
#include "WrapObject.h"
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PathPoint.h>
#include "WrapResult.h"
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Common/VisibleObject.h>
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
WrapObject::WrapObject() :
	Object(),
   _xyzBodyRotation(_xyzBodyRotationProp.getValueDblArray()),
   _translation(_translationProp.getValueDblVec()),
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
WrapObject::~WrapObject()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aWrapObject WrapObject to be copied.
 */
WrapObject::WrapObject(const WrapObject& aWrapObject) :
	Object(aWrapObject),
   _xyzBodyRotation(_xyzBodyRotationProp.getValueDblArray()),
   _translation(_translationProp.getValueDblVec()),
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
 * Set the data members of this WrapObject to their null values.
 */
void WrapObject::setNull()
{
	_quadrant = allQuadrants;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void WrapObject::setupProperties()
{
	const double defaultRotations[] = {0.0, 0.0, 0.0};
	_xyzBodyRotationProp.setName("xyz_body_rotation");
	_xyzBodyRotationProp.setValue(3, defaultRotations);
	_propertySet.append(&_xyzBodyRotationProp);

	const SimTK::Vec3 defaultTranslations(0.0);
	_translationProp.setName("translation");
	_translationProp.setValue(defaultTranslations);
	//_translationProp.setAllowableListSize(3);
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
void WrapObject::setup(Model& aModel, OpenSim::Body& aBody)
{
   _body = &aBody;
   _model = &aModel;

	setupQuadrant();

	SimTK::Rotation rot;
	rot.setRotationToBodyFixedXYZ(Vec3(_xyzBodyRotation[0], _xyzBodyRotation[1], _xyzBodyRotation[2]));
	_pose.set(rot, _translation);

	// Object is visible (has displayer) and depends on body it's attached to.
	_body->updDisplayer()->addDependent(getDisplayer());
	_displayer.setTransform(_pose);
	_displayer.setOwner(this);
}

//_____________________________________________________________________________
/**
 * Scale the wrap object by aScaleFactors. This base class method scales
 * only the _translation property, which is a local member. The derived classes
 * are expected to scale the object itself, because they contain the object's
 * dimensions.
 *
 * @param aScaleFactors The XYZ scale factors.
 */
void WrapObject::scale(const SimTK::Vec3& aScaleFactors)
{
   for (int i=0; i<3; i++)
      _translation[i] *= aScaleFactors[i];
}

//_____________________________________________________________________________
/**
 * set quadrants for the geometric object representing the wrap object
 * This has to be done after geometry object creation so it's not 
 * part of WrapObject::setup
 */
void WrapObject::setGeometryQuadrants(OpenSim::AnalyticGeometry *aGeometry) const
{
	// The following code should be moved to the base class WrapObject
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
 * Copy data members from one WrapObject to another.
 *
 * @param aWrapObject WrapObject to be copied.
 */
void WrapObject::copyData(const WrapObject& aWrapObject)
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
void WrapObject::setQuadrantName(const string& aName)
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
void WrapObject::setupQuadrant()
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
WrapObject& WrapObject::operator=(const WrapObject& aWrapObject)
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
 * Calculate the wrapping of one path segment over one wrap object.
 *
 * @param aPoint1 The first patth point
 * @param aPoint2 The second path point
 * @param aPathWrap An object holding the parameters for this path/wrap-object pairing
 * @param aWrapResult The result of the wrapping (tangent points, etc.)
 * @return The status, as a WrapAction enum
 */
int WrapObject::wrapPathSegment(const SimTK::State& s, PathPoint& aPoint1, PathPoint& aPoint2,
										  const PathWrap& aPathWrap, WrapResult& aWrapResult) const
{
   int return_code = noWrap;
	bool p_flag;
	Vec3 pt1(0.0);
	Vec3 pt2(0.0);

	// Convert the path points from the frames of the bodies they are attached
	// to to the frame of the wrap object's body
	_model->getSimbodyEngine().transformPosition(s, aPoint1.getBody(), aPoint1.getLocation(), getBody(), pt1);

	_model->getSimbodyEngine().transformPosition(s, aPoint2.getBody(), aPoint2.getLocation(), getBody(), pt2);

	// Convert the path points from the frame of the wrap object's body
	// into the frame of the wrap object
	pt1 = _pose.shiftBaseStationToFrame(pt1);
	pt2 = _pose.shiftBaseStationToFrame(pt2);

	return_code = wrapLine(s, pt1, pt2, aPathWrap, aWrapResult, p_flag);

   if (p_flag == true && return_code > 0) {
		// Convert the tangent points from the frame of the wrap object to the
		// frame of the wrap object's body
		aWrapResult.r1 = _pose.shiftFrameStationToBase(aWrapResult.r1);
		aWrapResult.r2 = _pose.shiftFrameStationToBase(aWrapResult.r2);

		// Convert the surface points (between the tangent points) from the frame of
		// the wrap object to the frame of the wrap object's body
		for (int i = 0; i < aWrapResult.wrap_pts.getSize(); i++)
			aWrapResult.wrap_pts.get(i) = _pose.shiftFrameStationToBase(aWrapResult.wrap_pts.get(i));
   }

   return return_code;
}
