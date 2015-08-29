/* -------------------------------------------------------------------------- *
 *                          OpenSim:  WrapObject.cpp                          *
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
    _quadrantName(_quadrantNameProp.getValueStr())
{
    setNull();
    setupProperties();
    constructProperties();
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
    _quadrantName(_quadrantNameProp.getValueStr())
{
    setNull();
    setupProperties();
    //constructProperties();
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
}

void WrapObject::constructProperties()
{
    constructProperty_display_preference(1);
    Array<double> defaultColor(1.0, 3); //color default to 0, 1, 1
    defaultColor[0] = 0.0; 

    constructProperty_color(defaultColor);
}

void WrapObject::connectToModelAndBody(Model& aModel, PhysicalFrame& aBody)
{
   _body = &aBody;
   _model = &aModel;

    setupQuadrant();

    SimTK::Rotation rot;
    rot.setRotationToBodyFixedXYZ(Vec3(_xyzBodyRotation[0], _xyzBodyRotation[1], _xyzBodyRotation[2]));
    _pose.set(rot, _translation);
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
 * part of WrapObject::connectToModelAndBody()
 *
void WrapObject::setGeometryQuadrants(OpenSim::AnalyticGeometry *aGeometry) const
{
    // The following code should be moved to the base class WrapObject
    bool    quads[] = {true, true, true, true, true, true};

    if (_quadrant != allQuadrants){
        // Turn off half wrap object
        if (_wrapSign==1)
            quads[2*_wrapAxis]=false;
        else
            quads[2*_wrapAxis+1]=false;
    }
    aGeometry->setQuadrants(quads);
}*/
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
 * based on the name of the quadrant. This should be called in 
 * connectToModelAndBody() and whenever the quadrant name changes.
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
 * @param aPoint1 The first path point
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
    //_model->getSimbodyEngine().transformPosition(s, aPoint1.getBody(), aPoint1.getLocation(), getBody(), pt1);
    pt1 = aPoint1.getBody()
        .findLocationInAnotherFrame(s, aPoint1.getLocation(), getBody());
    
    //_model->getSimbodyEngine().transformPosition(s, aPoint2.getBody(), aPoint2.getLocation(), getBody(), pt2);
    pt2 = aPoint2.getBody()
        .findLocationInAnotherFrame(s, aPoint2.getLocation(), getBody());

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
            aWrapResult.wrap_pts.updElt(i) = _pose.shiftFrameStationToBase(aWrapResult.wrap_pts.get(i));
   }

   return return_code;
}
