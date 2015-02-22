/* -------------------------------------------------------------------------- *
 *                          OpenSim:  PathPoint.cpp                           *
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
#include "PathPoint.h"
#include "BodySet.h"
#include "Model.h"
#include <OpenSim/Common/Geometry.h>
#include "GeometryPath.h"
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h> 
#include <OpenSim/Simulation/Wrap/WrapObject.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

Geometry* PathPoint::_defaultGeometry= AnalyticSphere::createSphere(0.005);

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
PathPoint::PathPoint() :
   _location(_locationProp.getValueDblVec()),
    _bodyName(_bodyNameProp.getValueStr())
{
    setNull();
    setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
PathPoint::~PathPoint()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aPoint PathPoint to be copied.
 */
PathPoint::PathPoint(const PathPoint &aPoint) :
   Object(aPoint),
   _location(_locationProp.getValueDblVec()),
    _bodyName(_bodyNameProp.getValueStr())
{
    setNull();
    setupProperties();
    copyData(aPoint);
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one PathPoint to another.
 *
 * @param aPoint PathPoint to be copied.
 */
void PathPoint::copyData(const PathPoint &aPoint)
{
    _location = aPoint._location;
    _displayer = aPoint._displayer;
    _bodyName = aPoint._bodyName;
    _body = aPoint._body;
    _path = aPoint._path;
}

//_____________________________________________________________________________
/**
 * Initialize a PathPoint with data from another PathPoint.
 *
 * @param aPoint PathPoint to be copied.
 */
void PathPoint::init(const PathPoint& aPoint)
{
    copyData(aPoint);
}

//_____________________________________________________________________________
/**
 * Set the data members of this PathPoint to their null values.
 */
void PathPoint::setNull()
{
    _body = NULL;
    _path = NULL;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void PathPoint::setupProperties()
{
    const SimTK::Vec3 defaultLocation(0.0);
    _locationProp.setName("location");
    _locationProp.setValue(defaultLocation);
    //_locationProp.setAllowableListSize(3);
    _propertySet.append(&_locationProp);

    _bodyNameProp.setName("body");
    _propertySet.append(&_bodyNameProp);
}

//_____________________________________________________________________________
/**
 * TODO: All Points should be Components that use Connectors
 * and extendConnect(). This must go away! -aseth
 */
void PathPoint::connectToModelAndPath(const Model& aModel, GeometryPath& aPath)
{
    _path = &aPath;
    _model  = &aModel;

    if (_bodyName == aModel.getGround().getName()){
        _body = &const_cast<Model*>(&aModel)->updGround();
        return;
    }

    // Look up the body by name in the kinematics engine and
    // store a pointer to it.
    if (!aModel.getBodySet().contains(_bodyName))
    {
        string errorMessage = "Body " + _bodyName + " referenced in path" + aPath.getName() + " not found in model " + aModel.getName();
        throw Exception(errorMessage);
    }
    _body = &const_cast<Model*>(&aModel)->updBodySet().get(_bodyName);
}

//_____________________________________________________________________________
/**
 * Update geometry of the muscle point.
 *
 */
void PathPoint::updateGeometry()
{
    Transform position;
    position.setP(_location);
    updDisplayer()->setTransform(position);
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
PathPoint& PathPoint::operator=(const PathPoint &aPoint)
{
    // BASE CLASS
    Object::operator=(aPoint);

    copyData(aPoint);

    return(*this);
}

//_____________________________________________________________________________
/**
 * Set the body that this point is fixed to.
 *
 * @param aBody Reference to the body.
 */
void PathPoint::setBody(PhysicalFrame& aBody)
{
    if (&aBody == _body)
        return;

    _body = &aBody;
    _bodyName = aBody.getName();
}

//_____________________________________________________________________________
/**
 * Change the body that this point is attached to. It assumes that the body is
 * already set, so that connectToModelAndPath() needs to be called to update 
 * dependent information.
 *
 * @param s State.
 * @param aBody Reference to the body.
 */
void PathPoint::changeBodyPreserveLocation(const SimTK::State& s, PhysicalFrame& aBody)
{
    if (!_body){
        throw Exception("PathPoint::changeBodyPreserveLocation attempted to "
            " change the body on PathPoint which was not assigned to a body.");
    }
    // if it is already assigned to aBody, do nothing
    if (_body == &aBody )
        return;

    // Preserve location means to switch bodies without changing
    // the location of the point in the inertial reference frame.
    //aBody.getModel().getSimbodyEngine().transformPosition(s, *_body, _location, aBody, _location);

    _location = _body->findLocationInAnotherFrame(s, _location, aBody);
    _bodyName = aBody.getName();

    // now assign this point's body to point to aBody
    _body = &aBody;

    connectToModelAndPath(aBody.getModel(), *getPath());
}

//_____________________________________________________________________________
/**
 * Set the XYZ location of the point.
 *
 * @param aLocation The XYZ coordinates.
 */
void PathPoint::setLocation( const SimTK::State& s, const SimTK::Vec3& aLocation)
{
    _location = aLocation;
}

//_____________________________________________________________________________
/**
 * Set the X, Y, or Z location of the point.
 *
 * @param aCoordIndex The coordinate to change (0=X, 1=Y, 2=Z).
 * @param aLocation The X, Y, or Z coordinate.
 */
void PathPoint::setLocation( const SimTK::State& s, int aCoordIndex, double aLocation)
{
    if (aCoordIndex >= 0 && aCoordIndex <= 2)
        _location[aCoordIndex] = aLocation;
}

//_____________________________________________________________________________
/**
 * Get the velocity of the point in the body's local reference frame.
 *
 * @param aVelocity The velocity.
 */

void PathPoint::getVelocity(const SimTK::State& s, SimTK::Vec3& aVelocity)
{
    aVelocity[0] = aVelocity[1] = aVelocity[2] = 0.0;
}

PathPoint* PathPoint::makePathPointOfType(PathPoint* aPoint, const string& aNewTypeName)
{
    PathPoint* newPoint = NULL;

    if (aPoint != NULL) {
        Object* newObject = Object::newInstanceOfType(aNewTypeName);
        if (newObject) {
            newPoint = dynamic_cast<PathPoint*>(newObject);
            if (newPoint) {
                // Copy the contents from aPoint.
                newPoint->init(*aPoint);
            }
        }
    }

    return newPoint;
}

//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________
/**
 * Scale the muscle point.
 *
 * @param aScaleFactors the XYZ scale factors to scale the point by.
 */
void PathPoint::scale(const SimTK::State& s, const SimTK::Vec3& aScaleFactors)
{
    for (int i = 0; i < 3; i++)
        _location[i] *= aScaleFactors[i];

    updateGeometry();
}
