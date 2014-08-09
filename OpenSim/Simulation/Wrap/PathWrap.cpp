/* -------------------------------------------------------------------------- *
 *                           OpenSim:  PathWrap.cpp                           *
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
#include "PathWrap.h"
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/Model/GeometryPath.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
PathWrap::PathWrap() :
    Object(),
    _wrapObjectName(_wrapObjectNameProp.getValueStr()),
    _methodName(_methodNameProp.getValueStr()),
    _range(_rangeProp.getValueIntArray())
{
    setNull();
    setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
PathWrap::~PathWrap()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aPathWrap PathWrap to be copied.
 */
PathWrap::PathWrap(const PathWrap& aPathWrap) :
    Object(aPathWrap),
    _wrapObjectName(_wrapObjectNameProp.getValueStr()),
    _methodName(_methodNameProp.getValueStr()),
    _range(_rangeProp.getValueIntArray())
{
    setNull();
    setupProperties();
    copyData(aPathWrap);
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this PathWrap to their null values.
 */
void PathWrap::setNull()
{
    _method = hybrid;

    resetPreviousWrap();
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void PathWrap::setupProperties()
{
    _wrapObjectNameProp.setName("wrap_object");
    _propertySet.append(&_wrapObjectNameProp);

    _methodNameProp.setName("method");
    _methodNameProp.setValue("Unassigned");
    _propertySet.append(&_methodNameProp);

    const int defaultRange[] = {-1, -1};
    _rangeProp.setName("range");
    _rangeProp.setValue(2, defaultRange);
    _rangeProp.setAllowableListSize(2);
    _propertySet.append(&_rangeProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 * @param s current state of  system
 * @param aModel pointer to the OpenSim model.
 */
void PathWrap::connectToModelAndPath(const Model& aModel, GeometryPath& aPath)
{
    int i;
    const BodySet& bodySet = aModel.getBodySet();

    _path = &aPath;

    for (i = 0; i < bodySet.getSize(); i++) {
        const WrapObject* wo = bodySet.get(i).getWrapObject(getWrapObjectName());
        if (wo) {
            _wrapObject = wo;
            _wrapPoints[0].setBody(bodySet.get(i));
            _wrapPoints[0].setWrapObject(wo);
            _wrapPoints[1].setBody(bodySet.get(i));
            _wrapPoints[1].setWrapObject(wo);
            break;
        }
    }

    // connectToModelAndPath() must be called after setBody() because it requires
    // that _bodyName already be assigned.
    _wrapPoints[0].connectToModelAndPath(aModel, aPath);
    _wrapPoints[1].connectToModelAndPath(aModel, aPath);

    if (_methodName == "hybrid" || _methodName == "Hybrid" || _methodName == "HYBRID")
        _method = hybrid;
    else if (_methodName == "midpoint" || _methodName == "Midpoint" || _methodName == "MIDPOINT")
        _method = midpoint;
    else if (_methodName == "axial" || _methodName == "Axial" || _methodName == "AXIAL")
        _method = axial;
    else if (_methodName == "Unassigned") {  // method was not specified in wrap object definition; use default
        _method = hybrid;
        _methodName = "hybrid";
    } else {  // method was specified incorrectly in wrap object definition; throw an exception
        string errorMessage = "Error: wrapping method for wrap object " + getName() + " was either not specified, or specified incorrectly.";
        throw Exception(errorMessage);
    }
}

//_____________________________________________________________________________
/**
 * Copy data members from one PathWrap to another.
 *
 * @param aPathWrap PathWrap to be copied.
 */
void PathWrap::copyData(const PathWrap& aPathWrap)
{
    _wrapObjectName = aPathWrap._wrapObjectName;
    _methodName = aPathWrap._methodName;
    _method = aPathWrap._method;
    _range = aPathWrap._range;
    _wrapObject = aPathWrap._wrapObject;
    _previousWrap = aPathWrap._previousWrap;

    _wrapPoints[0] = aPathWrap._wrapPoints[0];
    _wrapPoints[1] = aPathWrap._wrapPoints[1];
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
PathWrap& PathWrap::operator=(const PathWrap& aPathWrap)
{
    // BASE CLASS
    Object::operator=(aPathWrap);

    copyData(aPathWrap);

    return(*this);
}

PathWrapPoint& PathWrap::getWrapPoint(int aIndex)
{
    if (aIndex < 0 || aIndex > 1)
    {
        // TODO string errorMessage = "PathWrap::getWrapPoint(): invalid index (" + aIndex + ")";
        // throw Exception(errorMessage);
    }

    return _wrapPoints[aIndex];
}

void PathWrap::setStartPoint( const SimTK::State& s, int aIndex)
{
    if ((aIndex != _range[0]) && (aIndex == -1 || _range[1] == -1 || (aIndex >= 1 && aIndex <= _range[1])))
    {
        _range[0] = aIndex;
        _rangeProp.setValueIsDefault(false);
    }
}

void PathWrap::setEndPoint( const SimTK::State& s, int aIndex)
{
    if ((aIndex != _range[1]) && (aIndex == -1 || _range[0] == -1 || (aIndex >= _range[0] && aIndex <= _path->getPathPointSet().getSize())))
    {
        _range[1] = aIndex;
        _rangeProp.setValueIsDefault(false);
    }
}

void PathWrap::resetPreviousWrap()
{
    _previousWrap.startPoint = -1;
    _previousWrap.endPoint = -1;

    _previousWrap.wrap_pts.setSize(0);
    _previousWrap.wrap_path_length = 0.0;

    int i;
    for (i = 0; i < 3; i++) {
        _previousWrap.r1[i] = -std::numeric_limits<SimTK::Real>::infinity();
        _previousWrap.r2[i] = -std::numeric_limits<SimTK::Real>::infinity();
        _previousWrap.sv[i] = -std::numeric_limits<SimTK::Real>::infinity();
    }
}

void PathWrap::setPreviousWrap(const WrapResult& aWrapResult)
{
    _previousWrap = aWrapResult;
}

void PathWrap::setWrapObject(WrapObject& aWrapObject)
{
    _wrapObject = &aWrapObject;
    _wrapObjectName = aWrapObject.getName();
}

void PathWrap::setMethod(WrapMethod aMethod)
{
    if (aMethod == axial) {
        _method = axial;
        _methodName = "axial";
    } else if (aMethod == midpoint) {
        _method = midpoint;
        _methodName = "midpoint";
    } else if (aMethod == hybrid) {
        _method = hybrid;
        _methodName = "hybrid";
    }
}
