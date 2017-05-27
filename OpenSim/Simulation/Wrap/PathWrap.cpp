/* -------------------------------------------------------------------------- *
 *                           OpenSim:  PathWrap.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include <OpenSim/Simulation/Model/Model.h>

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
PathWrap::PathWrap() : ModelComponent()
{
    setNull();
    constructProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
PathWrap::~PathWrap()
{
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
    resetPreviousWrap();
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void PathWrap::constructProperties()
{
    constructProperty_wrap_object("");
    constructProperty_method("hybrid");
    OpenSim::Array<int> range(-1, 2);
    constructProperty_range(range);
}


void PathWrap::extendConnectToModel(Model& model)
{
    Super::extendConnectToModel(model);

    _path = dynamic_cast<const GeometryPath*>(&getOwner());
    std::string msg = "PathWrap '" + getName()
        + "' must have a GeometryPath as its owner.";
    OPENSIM_THROW_IF(_path == nullptr, Exception, msg);

    ComponentList<const PhysicalFrame> bodiesList =
        model.getComponentList<PhysicalFrame>();
    for (ComponentList<PhysicalFrame>::const_iterator it = bodiesList.begin();
            it != bodiesList.end(); ++it) {
        const WrapObject* wo = it->getWrapObject(getWrapObjectName());
        if (wo) {
            _wrapObject = wo;
            updWrapPoint1().setParentFrame(wo->getFrame());
            updWrapPoint1().setWrapObject(wo);
            updWrapPoint2().setParentFrame(wo->getFrame());
            updWrapPoint2().setWrapObject(wo);
            break;
        }
    }

    if (get_method() == "hybrid" || get_method() == "Hybrid" || get_method() == "HYBRID")
        _method = hybrid;
    else if (get_method() == "midpoint" || get_method() == "Midpoint" || get_method() == "MIDPOINT")
        _method = midpoint;
    else if (get_method() == "axial" || get_method() == "Axial" || get_method() == "AXIAL")
        _method = axial;
    else if (get_method() == "Unassigned") {  // method was not specified in wrap object definition; use default
        _method = hybrid;
        upd_method() = "hybrid";
    } else {  // method was specified incorrectly in wrap object definition; throw an exception
        string errorMessage = "Error: wrapping method for wrap object " + getName() + " was either not specified, or specified incorrectly.";
        throw Exception(errorMessage);
    }
}

void PathWrap::setStartPoint( const SimTK::State& s, int aIndex)
{
    if ((aIndex != get_range(0)) && 
        (aIndex == -1 || get_range(1) == -1 || (aIndex >= 1 && aIndex <= get_range(1))))
    {
        upd_range(0) = aIndex;
    }
}

void PathWrap::setEndPoint( const SimTK::State& s, int aIndex)
{
    if ((aIndex != get_range(1)) && 
        (aIndex == -1 || get_range(0) == -1 || (aIndex >= get_range(0) && aIndex <= _path->getPathPointSet().getSize())))
    {
        upd_range(1) = aIndex;
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
    upd_wrap_object() = aWrapObject.getName();
}

void PathWrap::setMethod(WrapMethod aMethod)
{
    if (aMethod == axial) {
        _method = axial;
        upd_method() = "axial";
    } else if (aMethod == midpoint) {
        _method = midpoint;
        upd_method() = "midpoint";
    } else if (aMethod == hybrid) {
        _method = hybrid;
        upd_method() = "hybrid";
    }
}
