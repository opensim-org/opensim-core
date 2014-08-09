/* -------------------------------------------------------------------------- *
 *                           OpenSim:  PathSpring.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2013 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                     *
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
#include "PathSpring.h"
#include "GeometryPath.h"
#include "PointForceDirection.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

static const Vec3 DefaultPathSpringColor(.9,.9,.9); // mostly white 

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
// Default constructor.
PathSpring::PathSpring()
{
    constructProperties();
}

PathSpring::PathSpring(const string& name, double restLength, 
                       double stiffness, double dissipation)
{
    constructProperties();
    setName(name);
    set_resting_length(restLength);
    set_stiffness(stiffness);
    set_dissipation(dissipation);
}

//_____________________________________________________________________________
/*
 * Construct and initialize the properties for the PathSpring.
 */
void PathSpring::constructProperties()
{
    setAuthors("Ajay Seth");
    constructProperty_GeometryPath(GeometryPath());
    constructProperty_resting_length(SimTK::NaN);
    constructProperty_stiffness(SimTK::NaN);
    constructProperty_dissipation(SimTK::NaN);

    constructOutput<double>("stretch", 
           std::bind(&PathSpring::getStretch, this, std::placeholders::_1),
                      SimTK::Stage::Position);
}

//_____________________________________________________________________________
/*
 * Set the resting length.
 */
void PathSpring::setRestingLength(double restingLength)
{
    set_resting_length(restingLength);
}
//_____________________________________________________________________________
/*
 * Set the stiffness.
 */
void PathSpring::setStiffness(double stiffness)
{
    set_stiffness(stiffness);
}
//_____________________________________________________________________________
/*
 * Set the dissipation.
 */
void PathSpring::setDissipation(double dissipation)
{
    set_dissipation(dissipation);
}

//_____________________________________________________________________________
/**
 * Perform some setup functions that happen after the
 * PathSpring has been deserialized or copied.
 *
 * @param aModel model containing this PathSpring.
 */
void PathSpring::connectToModel(Model& aModel)
{
    GeometryPath& path = upd_GeometryPath();
    const double& restingLength = get_resting_length();

    path.setDefaultColor(DefaultPathSpringColor);

    // Specify underlying ModelComponents prior to calling 
    // Super::connectToModel() to automatically propagate connectToModel()
    // to subcomponents. Subsequent addToSystem() will also be automatically
    // propagated to subcomponents.
    // TODO: this is awkward; subcomponent API needs to be revisited (sherm)
    addComponent(&path);

    //TODO: can't call this at start of override; this is an API bug.
    Super::connectToModel(aModel);

    // _model will be NULL when objects are being registered.
    if (!_model)
        return;

    // Resting length must be greater than 0.0.
    assert(restingLength > 0.0);

    path.setOwner(this);
}

//_____________________________________________________________________________
/**
 * allocate and initialize the SimTK state for this PathSpring.
 */
 void PathSpring::addToSystem(SimTK::MultibodySystem& system) const
{
    Super::addToSystem(system);
}


void PathSpring::initStateFromProperties( SimTK::State& s) const
{
    Super::initStateFromProperties(s);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// LENGTH and SPEED
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the length of the PathSpring. This is a convenience function that passes
 * the request on to the PathSpring path.
 *
 * @return Current length of the PathSpring path.
 */
double PathSpring::getLength(const SimTK::State& s) const
{
    return getGeometryPath().getLength(s);
}

double PathSpring::getStretch(const SimTK::State& s) const
{
    const double& length = getLength(s);
    const double& restingLength = get_resting_length();
    return length > restingLength ? (length-restingLength) : 0.0;
}


double PathSpring::getLengtheningSpeed(const SimTK::State& s) const
{
    return getGeometryPath().getLengtheningSpeed(s);
}

double PathSpring::getTension(const SimTK::State& s) const
{
    // evaluate tension in the spring
    // note tension is positive and produces shortening
    // damping opposes lengthening, which is positive lengthening speed
    // there for stretch and lengthening speed increase tension
    return getStiffness()*getStretch(s) *                   //elastic force
                (1+getDissipation()*getLengtheningSpeed(s));   //dissipation 
}


//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________
/**
 * Perform computations that need to happen before the PathSpring is scaled.
 * For this object, that entails calculating and storing the
 * length in the current body position.
 *
 * @param aScaleSet XYZ scale factors for the bodies.
 */
void PathSpring::preScale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
    updGeometryPath().preScale(s, aScaleSet);
}

//_____________________________________________________________________________

void PathSpring::scale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
    updGeometryPath().scale(s, aScaleSet);
}

//_____________________________________________________________________________
/**
 * Perform computations that need to happen after the PathSpring is scaled.
 * For this object, that entails comparing the length before and after scaling,
 * and scaling the resting length a proportional amount.
 *
 * @param aScaleSet XYZ scale factors for the bodies.
 */
void PathSpring::postScale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
    GeometryPath& path = updGeometryPath();
    path.postScale(s, aScaleSet);

    if (path.getPreScaleLength(s) > 0.0)
    {
        double scaleFactor = path.getLength(s) / path.getPreScaleLength(s);
        // Scale resting length by the same amount as the change in
        // total PathSpring length (in the current body position).
        upd_resting_length() *= scaleFactor;

        path.setPreScaleLength(s, 0.0);
    }
}


//=============================================================================
// COMPUTATION
//=============================================================================
/**
 * Compute the moment-arm of this muscle about a coordinate.
 */
double PathSpring::computeMomentArm(const SimTK::State& s, Coordinate& aCoord) const
{
    return getGeometryPath().computeMomentArm(s, aCoord);
}



void PathSpring::computeForce(const SimTK::State& s, 
                              SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                              SimTK::Vector& generalizedForces) const
{
    const GeometryPath& path = getGeometryPath();
    const double& tension = getTension(s);

    OpenSim::Array<PointForceDirection*> PFDs;
    path.getPointForceDirections(s, &PFDs);

    for (int i=0; i < PFDs.getSize(); i++) {
        applyForceToPoint(s, PFDs[i]->body(), PFDs[i]->point(), 
                          tension*PFDs[i]->direction(), bodyForces);
    }

    for(int i=0; i < PFDs.getSize(); i++)
        delete PFDs[i];
}

//_____________________________________________________________________________
/**
 * Get the visible object used to represent the PathSpring.
 */
const VisibleObject* PathSpring::getDisplayer() const
{ 
    return getGeometryPath().getDisplayer(); 
}

//_____________________________________________________________________________
/**
 * Update the visible object used to represent the PathSpring.
 */
void PathSpring::updateDisplayer(const SimTK::State& s) const
{
    getGeometryPath().updateDisplayer(s);
}
