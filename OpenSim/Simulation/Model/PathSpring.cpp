/* -------------------------------------------------------------------------- *
 *                           OpenSim:  PathSpring.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

static const Vec3 DefaultPathSpringColor(0, 1, 0); // Green for backward compatibility

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
void PathSpring::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();

    GeometryPath& path = upd_GeometryPath();
    path.setDefaultColor(DefaultPathSpringColor);

    OPENSIM_THROW_IF_FRMOBJ(
        (SimTK::isNaN(get_resting_length()) || get_resting_length() < 0),
        InvalidPropertyValue, getProperty_resting_length().getName(),
        "Resting length cannot be less than zero");
    OPENSIM_THROW_IF_FRMOBJ(
        (SimTK::isNaN(get_stiffness()) || get_stiffness() < 0),
        InvalidPropertyValue, getProperty_stiffness().getName(),
        "Stiffness cannot be less than zero");
    OPENSIM_THROW_IF_FRMOBJ(
        (SimTK::isNaN(get_dissipation()) || get_dissipation() < 0),
        InvalidPropertyValue, getProperty_dissipation().getName(),
        "Dissipation cannot be less than zero");
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


//==============================================================================
// SCALING
//==============================================================================
void PathSpring::
extendPostScale(const SimTK::State& s, const ScaleSet& scaleSet)
{
    Super::extendPostScale(s, scaleSet);

    GeometryPath& path = upd_GeometryPath();
    if (path.getPreScaleLength(s) > 0.0)
    {
        double scaleFactor = path.getLength(s) / path.getPreScaleLength(s);
        upd_resting_length() *= scaleFactor;

        // Clear the pre-scale length that was stored in the GeometryPath.
        path.setPreScaleLength(s, 0.0);
    }
}

//=============================================================================
// COMPUTATION
//=============================================================================
/**
 * Compute the moment-arm of this muscle about a coordinate.
 */
double PathSpring::computeMomentArm(const SimTK::State& s, const Coordinate& aCoord) const
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
        applyForceToPoint(s, PFDs[i]->frame(), PFDs[i]->point(), 
                          tension*PFDs[i]->direction(), bodyForces);
    }

    for(int i=0; i < PFDs.getSize(); i++)
        delete PFDs[i];
}
