/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Ligament.cpp                           *
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
#include "Ligament.h"
#include "PointForceDirection.h"

#include <OpenSim/Common/Assertion.h>
#include <OpenSim/Common/SimmSpline.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

static const Vec3 DefaultLigamentColor(0, 1, 0); // Green for backward compatibility 

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
// Default constructor.
Ligament::Ligament()
{
    constructProperties();
}

//_____________________________________________________________________________
/*
 * Construct and initialize the properties for the ligament.
 * 
 * You should give each property a meaningful name and an informative comment.
 * The name you give each property is the tag that will be used in the XML
 * file. The comment will appear before the property in the XML file.
 * In addition, the comments are used for tool tips in the OpenSim GUI.
 *
 * All properties are added to the property set. Once added, they can be
 * read in and written to file.
 */
void Ligament::constructProperties()
{
    setAuthors("Peter Loan");
    constructProperty_path(GeometryPath());
    constructProperty_resting_length(0.0);
    constructProperty_pcsa_force(0.0);

    int forceLengthCurvePoints = 13;
    double forceLengthCurveX[] = {-5.00000000,  0.99800000,  0.99900000,  1.00000000,  1.10000000,  1.20000000,  1.30000000,  1.40000000,  1.50000000,  1.60000000,  1.60100000,  1.60200000,  5.00000000};
    double forceLengthCurveY[] = {0.00000000,  0.00000000,  0.00000000,  0.00000000,  0.03500000,  0.12000000,  0.26000000,  0.55000000,  1.17000000,  2.00000000,  2.00000000,  2.00000000,  2.00000000};
    SimmSpline forceLengthCurve
       (forceLengthCurvePoints, forceLengthCurveX, forceLengthCurveY);

    constructProperty_force_length_curve(forceLengthCurve);
}


void Ligament::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();

    // Resting length must be greater than 0.0.
    OPENSIM_ASSERT_FRMOBJ(get_resting_length() > 0.0);

    updPath().setDefaultColor(DefaultLigamentColor);
}


//------------------------------------------------------------------------------
//                            REALIZE DYNAMICS
//------------------------------------------------------------------------------
// See if anyone has an opinion about the path color and change it if so.
void Ligament::extendRealizeDynamics(const SimTK::State& state) const {
    Super::extendRealizeDynamics(state); // Mandatory first line

    if(appliesForce(state)){
        const SimTK::Vec3 color = computePathColor(state);
        if (!color.isNaN())
            getPath().setColor(state, color);
    }
}

//------------------------------------------------------------------------------
//                          COMPUTE PATH COLOR
//------------------------------------------------------------------------------
// This is the Ligament base class implementation for choosing the path
// color. Derived classes can override this with something meaningful.
// TODO: should the default attempt to use the ligament tension to control
// colors? Not sure how to scale.
SimTK::Vec3 Ligament::computePathColor(const SimTK::State& state) const {
    return SimTK::Vec3(SimTK::NaN);
}

//------------------------------------------------------------------------------
//                             ADD TO SYSTEM
//------------------------------------------------------------------------------
/**
 * allocate and initialize the SimTK state for this ligament.
 */
 void Ligament::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);
    // Cache the computed tension and strain of the ligament

    this->_tensionCV = addCacheVariable("tension", 0.0, SimTK::Stage::Velocity);
    this->_strainCV = addCacheVariable("strain", 0.0, SimTK::Stage::Velocity);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// LENGTH
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the length of the ligament. This is a convenience function that passes
 * the request on to the ligament path.
 *
 * @return Current length of the ligament path.
 */
double Ligament::getLength(const SimTK::State& s) const
{
    return getPath().getLength(s);
}

//_____________________________________________________________________________
/**
 * Set the resting length.
 *
 * @param aRestingLength The resting length of the ligament.
 * @return Whether the resting length was successfully changed.
 */
bool Ligament::setRestingLength(double aRestingLength)
{
    set_resting_length(aRestingLength);
    return true;
}

//_____________________________________________________________________________
/**
 * Set the maximum isometric force.
 *
 * @param aMaxIsometricForce The maximum isometric force of the ligament.
 * @return Whether the maximum isometric force was successfully changed.
 */
bool Ligament::setMaxIsometricForce(double aMaxIsometricForce)
{
    set_pcsa_force(aMaxIsometricForce);
    return true;
}

//_____________________________________________________________________________
/**
 * Set the force-length curve.
 *
 * @param aForceLengthCurve Pointer to a force-length curve (Function).
 * @return Whether the force-length curve was successfully changed.
 */
bool Ligament::setForceLengthCurve(const Function& aForceLengthCurve)
{
    set_force_length_curve(aForceLengthCurve);
    return true;
}
//==============================================================================
// SCALING
//==============================================================================
void Ligament::extendPostScale(const SimTK::State& s, const ScaleSet& scaleSet)
{
    Super::extendPostScale(s, scaleSet);

    AbstractGeometryPath& path = updPath();
    if (path.getPreScaleLength(s) > 0.0)
    {
        double scaleFactor = path.getLength(s) / path.getPreScaleLength(s);
        upd_resting_length() *= scaleFactor;

        // Clear the pre-scale length that was stored in the path.
        path.setPreScaleLength(s, 0.0);
    }
}

const double& Ligament::getTension(const SimTK::State& s) const
{
    return getCacheVariableValue(s, _tensionCV);
}


//=============================================================================
// COMPUTATION
//=============================================================================
/**
 * Compute the moment-arm of this muscle about a coordinate.
 */
double Ligament::computeMomentArm(const SimTK::State& s, Coordinate& aCoord) const
{
    return getPath().computeMomentArm(s, aCoord);
}



void Ligament::computeForce(const SimTK::State& s, 
                              SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                              SimTK::Vector& generalizedForces) const
{
    const auto& path = getPath();
    const double& restingLength = get_resting_length();
    const double& pcsaForce = get_pcsa_force();

    double tension = 0;

    if (path.getLength(s) <= restingLength){
        setCacheVariableValue(s, _tensionCV, tension);
        return;
    }
    
    // evaluate normalized tendon force length curve
    tension = getForceLengthCurve().calcValue(
        SimTK::Vector(1, path.getLength(s)/restingLength))* pcsaForce;
    setCacheVariableValue(s, _tensionCV, tension);

    path.addInEquivalentForces(s, tension, bodyForces, generalizedForces);
}

