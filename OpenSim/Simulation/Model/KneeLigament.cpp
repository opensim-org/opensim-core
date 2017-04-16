/* -------------------------------------------------------------------------- *
 *                           OpenSim:  KneeLigament.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2013 Stanford University and the Authors                *
 * Author(s): Peter Loan, Dimitar Stanev                                      *
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
#include "KneeLigament.h"
#include "GeometryPath.h"
#include "PointForceDirection.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

static const Vec3 DefaultLigamentColor(.9,.9,.9); // mostly white 

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
// Default constructor.
KneeLigament::KneeLigament()
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
void KneeLigament::constructProperties()
{
    setAuthors("Dimitar Stanev");
    constructProperty_GeometryPath(GeometryPath());
    constructProperty_resting_length(0.0);
	constructProperty_epsilon_length(0.0);
	constructProperty_ligament_stiffness(0.0);
}


void KneeLigament::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();

    // Resting length must be greater than 0.0.
    assert(get_resting_length() > 0.0);

    GeometryPath& path = upd_GeometryPath();
    path.setDefaultColor(DefaultLigamentColor);
    addComponent(&path);
    path.setOwner(this);
}


//------------------------------------------------------------------------------
//                            REALIZE DYNAMICS
//------------------------------------------------------------------------------
// See if anyone has an opinion about the path color and change it if so.
void KneeLigament::extendRealizeDynamics(const SimTK::State& state) const {
    Super::extendRealizeDynamics(state); // Mandatory first line

    if(!isDisabled(state)){
        const SimTK::Vec3 color = computePathColor(state);
        if (!color.isNaN())
            getGeometryPath().setColor(state, color);
    }
}

//------------------------------------------------------------------------------
//                          COMPUTE PATH COLOR
//------------------------------------------------------------------------------
// This is the Ligament base class implementation for choosing the path
// color. Derived classes can override this with something meaningful.
// TODO: should the default attempt to use the ligament tension to control
// colors? Not sure how to scale.
SimTK::Vec3 KneeLigament::computePathColor(const SimTK::State& state) const {
    return SimTK::Vec3(SimTK::NaN);
}

//------------------------------------------------------------------------------
//                             ADD TO SYSTEM
//------------------------------------------------------------------------------
/**
 * allocate and initialize the SimTK state for this ligament.
 */
 void KneeLigament::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);
    // Cache the computed tension and strain of the ligament
    addCacheVariable<double>("tension", 0.0, SimTK::Stage::Velocity);
    addCacheVariable<double>("strain", 0.0, SimTK::Stage::Velocity);
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
double KneeLigament::getLength(const SimTK::State& s) const
{
    return getGeometryPath().getLength(s);
}

//_____________________________________________________________________________
/**
 * Set the resting length.
 *
 * @param aRestingLength The resting length of the ligament.
 * @return Whether the resting length was successfully changed.
 */
bool KneeLigament::setRestingLength(double aRestingLength)
{
    set_resting_length(aRestingLength);
    return true;
}

//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________
/**
 * Perform computations that need to happen before the ligament is scaled.
 * For this object, that entails calculating and storing the
 * length in the current body position.
 *
 * @param aScaleSet XYZ scale factors for the bodies.
 */
void KneeLigament::preScale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
    updGeometryPath().preScale(s, aScaleSet);
}

//_____________________________________________________________________________
/**
 * Scale the ligament.
 *
 * @param aScaleSet XYZ scale factors for the bodies
 * @return Whether or not the ligament was scaled successfully
 */
void KneeLigament::scale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
    updGeometryPath().scale(s, aScaleSet);
}

//_____________________________________________________________________________
/**
 * Perform computations that need to happen after the ligament is scaled.
 * For this object, that entails comparing the length before and after scaling,
 * and scaling the resting length a proportional amount.
 *
 * @param aScaleSet XYZ scale factors for the bodies.
 */
void KneeLigament::postScale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
    GeometryPath& path          = updGeometryPath();
    double&       restingLength = upd_resting_length();

    path.postScale(s, aScaleSet);

    if (path.getPreScaleLength(s) > 0.0)
    {
        double scaleFactor = path.getLength(s) / path.getPreScaleLength(s);

        // Scale resting length by the same amount as the change in
        // total ligament length (in the current body position).
        restingLength *= scaleFactor;

        path.setPreScaleLength(s, 0.0);
    }
}

const double& KneeLigament::getTension(const SimTK::State& s) const
{
    return getCacheVariableValue<double>(s, "tension"); 
}


//=============================================================================
// COMPUTATION
//=============================================================================
/**
 * Compute the moment-arm of this muscle about a coordinate.
 */
double KneeLigament::computeMomentArm(const SimTK::State& s, Coordinate& aCoord) const
{
    return getGeometryPath().computeMomentArm(s, aCoord);
}



void KneeLigament::computeForce(const SimTK::State& s, 
                              SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                              SimTK::Vector& generalizedForces) const
{
    const GeometryPath& path = getGeometryPath();
    const double& restingLength = get_resting_length();

    double force = 0;

    if (path.getLength(s) <= restingLength){
        setCacheVariableValue<double>(s, "tension", force);
        return;
    }
    
    // evaluate normalized tendon force length curve
	force = computeLigamentForceStrain(path.getLength(s));
    setCacheVariableValue<double>(s, "tension", force);

    OpenSim::Array<PointForceDirection*> PFDs;
    path.getPointForceDirections(s, &PFDs);

    for (int i=0; i < PFDs.getSize(); i++) {
        applyForceToPoint(s, PFDs[i]->body(), PFDs[i]->point(), 
                          force*PFDs[i]->direction(), bodyForces);
    }
    for(int i=0; i < PFDs.getSize(); i++)
        delete PFDs[i];
}

//_____________________________________________________________________________
/**
 * Get the visible object used to represent the Ligament.
 */
const VisibleObject* KneeLigament::getDisplayer() const
{ 
    return getGeometryPath().getDisplayer(); 
}

//_____________________________________________________________________________
/**
 * Update the visible object used to represent the Ligament.
 */
void KneeLigament::updateDisplayer(const SimTK::State& s) const
{
    getGeometryPath().updateDisplayer(s);
}

//_____________________________________________________________________________
/**
* Computes the strain force function of the ligament.
*/
double KneeLigament::computeLigamentForceStrain(double length) const
{
	double e = (length - get_resting_length()) / get_resting_length();
	double e_l = get_epsilon_length();
	double k = get_ligament_stiffness();

	if (e < 0)
	{
		return 0;
	}
	else if (e <= 2 * e_l)
	{
		return 0.25 * k * e * e / e_l;
	}
	else
	{
		return k * (e - e_l);
	}
}
