/* -------------------------------------------------------------------------- *
 *                        OpenSim:  EllipsoidJoint.cpp                        *
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

//==============================================================================
// INCLUDES
//==============================================================================
#include "EllipsoidJoint.h"
#include <OpenSim/Simulation/Model/Model.h>
#include "simbody/internal/MobilizedBody_Ellipsoid.h"

//==============================================================================
// STATICS
//==============================================================================
using namespace std;
using namespace SimTK;
using namespace OpenSim;

//==============================================================================
// CONSTRUCTORS AND DESTRUCTOR
//==============================================================================
EllipsoidJoint::EllipsoidJoint() : Super()
{
    constructProperties();
}

EllipsoidJoint::EllipsoidJoint(const std::string&    name,
                               const PhysicalFrame&  parent,
                               const PhysicalFrame&  child,
                               const SimTK::Vec3&    ellipsoidRadii) :
                               Super(name, parent, child)
{
    constructProperties();
    set_radii_x_y_z(ellipsoidRadii);
}

EllipsoidJoint::EllipsoidJoint(const std::string&    name,
                               const PhysicalFrame&  parent,
                               const SimTK::Vec3&    locationInParent,
                               const SimTK::Vec3&    orientationInParent,
                               const PhysicalFrame&  child,
                               const SimTK::Vec3&    locationInChild,
                               const SimTK::Vec3&    orientationInChild,
                               const SimTK::Vec3&    ellipsoidRadii) :
    Super(name, parent, locationInParent, orientationInParent,
          child, locationInChild, orientationInChild)
{
    constructProperties();
    set_radii_x_y_z(ellipsoidRadii);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void EllipsoidJoint::constructProperties()
{
    setAuthors("Ajay Seth");
    SimTK::Vec3 radii(NaN);
    constructProperty_radii_x_y_z(radii);
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the EllipsoidJoint's radii. If the system is created, will attempt
 * to update the default radii of the underlying MobilizedBody::Ellipsoid
 *
 * @param Vec3 of radii: X, Y, Z in the parent frame.
 */
void EllipsoidJoint::setEllipsoidRadii(const Vec3& radii)
{
    set_radii_x_y_z(radii);
}

//==============================================================================
// SCALING
//==============================================================================
void EllipsoidJoint::
extendScale(const SimTK::State& s, const ScaleSet& scaleSet)
{
    Super::extendScale(s, scaleSet);

    // Get scale factors (if an entry for the parent Frame's base Body exists).
    const Vec3& scaleFactors = getScaleFactors(scaleSet, getParentFrame());
    if (scaleFactors == ModelComponent::InvalidScaleFactors)
        return;

    //TODO: Need to scale transforms appropriately, given an arbitrary axis.
    upd_radii_x_y_z() = get_radii_x_y_z().elementwiseMultiply(scaleFactors);
}

//=============================================================================
// Simbody Model building.
//=============================================================================
//_____________________________________________________________________________
void EllipsoidJoint::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);
    // CREATE MOBILIZED BODY
    MobilizedBody::Ellipsoid mobod =
        createMobilizedBody<MobilizedBody::Ellipsoid>(system);
    mobod.setDefaultRadii(get_radii_x_y_z());
}

void EllipsoidJoint::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);

    const SimbodyMatterSubsystem& matter = getModel().getMatterSubsystem();
    
    if (!matter.getUseEulerAngles(s)){
        double xangle = getCoordinate(EllipsoidJoint::Coord::Rotation1X).getDefaultValue();
        double yangle = getCoordinate(EllipsoidJoint::Coord::Rotation2Y).getDefaultValue();
        double zangle = getCoordinate(EllipsoidJoint::Coord::Rotation3Z).getDefaultValue();
        Rotation r(BodyRotationSequence, xangle, XAxis, 
                                         yangle, YAxis, zangle, ZAxis);

        //EllipsoidJoint* mutableThis = const_cast<EllipsoidJoint*>(this);
        getChildFrame().getMobilizedBody().setQToFitRotation(s, r);
    }
}

void EllipsoidJoint::extendSetPropertiesFromState(const SimTK::State& state)
{
    Super::extendSetPropertiesFromState(state);

    // Override default in case of quaternions.
    const SimbodyMatterSubsystem& matter = getModel().getMatterSubsystem();
    if (!matter.getUseEulerAngles(state)) {

        Rotation r = getChildFrame().getMobilizedBody().getBodyRotation(state);
        Vec3 angles = r.convertRotationToBodyFixedXYZ();

        updCoordinate(EllipsoidJoint::Coord::Rotation1X).setDefaultValue(angles[0]);
        updCoordinate(EllipsoidJoint::Coord::Rotation2Y).setDefaultValue(angles[1]);
        updCoordinate(EllipsoidJoint::Coord::Rotation3Z).setDefaultValue(angles[2]);
    }
}

void EllipsoidJoint::generateDecorations
       (bool                                        fixed, 
        const ModelDisplayHints&                    hints,
        const SimTK::State&                         state,
        SimTK::Array_<SimTK::DecorativeGeometry>&   geometryArray) const
{
    // invoke parent class method, this draws 2 Frames
    Super::generateDecorations(fixed,hints,state,geometryArray); 
    // The next line is necessary to avoid ellipsoid below added twice
    // since this method is called with fixed={true, false}
    if (!fixed) return; 
    // Construct the visible Ellipsoid
    SimTK::DecorativeEllipsoid ellipsoid(get_radii_x_y_z());
    const OpenSim::PhysicalFrame& frame = getParentFrame();
    ellipsoid.setColor(Vec3(0.0, 1.0, 1.0));
    ellipsoid.setBodyId(frame.getMobilizedBodyIndex());
    ellipsoid.setTransform(frame.findTransformInBaseFrame());
    geometryArray.push_back(ellipsoid);
}
