/* -------------------------------------------------------------------------- *
 *                   OpenSim:  CantileverFreeBeamJoint.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2025 Stanford University and the Authors                *
 * Author(s): Nicholas Bianco                                                 *
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

 #include "CantileverFreeBeamJoint.h"
 #include <OpenSim/Simulation/Model/Model.h>
 #include "simbody/internal/MobilizedBody_CantileverFreeBeam.h"

 using namespace OpenSim;

//=============================================================================
// CONSTRUCTORS
//=============================================================================
CantileverFreeBeamJoint::CantileverFreeBeamJoint() : Super() {
    constructProperties();
}

CantileverFreeBeamJoint::CantileverFreeBeamJoint(const std::string& name,
        const PhysicalFrame& parent, const PhysicalFrame& child,
        const SimTK::Real& beamLength) : Joint(name, parent, child) {
    constructProperties();
    set_beam_length(beamLength);
}

CantileverFreeBeamJoint::CantileverFreeBeamJoint(const std::string& name,
        const PhysicalFrame& parent, const SimTK::Vec3& locationInParent,
        const SimTK::Vec3&  orientationInParent, const PhysicalFrame& child,
        const SimTK::Vec3& locationInChild,
        const SimTK::Vec3& orientationInChild, const SimTK::Real& beamLength) :
            Joint(name, parent, locationInParent, orientationInParent, child,
                  locationInChild, orientationInChild) {
    constructProperties();
    set_beam_length(beamLength);
}

void CantileverFreeBeamJoint::constructProperties() {
    constructProperty_beam_length(1.0);
}

//=============================================================================
// MODEL COMPONENT INTERFACE
//=============================================================================
void CantileverFreeBeamJoint::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    SimTK::MobilizedBody::CantileverFreeBeam mobod =
        createMobilizedBody<SimTK::MobilizedBody::CantileverFreeBeam>(system);
    mobod.setDefaultLength(get_beam_length());
}

//=============================================================================
// COMPONENT INTERFACE
//=============================================================================
void CantileverFreeBeamJoint::generateDecorations(bool fixed,
        const ModelDisplayHints& hints,
        const SimTK::State& state,
        SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const {
    Super::generateDecorations(fixed, hints, state, geometry);
    if (fixed) return;

    const SimTK::Real L = get_beam_length();
    const SimTK::Real L2 = L*L;
    const SimTK::Real L4 = L2*L2;

    auto calcBeamDeflection = [L, L2](const SimTK::Real& angle,
            const SimTK::Real& z) -> SimTK::Real {
        return angle * (z*z*(3.0*L - z))/(3.0*L2);
    };
    auto calcBeamDisplacement = [L, L2, L4](const SimTK::Real& angle,
            const SimTK::Real& z) -> SimTK::Real {
        return -(angle*angle * z*z*z *(20.0*L2 - 15.0*L*z + 3.0*z*z)) /
                (30.0*L4);
    };

    // Retrieve the rotation angles.
    const SimTK::MobilizedBody& mb = getChildFrame().getMobilizedBody();
    const SimTK::Transform& X_FM = mb.getMobilizerTransform(state);
    const SimTK::Rotation& R = X_FM.R();
    SimTK::Vec3 angles = R.convertRotationToBodyFixedXYZ();

    // Calculate the first point.
    const SimTK::Transform& X_BM = mb.getOutboardFrame(state);
    const SimTK::Transform& X_GB = mb.getBodyTransform(state);
    SimTK::Transform X_GF = X_GB * X_BM * ~X_FM;
    SimTK::Vec3 prevPoint_G = X_GF.p();

    // Calculate points along the beam and draw lines between them.
    for (int i = 0; i < 50 +  1; i++) {
        SimTK::Real z = i * L / 50;
        SimTK::Real d_y = -calcBeamDeflection(angles[0], z);
        SimTK::Real d_x = calcBeamDeflection(angles[1], z);
        SimTK::Real d_z = calcBeamDisplacement(angles[0], z) +
                    calcBeamDisplacement(angles[1], z);
        SimTK::Vec3 nextPoint_F(d_x, d_y, z + d_z);
        SimTK::Vec3 nextPoint_G = X_GF * nextPoint_F;
        geometry.push_back(
            SimTK::DecorativeLine(prevPoint_G, nextPoint_G)
                .setColor(SimTK::Vec3(1.0, 0.0, 0.0))
                .setLineThickness(4)
        );

        prevPoint_G = nextPoint_G;
    }
}
