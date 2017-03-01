#ifndef OPENSIM_SIMULATION_COMPONENTS_FOR_TESTING_H_
#define OPENSIM_SIMULATION_COMPONENTS_FOR_TESTING_H_

/* -------------------------------------------------------------------------- *
 *                OpenSim: SimulationComponentsForTesting.h                   *
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

/** This file contains simulation-related Components that may be used by
 * multiple tests in this directory and its subdirectories, and that exist only
 * for use in tests. */

#include <OpenSim/Simulation/osimSimulation.h>
#include <simbody/internal/MobilizedBody_BuiltIns.h>

using namespace OpenSim;
using namespace std;

//==============================================================================
// CompoundJoint necessary for testing equivalent kinematics and body force
// calculations for joints comprised of more than one mobilized body.
//==============================================================================
class CompoundJoint : public Joint {
OpenSim_DECLARE_CONCRETE_OBJECT(CompoundJoint, Joint);

public:
    /** Indices of Coordinates. */
    enum class Coord : unsigned {
        Rotation1X,
        Rotation2Y,
        Rotation3Z
    };

private:
    /** Specify the Coordinates of this CompoundJoint */
    CoordinateIndex rx{ constructCoordinate(Coordinate::MotionType::Rotational,
                                   static_cast<unsigned>(Coord::Rotation1X)) };
    CoordinateIndex ry{ constructCoordinate(Coordinate::MotionType::Rotational,
                                   static_cast<unsigned>(Coord::Rotation2Y)) };
    CoordinateIndex rz{ constructCoordinate(Coordinate::MotionType::Rotational,
                                   static_cast<unsigned>(Coord::Rotation3Z)) };

public:
    // CONSTRUCTION
    using Joint::Joint;

protected:
    void extendAddToSystem(SimTK::MultibodySystem& system) const override
    {
        using namespace SimTK;

        Super::extendAddToSystem(system);

        // PARENT TRANSFORM
        const SimTK::Transform& P_Po =
            getParentFrame().findTransformInBaseFrame();
        // CHILD TRANSFORM
        const SimTK::Transform& B_Bo =
            getChildFrame().findTransformInBaseFrame();

        int coordinateIndexForMobility = 0;

        SimTK::Transform childTransform0(Rotation(), Vec3(0));

        SimTK::Body::Massless massless;

        // CREATE MOBILIZED BODY for body rotation about body Z
        MobilizedBody masslessBody1 = createMobilizedBody<MobilizedBody::Pin>(
            system.updMatterSubsystem().updMobilizedBody(
                getParentFrame().getMobilizedBodyIndex()),
            P_Po,
            massless,
            childTransform0,
            coordinateIndexForMobility);

        // Find the joint frame with Z aligned to body X
        Rotation rotToX(Pi/2, YAxis);
        SimTK::Transform parentTransform1(rotToX, Vec3(0));
        SimTK::Transform childTransform1(rotToX, Vec3(0));

        // CREATE MOBILIZED BODY for body rotation about body X
        MobilizedBody masslessBody2 = createMobilizedBody<MobilizedBody::Pin>(
            masslessBody1,
            parentTransform1,
            massless,
            childTransform1,
            coordinateIndexForMobility);

        // Now Find the joint frame with Z aligned to body Y
        Rotation rotToY(-Pi/2, XAxis);
        SimTK::Transform parentTransform2(rotToY, Vec3(0));
        SimTK::Transform childTransform2(B_Bo.R()*rotToY, B_Bo.p());

        // CREATE MOBILIZED BODY for body rotation about body Y
        MobilizedBody mobBod = createMobilizedBody<MobilizedBody::Pin>(
            masslessBody2,
            parentTransform2,
            getChildInternalRigidBody(),
            childTransform2,
            coordinateIndexForMobility, &getChildFrame());
    }

}; // end of CompoundJoint

#endif // OPENSIM_SIMULATION_COMPONENTS_FOR_TESTING_H_
