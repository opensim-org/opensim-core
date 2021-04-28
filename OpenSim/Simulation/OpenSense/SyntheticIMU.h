#ifndef OPENSIM_SYNTHETIC_IMU_H_
#define OPENSIM_SYNTHETIC_IMU_H_
/* -------------------------------------------------------------------------- *
*                        OpenSim:  SyntheticIMU.h                            *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2021 Stanford University and the Authors                *
* Author(s): Ayman Habib                                                     *
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

#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/OpenSense/ExperimentalFrame.h>


namespace OpenSim {
//=============================================================================
//                               SyntheticIMU
//=============================================================================
/**
SyntheticIMU is a Model Component that represents an IMU along with its Geometry
for visualization, noise model.


@authors Ayman Habib
**/
class OSIMSIMULATION_API SyntheticIMU : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(SyntheticIMU, ModelComponent);

public:
    SyntheticIMU() { constructProperties(); }
    // Attachment frame for placement/visualization
    OpenSim_DECLARE_PROPERTY(attachment_frame, ExperimentalFrame,
        "The frame used to attach and visualize the Synthetic IMU.");
    OpenSim_DECLARE_OUTPUT(transform, SimTK::Transform, calcTransformInGround,
            SimTK::Stage::Position);
    OpenSim_DECLARE_OUTPUT(angular_velocity, SimTK::Vec3,
            calcAngularVelocity, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(linear_acceleration, SimTK::Vec3,
            calcLinearAcceleration, SimTK::Stage::Dynamics);
    // Outputs
    SimTK::Transform calcTransformInGround(const SimTK::State& s) const {
        return get_attachment_frame().
                getInput<SimTK::Vec3>("transform_in_ground")
                        .getValue(s);
    }
    SimTK::Vec3 calcAngularVelocity(const SimTK::State& s) const {
        return get_attachment_frame().getAngularVelocityInGround(s);
    }
    SimTK::Vec3 calcLinearAcceleration(const SimTK::State& s) const {
        return get_attachment_frame().getLinearAccelerationInGround(s);
    }

private:
    void constructProperties() {
        constructProperty_attachment_frame(ExperimentalFrame());
    }

}; // End of class SyntheticIMU

}
#endif // OPENSIM_SYNTHETIC_IMU_H_