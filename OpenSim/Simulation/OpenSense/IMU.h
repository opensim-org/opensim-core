#ifndef OPENSIM_IMU_H_
#define OPENSIM_IMU_H_
/* -------------------------------------------------------------------------- *
*                        OpenSim:           IMU.h                            *
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
#include <OpenSim/Simulation/Model/PhysicalFrame.h>


namespace OpenSim {
//=============================================================================
//                               IMU
//=============================================================================
/**
IMU is a Model Component that represents a virtual IMU along with its Geometry
for visualization. This component reports angular velocity and linear
acceleration (with gravitational acceleration subtracted) vectors expressed in
the IMU frame to mimic the behavior of real-world IMUs. It also reports
orientations between the IMU and ground frame as Quaternions, expressed in the
ground frame.

@todo add noise model, limits/saturation, as needed.

@authors Ayman Habib
**/
class OSIMSIMULATION_API IMU : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(IMU, ModelComponent);

public:
    IMU() = default;
    virtual ~IMU() = default;
    IMU(const IMU&) = default;
    IMU(IMU&&) = default;
    IMU& operator=(const IMU&) = default;
    IMU& operator=(IMU&&) = default;

    // Attachment frame for placement/visualization
    OpenSim_DECLARE_SOCKET(
            frame, PhysicalFrame, "The frame to which the IMU is attached.");

    OpenSim_DECLARE_OUTPUT(orientation_as_quaternion, SimTK::Quaternion,
            calcOrientationAsQuaternion,
            SimTK::Stage::Position);
    OpenSim_DECLARE_OUTPUT(gyroscope_signal, SimTK::Vec3,
            calcGyroscopeSignal, SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(accelerometer_signal, SimTK::Vec3,
            calcAccelerometerSignal, SimTK::Stage::Acceleration);
    // Outputs
    /// Report the Transform of this IMU in the Ground frame.
    /// @note Requires realizing the State to SimTK::Stage::Position.
    SimTK::Transform calcTransformInGround(const SimTK::State& s) const {
        return get_frame().getTransformInGround(s);
    }
    /// Report the orientation of this IMU in ground frame expressed as a
    /// Quaternion.
    /// @note Requires realizing the State to SimTK::Stage::Position.
    SimTK::Quaternion calcOrientationAsQuaternion(const SimTK::State& s) const {
        return SimTK::Quaternion(calcTransformInGround(s).R());
    }
    /// Report the angular velocity of this IMU in the frame it is attached to.
    /// @note Requires realizing the State to SimTK::Stage::Velocity.
    SimTK::Vec3 calcGyroscopeSignal(const SimTK::State& s) const {
        const auto& model = getModel();
        const auto& ground = model.getGround();
        return ground.expressVectorInAnotherFrame(s,
            get_frame().getAngularVelocityInGround(s), get_frame());
    }
    /// Report the linear acceleration of the frame to which this IMU is
    /// attached in the Ground grame. Gravity is subtracted and result expressed
    /// in the frame to which the IMU is attached.
    /// @note Requires realizing the State to SimTK::Stage::Acceleration.
    SimTK::Vec3 calcAccelerometerSignal(
            const SimTK::State& s) const {
        const auto& model = getModel();
        const auto& ground = model.getGround();
        const auto& gravity = model.getGravity();
        SimTK::Vec3 linearAcceleration =
                get_frame().getLinearAccelerationInGround(s);
        return ground.expressVectorInAnotherFrame(
                s, linearAcceleration - gravity, get_frame());
    }
    void generateDecorations(bool fixed, const ModelDisplayHints& hints,
        const SimTK::State& state,
        SimTK::Array_<SimTK::DecorativeGeometry>& appendToThis)
        const override {
        if (!fixed) return;

        // @TODO default color, size, shape should be obtained from hints
        const OpenSim::PhysicalFrame& physFrame = this->get_frame();
        SimTK::Transform relativeXform = physFrame.findTransformInBaseFrame();
        appendToThis.push_back(
                SimTK::DecorativeBrick(SimTK::Vec3(0.02, 0.01, 0.005))
                        .setBodyId(physFrame.getMobilizedBodyIndex())
                                        .setColor(SimTK::Orange)
                        .setTransform(relativeXform));
    }

private:
 
    const PhysicalFrame& get_frame() const {
        return getSocket<PhysicalFrame>("frame").getConnectee();
    }
}; // End of class IMU

}
#endif // OPENSIM_IMU_H_
