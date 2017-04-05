#ifndef _OPENSIM_HOPPERDEVICE_H_
#define _OPENSIM_HOPPERDEVICE_H_
/* -------------------------------------------------------------------------- *
 *               OpenSim:  HopperDevice.h                                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Chris Dembia, Shrinidhi K. Lakshmikanth, Ajay Seth,             *
 *            Thomas Uchida                                                   *
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

#include "osimExampleComponentsDLL.h"
#include <OpenSim/Simulation/Model/PathActuator.h>
#include <OpenSim/Simulation/Model/Actuator.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim {

//------------------------------------------------------------------------------
// HopperDevice is a type of ModelComponent that contains all the parts
// comprising a assistive device model (a PathActuator plus bodies and joints
// for attaching the actuator to the hopper or testbed). Devices are built by
// buildDevice() (see buildDeviceModel.cpp).
// This class is written to be used with Hopper example and is not generic
// to be used elsewhere.
//------------------------------------------------------------------------------
class OSIMEXAMPLECOMPONENTS_API HopperDevice : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(HopperDevice, ModelComponent);

public:
    // Outputs that report quantities in which we are interested.
    // The total length of the device.
    OpenSim_DECLARE_OUTPUT(length, double, getLength, SimTK::Stage::Position);

    // The lengthening speed of the device.
    OpenSim_DECLARE_OUTPUT(speed, double, getSpeed, SimTK::Stage::Velocity);

    // The force transmitted by the device.
    OpenSim_DECLARE_OUTPUT(tension, double, getTension, SimTK::Stage::Dynamics);

    // The power produced(+) or dissipated(-) by the device.
    OpenSim_DECLARE_OUTPUT(power, double, getPower, SimTK::Stage::Dynamics);

    // The height of the model to which the device is attached.
    OpenSim_DECLARE_OUTPUT(height, double, getHeight, SimTK::Stage::Position);

    // The center of mass height of the model to which the device is attached.
    OpenSim_DECLARE_OUTPUT(com_height, double, getCenterOfMassHeight,
                           SimTK::Stage::Position);

    // Member functions that access quantities in which we are interested. These
    // methods are used by the outputs declared above.
    double getLength(const SimTK::State& s) const {
        return getComponent<PathActuator>("cableAtoB").getLength(s);
    }
    double getSpeed(const SimTK::State& s) const {
        return getComponent<PathActuator>("cableAtoB").getLengtheningSpeed(s);
    }
    double getTension(const SimTK::State& s) const {
        return getComponent<PathActuator>("cableAtoB").computeActuation(s);
    }
    double getPower(const SimTK::State& s) const {
        return getComponent<PathActuator>("cableAtoB").getPower(s);
    }
    double getHeight(const SimTK::State& s) const {
        static const std::string hopperHeightCoord = "/Dennis/slider/yCoord";
        return getModel().getComponent(hopperHeightCoord)
            .getOutputValue<double>(s, "value");
    }
    double getCenterOfMassHeight(const SimTK::State& s) const {
        SimTK::Vec3 com_position = getModel().calcMassCenterPosition(s);
        return com_position[SimTK::YAxis];
    }

protected:
    // Change the color of the device's path as its tension changes.
    void extendRealizeDynamics(const SimTK::State& s) const override {
        const auto& actuator = getComponent<PathActuator>("cableAtoB");
        double level = fmin(1., getTension(s) / actuator.get_optimal_force());
        actuator.getGeometryPath().setColor(s, SimTK::Vec3(0.1, level, 0.1));
    }

}; // end of HopperDevice

} // namespace OpenSim
    
#endif // _OPENSIM_HOPPERDEVICE_H_
