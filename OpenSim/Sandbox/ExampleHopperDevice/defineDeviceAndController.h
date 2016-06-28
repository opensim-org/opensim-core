#ifndef _defineDeviceAndController_h_
#define _defineDeviceAndController_h_
/* -------------------------------------------------------------------------- *
 *                   OpenSim:  defineDeviceAndController.h                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
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

/* This file defines two classes we need to complete the example: Device (a type
of ModelComponent) and PropMyoController (a type of Controller). We will
ultimately be attaching a Device to our hopper model (to help it hop higher); a
PropMyoController will provide a control signal for the Device.

Several lines of code need to be added to this file; see exampleHopperDevice.cpp
and the "TODO" comments below for instructions. */

#include <OpenSim/OpenSim.h>

namespace OpenSim {


//------------------------------------------------------------------------------
// Device is a type of ModelComponent that contains all the parts comprising the
// assistive device model (a PathActuator plus bodies and joints for attaching
// the actuator to the hopper or testbed). Devices are built by buildDevice()
// (see buildDeviceModel.cpp).
// [Step 2, Task A]
//------------------------------------------------------------------------------
class Device : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(Device, Component);

public:
    // Outputs that report quantities in which we are interested.
    // The total length of the device.
    OpenSim_DECLARE_OUTPUT(length, double, getLength, SimTK::Stage::Position);

    // The lengthening speed of the device.
    //TODO: Add an output called "speed" (to report the PathActuator's
    //      lengthening speed).
	OpenSim_DECLARE_OUTPUT(speed, double, getSpeed, SimTK::Stage::Velocity);

    // The force transmitted by the device.
    //TODO: Add an output called "tension".

    // The power produced(+) or dissipated(-) by the device.
    //TODO: Add an output called "power".

    // The height of the model to which the device is attached.
    //TODO: Add an output called "height".

    // The center of mass height of the model to which the device is attached.
    //TODO: Add an output called "com_height".

    // Member functions that access quantities in which we are interested. These
    // methods are used by the outputs declared above.
    double getLength(const SimTK::State& s) const {
        return getComponent<PathActuator>("cableAtoB").getLength(s);
    }
    double getSpeed(const SimTK::State& s) const {
        //TODO
        return getComponent<PathActuator>("cableAtoB").getLengtheningSpeed(s);
    }
    double getTension(const SimTK::State& s) const {
        //TODO
        return getComponent<PathActuator>("cableAtoB").computeActuation(s);
    }
    double getPower(const SimTK::State& s) const {
        //TODO
        return getComponent<PathActuator>("cableAtoB").getPower(s);
    }
    double getHeight(const SimTK::State& s) const {
        //TODO: Provide the name of the output corresponding to the hopper's
        //      height. You found this in Step 1, Task A.
        const std::string hopperHeightOutput = "/Dennis/slider/slider/height"; //fill this in

        //TODO: Use "getModel().getOutputValue<?????>(?????);" to return the
        //      output indicated by hopperHeightOutput.
		return getModel().getOutputValue<double>(s, hopperHeightOutput);
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

}; // end of Device


//------------------------------------------------------------------------------
// PropMyoController is a type of Controller that produces a control signal k*a,
// where 'k' is the gain property and 'a' is the activation input. This
// Controller is intended to simulate a proportional myoelectric controller [1],
// and can control any ScalarActuator (set using the "actuator" connector).
// [1] https://en.wikipedia.org/wiki/Proportional_myoelectric_control
// [Step 2, Task B]
//------------------------------------------------------------------------------
class PropMyoController : public Controller {
    OpenSim_DECLARE_CONCRETE_OBJECT(PropMyoController, Controller);

public:
    // Property of the controller for converting muscle activation into a
    // control signal.
    OpenSim_DECLARE_PROPERTY(gain, double,
        "Gain used to convert muscle activation into a control signal");

    // Connector to the ScalarActuator for which the controller is computing a
    // control signal.
    //TODO: Add a connector called "actuator" for connecting the ScalarActuator.
	OpenSim_DECLARE_CONNECTOR(actuator, ScalarActuator, 
		"The actuator for which the controller is computing a control signal");

    // Input the activation signal 'a' to which the controller's output should
    // be proportional.
    OpenSim_DECLARE_INPUT(activation, double, SimTK::Stage::Model,
        "The signal to which the controller's output is proportional");

    // Output the control signal generated by the controller.
    OpenSim_DECLARE_OUTPUT(myo_control, double, computeControl,
                           SimTK::Stage::Time);

    PropMyoController() {
        constructProperties();
    }

    // Member function for computing the proportional control signal k*a.
    double computeControl(const SimTK::State& s) const
    {
        double activation = getInputValue<double>(s, "activation");

        //TODO: Design a control strategy that improves jump height.
        return (activation < 0.31) ? 0. : get_gain() * activation;
    }

    // Member function for adding the control signal computed above into the
    // actuator's Vector of controls.
    void computeControls(const SimTK::State& s,
                         SimTK::Vector& controls) const override
    {
        double signal = computeControl(s);
        const auto& actuator = getConnectee<ScalarActuator>("actuator");
        SimTK::Vector thisActuatorsControls(1, signal);
        actuator.addInControls(thisActuatorsControls, controls);
    }

private:
    void constructProperties() {
        constructProperty_gain(1.0);
    }

}; // end of PropMyoController

} // end of namespace OpenSim

#endif // _defineDeviceAndController_h_
