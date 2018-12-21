#ifndef MOCO_ACTIVATIONCOORDINATEACTUATOR_H
#define MOCO_ACTIVATIONCOORDINATEACTUATOR_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: ActivationCoordinateActuator.h                               *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Actuators/CoordinateActuator.h>

namespace OpenSim {

/// Similar to CoordinateActuator (simply produces a generalized force) but
/// with first-order linear activation dynamics. This actuator has one state
/// variable, `activation`, with \f$ \dot{a} = (u - a) / \tau \f$, where
/// \f$ a \f$ is activation, \f$ u \f$ is excitation, and \f$ \tau \f$ is the
/// activation time constant (there is no separate deactivation time constant).
/// <b>Default %Property Values</b>
/// @verbatim
/// activation_time_constant: 0.01
/// default_activation: 0.5
/// @endverbatim
class OSIMMOCO_API
    ActivationCoordinateActuator : public CoordinateActuator {
    OpenSim_DECLARE_CONCRETE_OBJECT(ActivationCoordinateActuator,
        CoordinateActuator);
public:
    OpenSim_DECLARE_PROPERTY(activation_time_constant, double,
        "Larger value means activation can change more rapidly "
        "(units: seconds).");

    OpenSim_DECLARE_PROPERTY(default_activation, double,
        "Value of activation in the default state returned by initSystem().");

    ActivationCoordinateActuator() {
        constructProperties();
    }

protected:
    void extendAddToSystem(SimTK::MultibodySystem& system) const override {
        Super::extendAddToSystem(system);
        addStateVariable("activation", SimTK::Stage::Dynamics);
    }

    void extendInitStateFromProperties(SimTK::State& s) const override {
        Super::extendInitStateFromProperties(s);
        setStateVariableValue(s, "activation", get_default_activation());
    }

    void extendSetPropertiesFromState(const SimTK::State& s) override {
        Super::extendSetPropertiesFromState(s);
        set_default_activation(getStateVariableValue(s, "activation"));
    }

    // TODO no need to do clamping, etc; CoordinateActuator is bidirectional.
    void computeStateVariableDerivatives(const SimTK::State& s) const override
    {
        const auto& tau = get_activation_time_constant();
        const auto& u = getControl(s);
        const auto& a = getStateVariableValue(s, "activation");
        const SimTK::Real adot = (u - a) / tau;
        setStateVariableDerivativeValue(s, "activation", adot);
    }

    double computeActuation(const SimTK::State& s) const override {
        return getStateVariableValue(s, "activation") * getOptimalForce();
    }
private:
    void constructProperties() {
        constructProperty_activation_time_constant(0.010);
        constructProperty_default_activation(0.5);
    }
};

} // namespace OpenSim

#endif // MOCO_ACTIVATIONCOORDINATEACTUATOR_H
