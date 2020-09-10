#ifndef OPENSIM_ACTIVATION_COORDINATE_ACTUATOR_H
#define OPENSIM_ACTIVATION_COORDINATE_ACTUATOR_H
/* -------------------------------------------------------------------------- *
 *                  OpenSim:  ActivationCoordinateActuator.h                  *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2020 Stanford University and the Authors                *
 * Author(s): Christopher Dembia                                              *
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

#include "osimActuatorsDLL.h"

#include "CoordinateActuator.h"

namespace OpenSim {

/**
Similar to CoordinateActuator (simply produces a generalized force) but
with first-order linear activation dynamics. This actuator has one state
variable, `activation`, with \f$ \dot{a} = (x - a) / \tau \f$, where
\f$ a \f$ is activation, \f$ x \f$ is excitation, and \f$ \tau \f$ is the
activation time constant (there is no separate deactivation time constant).
The statebounds_activation output is used in Moco to set default values for
the activation state variable.
<b>Default %Property Values</b>
@verbatim
activation_time_constant: 0.01
default_activation: 0.5
@endverbatim
 */
class OSIMACTUATORS_API ActivationCoordinateActuator
        : public CoordinateActuator {
    OpenSim_DECLARE_CONCRETE_OBJECT(ActivationCoordinateActuator,
        CoordinateActuator);
public:
    OpenSim_DECLARE_PROPERTY(activation_time_constant, double,
        "Larger value means activation changes more slowly "
        "(units: seconds; default: 0.01 seconds).");

    OpenSim_DECLARE_PROPERTY(default_activation, double,
        "Value of activation in the default state (default: 0.5).");

    OpenSim_DECLARE_OUTPUT(statebounds_activation, SimTK::Vec2,
        getBoundsActivation, SimTK::Stage::Model);

    ActivationCoordinateActuator() {
        constructProperties();
    }

    /// Provide the coordinate name.
    explicit ActivationCoordinateActuator(const std::string& coordinateName)
            : ActivationCoordinateActuator() {
        if (!coordinateName.empty()) {
            set_coordinate(coordinateName);
        }
    }

    /// The lower bound on activation is getMinControl() and the upper bound is
    /// getMaxControl().
    /// Whether these bounds are enforced is determined by the solver used.
    SimTK::Vec2 getBoundsActivation(const SimTK::State&) const {
        return SimTK::Vec2(getMinControl(), getMaxControl());
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

    void computeStateVariableDerivatives(const SimTK::State& s) const override {
        // No need to do clamping, etc; CoordinateActuator is bidirectional.
        const auto& tau = get_activation_time_constant();
        const auto& x = getControl(s);
        const auto& a = getStateVariableValue(s, "activation");
        const SimTK::Real adot = (x - a) / tau;
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

#endif // OPENSIM_ACTIVATION_COORDINATE_ACTUATOR_H
