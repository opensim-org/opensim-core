#ifndef OPENSIM_DELAY_H_
#define OPENSIM_DELAY_H_
/* -------------------------------------------------------------------------- *
 *                             OpenSim: Delay.h                               *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
 * Author(s): Chris Dembia                                                    *
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

#include "osimCommonDLL.h"
#include "Component.h"

namespace OpenSim {

/** This operator delays its input by a given time delay. The value of the
 * output at time `t` is the value the input had at time `t-delay`.
 * A Delay can be used to model the delay in neural reflex circuits.
 *
 * This Component has a single input named "input" and a single output named
 * "output". The typical way to use this Component is as a subcomponent in
 * another component.
 *
 * For times prior to the start of a simulation this Component behaves as
 * though the input value had been constant at its initial value.
 *
 * A delay duration of 0 just passes through the value
 * of the input and introduces minimal computational overhead.
 *
 * @tparam T The type of the quantity to be delayed. Common choices are a
 * SimTK::Real (see Delay) or a SimTK::Vector (see DelayVector).
 *
 * @see Delay, DelayVector
 *
 * ### Accuracy
 * The delayed quantity is obtained by linearly interpolating between past
 * successful time steps. The accuracy of the output of the delay is not
 * controlled by the integrators, and you must make sure the
 * delay is sufficiently accurate for your use case.
 * Smaller time steps improve the accuracy of the output; the maximum time
 * step should be significantly smaller than the delay duration.
 * See SimTK::Integrator::setMaximumStepSize.
 *
 * The delayed quantity will be exactly correct if there was a time
 * step at exactly `t - delay`; you can obtain an accurate delayed output
 * if use a fixed step integrator whose with a delay duration that is
 * a multiple of the fixed step size.
 *
 * ### Discontinuous input
 * If the input to the delay is discontinuous,
 * the output of the delay is likely to be grossly inaccurate around the
 * discontinuity (as a result of linear interpolation).
 * You can remedy this problem in the case that your discontinuity is
 * piecewise constant by using an event trigger to trigger time steps
 * at the time of the discontinuity.
 * For example, if you have a step input at 0.5 seconds, use an
 * event trigger to force a time step at 0.5 seconds.
 *
 * ### Example
 * Here is a basic example in which a controller specifies
 * the control signal for a single actuator using the delayed value of a
 * coordinate:
 * @code
 * using namespace OpenSim;
 * using namespace SimTK;
 * class MyController : Controller {
 * OpenSim_DECLARE_CONCRETE_OBJECT(MyController, Controller);
 * public:
 *     OpenSim_DECLARE_PROPERTY(delay, double, "Duration of delay (seconds).");
 *     MyController() { constructProperties(); }
 *     void computeControls(const State& s, Vector& controls) const override {
 *         double q = _coordDelay.getValue(s);
 *         // Delayed negative feedback control law.
 *         double u = -10 * q;
 *         getModel().getActuatorSet().get("my_actuator").addInControls(
 *             Vector(1, u), controls);
 *     }
 * private:
 *     void constructProperties() {
 *          constructProperty_delay(0.01); // 10 milliseconds
 *     }
 *     void extendFinalizeFromProperties() override {
 *         Super::extendFinalizeFromProperties();
 *         _coordDelay.set_delay(get_delay());
 *         addComponent(&_coordDelay);
 *     }
 *     void extendConnectToModel(Model& model) override {
 *         Super::extendConnectToModel(model);
 *         const auto& coord = model.getCoordinateSet().get("ankle_angle");
 *         // Set the input for the Delay component.
 *         _coordDelay.getInput("input").connect(coord.getOutput("value"));
 *     }
 *
 *     Delay _coordDelay;
 * };
 * @endcode
 *
 * This class is implemented via a SimTK::Measure_::Delay; see its
 * documentation for more information.
 *
 * @ingroup operators
 *
 */
template<typename T>
class Delay_ : public Component {
OpenSim_DECLARE_CONCRETE_OBJECT_T(Delay_, T, Component);
public:

    OpenSim_DECLARE_PROPERTY(delay, double,
        "The duration (nonnegative, seconds) by which the output is delayed.");
    OpenSim_DECLARE_PROPERTY(can_use_current_value, bool,
        "(Advanced) Use current value of input to compute output.");

    /// Default constructor.
    Delay_();

    /// Convenience constructor that sets the delay property.
    explicit Delay_(double delay);

    /// Get the delayed value (the input's value at time t-delay).
    T getValue(const SimTK::State& s) const;

private:

    void constructProperties() override;
    void constructInputs() override;
    void constructOutputs() override;

    void extendFinalizeFromProperties() override;
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    typename SimTK::Measure_<T>::Delay m_delayMeasure;
};

template<class T>
Delay_<T>::Delay_() {
    constructInfrastructure();
}

template<class T>
Delay_<T>::Delay_(double delay) : Delay_() {
    set_delay(delay);
}

template<class T>
void Delay_<T>::constructProperties() {
    constructProperty_delay(0.0);
    constructProperty_can_use_current_value(false);
}

template<class T>
void Delay_<T>::constructInputs() {
    // TODO the requiredAt Stage should be the same as this Delay's output dependsOn stage.
    constructInput<T>("input", SimTK::Stage::Time);
}

template<class T>
void Delay_<T>::constructOutputs() {
    // TODO the depensdOn stage should be the dependsOn stage of the output
    // that is wired to this Delay's input.
    constructOutput<T>("output", &Delay_::getValue, SimTK::Stage::Time);
}

template<class T>
T Delay_<T>::getValue(const SimTK::State& s) const {
    if (get_delay() > 0) {
        return m_delayMeasure.getValue(s);
    } else {
        return getInputValue<T>(s, "input");
    }
}

template<class T>
void Delay_<T>::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();
    SimTK_VALUECHECK_NONNEG_ALWAYS(get_delay(),
            "delay", "Delay::extendFinalizeFromProperties()");
}

template<class T>
void Delay_<T>::extendAddToSystem(SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    if (get_delay() > 0) {
        // This is necessary because Simbody does not currently handle
        // a delay of 0 properly. See Simbody GitHub issue #360.
        auto &sub = system.updDefaultSubsystem();
        const auto &input = *static_cast<const Input<T> *>(&getInput("input"));
        const_cast<Delay_ *>(this)->m_delayMeasure =
                typename SimTK::Measure_<T>::Delay(sub,
                                                   InputMeasure<T>(sub, input),
                                                   get_delay());
        const_cast<Delay_ *>(this)->m_delayMeasure.setCanUseCurrentValue(
                get_can_use_current_value());

    }
}

/** A Delay component that allows one to delay the value of a scalar quantity.
 * This is the most common type of Delay. See Delay_ for detailed information.
 */
typedef Delay_<SimTK::Real> Delay;

/** A Delay component that allows one to delay the value of a SimTK::Vector.
 * For example, you could use this DelayVector to delay the value of
 * the Model's generalized coordinates. See Delay_ for detailed information.
 */
typedef Delay_<SimTK::Vector> DelayVector;

} // namespace OpenSim

#endif // OPENSIM_DELAY_H_
