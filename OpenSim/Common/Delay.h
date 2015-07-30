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
 * output at time t is the value the input had at time t-delay.
 * A Delay can be used to model the delay in neural reflex circuits.
 *
 * For times prior to the start of a simulation this Component behaves as
 * though the input value had been constant at its initial value.
 *
 * @note The output will be incorrect if the delay is smaller than the
 * integrator's accuracy setting. Make sure that the integrator's accuracy
 * is tighter than your delay duration (e.g., accuracy of 1e-3 or less for a
 * delay of 1e-3 seconds).
 * Also, a delay of 0 currently does not produce the correct behavior;
 * this is a known bug.
 *
 * @tparam T The type of the quantity to be delayed. Common choices are a
 * SimTK::Real (see Delay) or a SimTK::Vector (see DelayVector).
 *
 * @see Delay, DelayVector
 *
 *
 * This Component has a single input named "input" and a single output named
 * "output". The typical way to use this Component is as a subcomponent in
 * another component. Here is a basic example in which a controller specifies
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
 *     void computeControls(const State& s, Vector &controls) const override {
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
 *         const auto& coord = model.getCoordinateSet()[0];
 *         // Set the input for the Delay component.
 *         _coordDelay.getInput("input").connect(coord.getOutput("value"));
 *     }
 *
 *     Delay _coordDelay;
 * };
 * @endcode
 *
 * This class is implemented via a SimTK::Measure_<T>::Delay.
 *
 * @ingroup operators
 *
 */
template<typename T>
class Delay_ : public Component {
OpenSim_DECLARE_CONCRETE_OBJECT_T(Delay_, T, Component);
public:

    /** @name Property declarations
    These are the serializable properties associated with a Delay. **/
    /**@{**/
    OpenSim_DECLARE_PROPERTY(delay, double,
        "The duration (nonnegative, seconds) by which the output is delayed.");
    /**@}**/

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
    return m_delayMeasure.getValue(s);
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
    auto& sub = system.updDefaultSubsystem();
    const auto& input = *static_cast<const Input<T>*>(&getInput("input"));
    const_cast<Delay_*>(this)->m_delayMeasure =
            typename SimTK::Measure_<T>::Delay(sub,
                                               InputMeasure<T>(sub, input),
                                               get_delay());
}

/** A Delay component that allows one to delay the value of a scalar quantity.
 * This is the most common type of Delay. See Delay_ for detailed information.
 *
 */
using Delay = Delay_<SimTK::Real>;

/** A Delay component that allows one to delay the value of a SimTK::Vector.
 * For example, you could use this DelayVector to delay the value of
 * the Model's generalized coordinates. See Delay_ for detailed information.
 */
using DelayVector = Delay_<SimTK::Vector>;

} // namespace OpenSim

#endif // OPENSIM_DELAY_H_
