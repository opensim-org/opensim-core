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
 * though the source value had been constant at its initial value.
 *
 * This Component has a single input named "input" and a single output named
 * "output". The typical way to use this Component is as a subcomponent in
 * another component. Here is a basic example in which a controller specifies
 * the control signal for a single actuator using the delayed value of a
 * coordinate:
 *
 * @code
 * using namespace OpenSim;
 * using namespace SimTK;
 * class MyController : Controller {
 * OpenSim_DECLARE_CONCRETE_OBJECT(MyController, Controller);
 * public:
 *     void computeControls(const State& s, Vector &controls) const override {
 *         double q = _coordDelay.getOutput<double>(s, "output");
 *         // Delayed negative feedback control law.
 *         double u = -10 * q;
 *         getModel().getActuatorSet()[0].addInControls(Vector(1, u), controls);
 *     }
 * private:
 *     void extendConnectToModel(Model& model) override {
 *         Super::extendConnectToModel(model);
 *         const auto& coord = model.getCoordinateSet()[0];
 *         // Set the input for the Delay component.
 *         _coordDelay.getInput("input").connect(coord.getOutput("value"));
 *         _coordDelay.set_delay(0.01); // delay by 10 milliseconds.
 *         // Set the Delay as a subcomponent.
 *         addComponent(&_coordDelay);
 *     }
 *
 *     Delay _coordDelay;
 * };
 * @endcode
 *
 * The implementation makes use of a SimTK::Measure::Delay.
 *
 */
class OSIMCOMMON_API Delay : public Component {
OpenSim_DECLARE_CONCRETE_OBJECT(Delay, Component);
public:

    /** @name Property declarations
    These are the serializable properties associated with a Delay. **/
    /**@{**/
    OpenSim_DECLARE_PROPERTY(delay, double,
            "The time (in seconds) by which the output should be delayed.");
    /**@}**/

    Delay();

    /// Get the delayed value (the input's value at time t-delay).
    double getValue(const SimTK::State& s) const;

private:

    void constructProperties() override;
    void constructInputs() override;
    void constructOutputs() override;

    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    SimTK::MeasureIndex _delayMeasureIndex;
};

} // namespace OpenSim

#endif // OPENSIM_DELAY_H_
