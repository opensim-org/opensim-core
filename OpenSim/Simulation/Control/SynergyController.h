#ifndef OPENSIM_SYNERGYCONTROLLER_H
#define OPENSIM_SYNERGYCONTROLLER_H
/* -------------------------------------------------------------------------- *
 * OpenSim: SynergyController.h                                               *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2020 Stanford University and the Authors                     *
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

#include <OpenSim/Simulation/osimSimulationDLL.h>

#include <OpenSim/Simulation/Control/Controller.h>
#include <OpenSim/Simulation/Model/Actuator.h>

namespace OpenSim {

// TODO how to handle stage?
template <typename T>
class Concatenator_ : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT_T(Concatenator_, T, Component);
public:
    OpenSim_DECLARE_LIST_INPUT(inputs, T, SimTK::Stage::Dynamics, "TODO");
    OpenSim_DECLARE_OUTPUT(output, SimTK::Vector_<T>, getConcatenatedVector,
            SimTK::Stage::Dynamics);

    SimTK::Vector_<T> getConcatenatedVector(const SimTK::State& s) const {
        const auto& input = this->template getInput<T>("inputs");
        const int numChannels = input.getNumConnectees();
        SimTK::Vector_<T> output(numChannels);
        for (int ichan = 0; ichan < numChannels; ++ichan) {
            output[ichan] = input.getChannel(ichan).getValue(s);
        }
        return output;
    }
};

using Concatenator = Concatenator_<double>;

class OSIMSIMULATION_API SynergyWeightsVector : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(SynergyWeightsVector, Object);
public:
    OpenSim_DECLARE_PROPERTY(weights, SimTK::Vector, "TODO");
    SynergyWeightsVector() {
        constructProperty_weights(SimTK::Vector());
    }
    SynergyWeightsVector(std::string actuatorPath,
            SimTK::Vector v) : SynergyWeightsVector() {
        setName(std::move(actuatorPath));
        set_weights(std::move(v));
    }
};

class OSIMSIMULATION_API SynergyController : public Controller {
    OpenSim_DECLARE_CONCRETE_OBJECT(SynergyController, Controller);
public:
    OpenSim_DECLARE_LIST_PROPERTY(synergy_weights, SynergyWeightsVector,
            "TODO");
    // TODO LIST_INPUT?
    OpenSim_DECLARE_INPUT(synergy_controls, SimTK::Vector,
            SimTK::Stage::Dynamics,
            "The synergy control signals ordered TODO.");

    SynergyController();

    void setSynergyWeights(std::string actuator, SimTK::Vector weights);

    void computeControls(
            const SimTK::State& s, SimTK::Vector& controls) const override;
protected:
    void extendFinalizeFromProperties() override;

private:
    void constructProperties();

    SimTK::Matrix m_synergyWeights;
};

} // namespace OpenSim

#endif // OPENSIM_SYNERGYCONTROLLER_H
