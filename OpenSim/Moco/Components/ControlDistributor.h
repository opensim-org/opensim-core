#ifndef OPENSIM_CONTROL_DISTRIBUTOR_H
#define OPENSIM_CONTROL_DISTRIBUTOR_H
/* -------------------------------------------------------------------------- *
 *                      OpenSim: ControlDistributor.h                         *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2024 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
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

#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Moco/osimMocoDLL.h>

namespace OpenSim {

/**
 * This component stores a vector of control values that can be used to
 * allocate controls to other components in a model.
 *
 * A control value can be added to the ControlDistributor using `addControl()`.
 * This adds a channel to the Output `controls` which is then available to
 * distribute to other components via `Input` connections.
 *
 * The control values are stored in a discrete variable in the state. The
 * control values are stored in the same order they were added to the
 * distributor. The control values can be set and retrieved using the
 * `setControls()`, `updControls()`, and `getControls()` methods.
 *
 * This class is primarily intended for use in Moco where it is used to
 * distribute control values from an optimal control problem to InputControllers
 * in a model. Therefore, "controls" in the context of this class do not
 * necessarily correspond to controls in a model (i.e., from
 * Model::getControls()). However, this class is generally useful for
 * distributing scalar values stored in a discrete variable to other components
 * in a model that accept an `Input`.
 *
 * ## Usage
 *
 * Constructing a ControlDistributor and adding controls:
 * @code
 * auto controlDistributor = make_unique<ControlDistributor>();
 * controlDistributor->addControl("/forceset/soleus_r");
 * controlDistributor->addControl("/my_input_controller_value");
 * controlDistributor->addControl("/my_custom_component_input");
 * model.addComponent(controlDistributor.release());
 * @endcode
 *
 * Connecting all `Output` controls to another component:
 * @code
 * const auto& output = controlDistributor->getOutput("controls");
 * auto& controller =
 *     model.updComponent<ActuatorInputController>("/my_actu_controller");
 * controller.connectInput_inputs(output);
 *@endcode
 *
 * Connecting an individual control channel using an alias:
 * @code
 * const auto& output = controlDistributor->getOutput("controls");
 * const auto& channel = output.getChannel("/forceset/soleus_r");
 * auto& controller =
 *     model.updComponent<ActuatorInputController>("/my_actu_controller");
 * controller.connectInput_inputs(channel, "/forceset/soleus_r");
 * @endcode
 */
class OSIMMOCO_API ControlDistributor : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(ControlDistributor, ModelComponent);

public:
//=============================================================================
// OUTPUTS
//=============================================================================
    OpenSim_DECLARE_LIST_OUTPUT(controls, double, getControlForOutputChannel,
            SimTK::Stage::Dynamics);

//=============================================================================
// METHODS
//=============================================================================

    // CONSTRUCTION AND DESTRUCTION
    ControlDistributor();
    ~ControlDistributor() noexcept override;

    ControlDistributor(const ControlDistributor&);
    ControlDistributor(ControlDistributor&&);

    ControlDistributor& operator=(const ControlDistributor&);
    ControlDistributor& operator=(ControlDistributor&&);

    // GET AND SET
    /**
     * Add a control to the control distributor.
     */
    void addControl(const std::string& controlName);

    /**
     * Set the distributor controls stored in the discrete variable.
     *
     * The `controls` vector must be the same size as the number of controls
     * added to the distributor by `addControl()`.
     */
    void setControls(SimTK::State& s, const SimTK::Vector& controls) const;

    /**
     * Get a writable reference to the distributor controls stored in the
     * discrete variable.
     */
    SimTK::Vector& updControls(SimTK::State& s) const;

    /**
     * Get a const reference to the distributor controls stored in the discrete
     * variable.
     */
    const SimTK::Vector& getControls(const SimTK::State& s) const;

    /**
     * Get the control value for the requested output channel.
     */
    double getControlForOutputChannel(
            const SimTK::State& s, const std::string& channel) const;

    /**
     * Get the names of the controls in the order they were added to the
     * distributor.
     */
    std::vector<std::string> getControlNamesInOrder() const;

    /**
     * Get a map of control names to their indices in the control vector.
     */
    std::unordered_map<std::string, int> getControlIndexMap() const {
        return m_controlIndexMap;
    }

protected:
    // MODEL COMPONENT INTERFACE
    void extendRealizeTopology(SimTK::State& state) const override;
    void extendFinalizeFromProperties() override;

private:
    std::unordered_map<std::string, int> m_controlIndexMap;
    mutable SimTK::DiscreteVariableIndex m_discreteVarIndex;
};

} // namespace OpenSim

#endif // OPENSIM_CONTROL_DISTRIBUTOR_H
