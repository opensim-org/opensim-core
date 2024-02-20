#ifndef OPENSIM_INPUT_CONTROLLER_H
#define OPENSIM_INPUT_CONTROLLER_H
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  InputController.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2024 Stanford University and the Authors                *
 * Author(s): Nicholas Bianco                                                 *
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

#include "Controller.h"

#include <OpenSim/Simulation/SimulationUtilities.h>
#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim {

/**
 * InputController is a simple intermediate abstract class for a Controller that
 * computes controls based on scalar values defined via a list Input.
 *
 * Since InputController is an abstract class, derived classes must still
 * implement Controller's virtual computeControls() method. Additionally, it is
 * up to the derived class to define how the scalar values from the Input are
 * mapped to the controls for the actuators in the controller's ActuatorSet.
 * Finally, derived classes must implement the getExpectedInputChannelAliases()
 * method to provide a list of expected Input channel connections to Output
 * channels from other components (e.g., ControlDistributor).
 *
 * InputController provides convenience methods for getting the names and
 * indexes of the controls for the actuators in the controller's ActuatorSet.
 * Non-scalar actuators will have multiple controls, and therefore have multiple
 * control names and indexes. Control information is only available after calling
 * Model::finalizeConnections().
 *
 * Actuator control names and indexes are based on the convention used by the
 * utility function SimulationUtilities::createControlNamesFromModel(), which
 * returns control names and indexes based on the order of the actuators stored
 * in the model. However, we do not check if the order of the actuators stored
 * in the model matches the order of the controls in the underlying system. Use
 * the utility function SimulationUtilities::checkOrderSystemControls() to
 * perform this check when using this controller.
 */
class OSIMSIMULATION_API InputController : public Controller {
OpenSim_DECLARE_ABSTRACT_OBJECT(InputController, Controller);

public:
//=============================================================================
// INPUTS
//=============================================================================
    OpenSim_DECLARE_LIST_INPUT(inputs, double, SimTK::Stage::Velocity,
        "The scalar values used to compute the controls for the actuators.");

//=============================================================================
// METHODS
//=============================================================================

    // CONSTRUCTION AND DESTRUCTION
    InputController();
    ~InputController() override;

    InputController(const InputController& other);
    InputController& operator=(const InputController& other);

    InputController(InputController&& other);
    InputController& operator=(InputController&& other);

    // INTERFACE METHODS
    /**
     * Get the expected list of Input channel aliases.
     *
     * Concrete implementations of InputController must implement this method
     * to provide a list of expected Input channel connections to Output channels
     * from other components.
     *
     * @note Since the alias names may depend on the list of connected actuators,
     *       this is only valid after actuators are connected and
     *       Model::finalizeConnections() has been called.
     */
    virtual std::vector<std::string> getExpectedInputChannelAliases() const = 0;

    // METHODS
    /**
     * Get the names of the controls for the actuators in the controller's
     * ActuatorSet.
     *
     * For scalar actuators, these names are simply the paths of
     * the actuators in the model. For non-scalar actuators, these names are
     * actuator path plus an additional suffix representing the index of the
     * control in the actuator's control vector (e.g., "/actuator_0").
     *
     * @note Only valid after actuators are connected and
     *       Model::finalizeConnections() has been called.
     *
     * @note This does *not* check if the order of the actuators stored in the
     *       model matches the order of the controls in the underlying system.
     *       Use SimulationUtilities::checkOrderSystemControls() to perform
     *       this check.
     */
    const std::vector<std::string>& getControlNames() const;

    /**
     * Get the model control indexes for the controls associated with the
     * actuators in the controller's ActuatorSet.
     *
     * The control indexes are based on the order of the actuators stored in the
     * model. Non-scalar actuators will have multiple controls, and therefore
     * have multiple control indexes. The order of the returned indexes matches
     * the order of the control names returned by getControlNames().
     *
     * @note Only valid after actuators are connected and
     *       Model::finalizeConnections() has been called.
     *
     * @note This does *not* check if the order of the actuators stored in the
     *       model matches the order of the controls in the underlying system.
     *       Use SimulationUtilities::checkOrderSystemControls() to perform
     *       this check.
     */
    const std::vector<int>& getControlIndexes() const;

protected:
    // MODEL COMPONENT INTERFACE
    void extendConnectToModel(Model& model) override;

private:
    std::vector<std::string> m_controlNames;
    std::vector<int> m_controlIndexes;
};

/**
 * ActuatorInputController is the simplest concrete implementation of an
 * InputController.
 *
 * It passes the scalar values from the controller's list Input to the actuators
 * in the controller's ActuatorSet. Therefore, the expected Input channel aliases
 * are simply the names of the actuator controls added to this controller. The
 * Input channel aliases must match the names of the actuator controls in the
 * model, but they do not need to be in the same order.
 */
class OSIMSIMULATION_API ActuatorInputController : public InputController {
    OpenSim_DECLARE_CONCRETE_OBJECT(ActuatorInputController, InputController);
public:

    // CONSTRUCTION AND DESTRUCTION
    ActuatorInputController();
    ~ActuatorInputController() override;

    ActuatorInputController(const ActuatorInputController& other);
    ActuatorInputController& operator=(const ActuatorInputController& other);

    ActuatorInputController(ActuatorInputController&& other);
    ActuatorInputController& operator=(ActuatorInputController&& other);

    // INPUT CONTROLLER INTERFACE
    std::vector<std::string> getExpectedInputChannelAliases() const override;

    // CONTROLLER INTERFACE
    void computeControls(
            const SimTK::State& s, SimTK::Vector& controls) const override;

protected:
    // MODEL COMPONENT INTERFACE
    void extendConnectToModel(Model& model) override;

private:
    std::vector<int> m_controlIndexesInConnecteeOrder;
};

} // namespace OpenSim

#endif // OPENSIM_INPUT_CONTROLLER_H
