#ifndef OPENSIM_ACTUATOR_INPUT_CONTROLLER_H
#define OPENSIM_ACTUATOR_INPUT_CONTROLLER_H
/* -------------------------------------------------------------------------- *
 *                    OpenSim:  ActuatorInputController.h                     *
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

#include <OpenSim/Simulation/Control/InputController.h>

namespace OpenSim {

/**
 * ActuatorInputController is the simplest concrete implementation of an
 * InputController used internally by Moco to pass scalar control values to the 
 * actuators not associated with a user-added controller.
 *
 * The scalar values from the controller's list Input are passed to the 
 * actuators in the controller's ActuatorSet. Therefore, the expected Input 
 * channel aliases are simply the names of the actuator controls added to this 
 * controller. The Input channel aliases must match the names of the actuator 
 * controls in the model, but they do not need to be in the same order.
 */
class ActuatorInputController : public InputController {
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

    /// @brief Test comment. 
    // TODO if no actuators connected, returns empty vector.
    std::vector<std::string> getInputControlLabels() const override;
    void computeControlsImpl(
            const SimTK::State& s, SimTK::Vector& controls) const override;

protected:
    // MODEL COMPONENT INTERFACE
    void extendConnectToModel(Model& model) override;

private:
    std::vector<int> m_controlIndexesInConnecteeOrder;
};

} // namespace OpenSim

#endif // OPENSIM_ACTUATOR_INPUT_CONTROLLER_H