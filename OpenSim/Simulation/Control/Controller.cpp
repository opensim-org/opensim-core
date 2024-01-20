/* -------------------------------------------------------------------------- *
 *                          OpenSim:  Controller.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Frank C. Anderson, Chand T. John, Samuel R. Hamner   *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "Controller.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Actuator.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
Controller::Controller() :
        ModelComponent{}, _numControls{0} {
    constructProperties();
}

Controller::~Controller() noexcept = default;

Controller::Controller(const Controller&) = default;

Controller::Controller(Controller&&) = default;

Controller& Controller::operator=(const Controller&) = default;

Controller& Controller::operator=(Controller&&) = default;

void Controller::constructProperties() {
    setAuthors("Ajay Seth, Frank Anderson, Chand John, Samuel Hamner");
    constructProperty_enabled(true);
}

//=============================================================================
// MODEL COMPONENT INTERFACE
//=============================================================================
void Controller::updateFromXMLNode(SimTK::Xml::Element& node,
                                   int versionNumber) {
    if (versionNumber < XMLDocument::getLatestVersion()) {
        if (versionNumber < 30509) {
            // Rename property 'isDisabled' to 'enabled' and
            // negate the contained value.
            std::string oldName{"isDisabled"};
            std::string newName{"enabled"};
            if(node.hasElement(oldName)) {
                auto elem = node.getRequiredElement(oldName);
                bool isDisabled = false;
                elem.getValue().tryConvertToBool(isDisabled);

                // now update tag name to 'enabled'
                elem.setElementTag(newName);
                // update its value to be the opposite of 'isDisabled'
                elem.setValue(SimTK::String(!isDisabled));
            }
        }
        if (versionNumber < 40600) {
            if (node.hasElement("actuator_list")) {
                // Rename element from 'actuator_list' to 'socket_actuators'.
                auto actuators = node.getRequiredElement("actuator_list");
                actuators.setElementTag("socket_actuators");

                // Store the space-delimited actuator names in a temporary
                // variable. We'll use these names to connect the actuators
                // to the socket in extendConnectToModel().
                std::string values = actuators.getValueAs<std::string>();
                std::istringstream iss(values);
                _actuatorNamesFromXML = std::vector<std::string>{
                        std::istream_iterator<std::string>{iss},
                        std::istream_iterator<std::string>{}};

                // Clear the value of the element so finalizeConnections() does
                // not try to connect to invalid connectee paths.
                actuators.setValueAs<std::string>("");
            }
        }
    }

    Super::updateFromXMLNode(node, versionNumber);
}

void Controller::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);

    // If XML deserialization saved a list of actuator names, use them to
    // create valid connections to the 'actuators' list Socket.
    if (!_actuatorNamesFromXML.empty()) {
        auto& socket = updSocket<Actuator>("actuators");
        for (const auto& actuatorName : _actuatorNamesFromXML) {
            // If the actuator name is "ALL", connect all actuators and break.
            if (IO::Uppercase(actuatorName) == "ALL") {
                setActuators(model.getComponentList<Actuator>());
                break;
            }

            // Otherwise, find the actuator by name and connect it.
            for (const auto& actuator : model.getComponentList<Actuator>()) {
                if (actuator.getName() == actuatorName) {
                    // Connect the actuator to the socket.
                    addActuator(actuator);
                    break;
                }
            }
        }
        // Call finalizeConnection() to sync the connectee path names with the
        // connected Actuators.
        socket.finalizeConnection(model);
        _actuatorNamesFromXML.clear();
    }
}

//=============================================================================
// GET AND SET
//=============================================================================
bool Controller::isEnabled() const {
    return get_enabled();
}

void Controller::setEnabled(bool aTrueFalse) {
    upd_enabled() = aTrueFalse;
}

void Controller::setActuators(const Set<Actuator>& actuators) {
    updSocket<Actuator>("actuators").disconnect();
    for (int i = 0; i < actuators.getSize(); i++){
        addActuator(actuators.get(i));
    }
}

void Controller::setActuators(
        const SimTK::Array_<SimTK::ReferencePtr<const Actuator>>& actuators) {
    updSocket<Actuator>("actuators").disconnect();
    for (const auto& actu : actuators) {
        addActuator(actu.getRef());
    }
}

void Controller::setActuators(const ComponentList<const Actuator>& actuators) {
    updSocket<Actuator>("actuators").disconnect();
    for (const auto& actu : actuators) {
        addActuator(actu);
    }
}

void Controller::addActuator(const Actuator& actuator) {
    appendSocketConnectee_actuators(actuator);
}

SimTK::Array_<SimTK::ReferencePtr<const Actuator>>
Controller::getActuators() const {
    const auto& socket = getSocket<Actuator>("actuators");
    int nc = static_cast<int>(socket.getNumConnectees());
    SimTK::Array_<SimTK::ReferencePtr<const Actuator>> actuators(nc);
    for (int i = 0; i < (int)socket.getNumConnectees(); ++i) {
        const Actuator& actu = socket.getConnectee(i);
        actuators[i] = &actu;
    }

    return actuators;
}
