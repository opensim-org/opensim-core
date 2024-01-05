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
// COMPONENT INTERFACE
//=============================================================================
void Controller::updateFromXMLNode(SimTK::Xml::Element& node,
                                   int versionNumber) {
    if(versionNumber < XMLDocument::getLatestVersion()) {
        if(versionNumber < 30509) {
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
    }

    Super::updateFromXMLNode(node, versionNumber);
}

void Controller::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);
    // TODO check if all actuators in connectee paths are in the model?
}

void Controller::extendAddToSystem(SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
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
        const auto& actu = actuators.get(i);
        addActuator(actuators[i]);
    }
}

void Controller::setActuators(const SimTK::Array_<Actuator>& actuators) {
    updSocket<Actuator>("actuators").disconnect();
    for (const auto& actu : actuators) {
        addActuator(actu);
    }
}

void Controller::addActuator(const Actuator& actuator) {
    connectSocket_actuators(actuator);
}

// const SimTK::Array_<const Actuator>& Controller::getActuators() const {
//     const auto& socket = getSocket<Actuator>("actuators");
//     const int nc = static_cast<int>(socket.getNumConnectees());
//     for (int i = 0; i < nc; ++i) {
//         const auto& actuator = socket.getConnectee(i);
//         _actuators.push_back(&actuator);
//     }
//
// }


// const Set<const Actuator>& Controller::getActuatorSet() const {
//
//     // TODO
// }
