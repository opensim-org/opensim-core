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
#include <OpenSim/Common/IO.h>

//=============================================================================
// STATICS
//=============================================================================

using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Controller::Controller() :
    ModelComponent{},
    _numControls{0},
    _actuatorSet{}
{
    constructProperties();
}

Controller::Controller(Controller const& src) :
    ModelComponent{src},
    PropertyIndex_enabled{src.PropertyIndex_enabled},
    PropertyIndex_actuator_list{src.PropertyIndex_actuator_list},
    _numControls{src._numControls},
    _actuatorSet{}
{
    // care: the reason this custom copy constructor exists is to prevent
    // a memory leak (#3247)
    _actuatorSet.setMemoryOwner(false);
}

Controller& Controller::operator=(Controller const& src)
{
    // care: the reason this custom copy assignment exists is to prevent
    // a memory leak (#3247)

    if (&src != this)
    {
        static_cast<ModelComponent&>(*this) = static_cast<ModelComponent const&>(src);
        PropertyIndex_enabled = src.PropertyIndex_enabled;
        PropertyIndex_actuator_list = src.PropertyIndex_actuator_list;
        _numControls = src._numControls;
        _actuatorSet.setSize(0);
    }
    return *this;
}

Controller::~Controller() noexcept = default;

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
//_____________________________________________________________________________

/**
 * Connect properties to local pointers.
 */
void Controller::constructProperties()
{
    setAuthors("Ajay Seth, Frank Anderson, Chand John, Samuel Hamner");
    constructProperty_enabled(true);
    constructProperty_actuator_list();
    _actuatorSet.setMemoryOwner(false);
}

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

//=============================================================================
// GET AND SET
//=============================================================================

//-----------------------------------------------------------------------------
// ON/OFF
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get whether or not this controller is enabled.
 */
bool Controller::isEnabled() const
{
    return get_enabled();
}
//_____________________________________________________________________________
/**
 * Turn this controller on or off.
 */
void Controller::setEnabled(bool aTrueFalse)
{
    upd_enabled() = aTrueFalse;
}

// for any post XML deserialization initialization
void Controller::extendConnectToModel(Model& model)
{
    Super::extendConnectToModel(model);

    // TODO this custom connection code can all disappear
    // if we use a list Socket<Actuator> 

    // make sure controller does not take ownership
    _actuatorSet.setSize(0);
    _actuatorSet.setMemoryOwner(false);

    int nac = getProperty_actuator_list().size();
    if (nac == 0)
        return;
    
    auto actuators = model.getComponentList<Actuator>();
    if (IO::Uppercase(get_actuator_list(0)) == "ALL"){
        for (auto& actuator : actuators) {
            _actuatorSet.adoptAndAppend(&actuator);
        }
        return;
    }
    else{
         for (int i = 0; i < nac; ++i) {
             bool found = false;
             for (auto& actuator : actuators) {
                if (get_actuator_list(i) == actuator.getName()) {
                    _actuatorSet.adoptAndAppend(&actuator);
                    found = true;
                    break;
                }
             }
            if (!found) {
                cerr << "WARN: Controller::connectToModel : Actuator "
                    << get_actuator_list(i) <<
                    " was not found and will be ignored." << endl;
            }
        }
    }
}

/**
 * Create a Controller in the SimTK::System
 */
void Controller::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);
}

// makes a request for which actuators a controller will control
void Controller::setActuators(const Set<Actuator>& actuators)
{
    //TODO this needs to be setting a Socket list of Actuators

    // make sure controller does NOT assume ownership
    _actuatorSet.setSize(0);
    _actuatorSet.setMemoryOwner(false);
    //Rebuild consistent set of actuator lists
    updProperty_actuator_list().clear();
    for (int i = 0; i< actuators.getSize(); i++){
        addActuator(actuators[i]);
    }
}


void Controller::addActuator(const Actuator& actuator)
{
    _actuatorSet.adoptAndAppend(&actuator);

    int found = updProperty_actuator_list().findIndex(actuator.getName());
    if (found < 0) //add if the actuator isn't already in the list
        updProperty_actuator_list().appendValue(actuator.getName());
}

Set<const Actuator>& Controller::updActuators() { return _actuatorSet; }

const Set<const Actuator>& Controller::getActuatorSet() const { return _actuatorSet; }
