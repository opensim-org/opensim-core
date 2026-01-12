/* -------------------------------------------------------------------------- *
 *                     OpenSim:  ControlSetController.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Jack Middleton                                                  *
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

/*
 * Author: Jack Middleton
 */

//=============================================================================
// INCLUDES
//=============================================================================
#include "ControlSetController.h"

#include "ControlLinear.h"
#include "ControlSet.h"
#include "Controller.h"

#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/Actuator.h>
#include <OpenSim/Simulation/Model/Model.h>

//=============================================================================
// STATICS
//=============================================================================

// This command indicates that any identifier (class, variable, method, etc.)
// defined within the OpenSim namespace can be used in this file without the
// "OpenSim::" prefix.
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
ControlSetController::~ControlSetController() { delete _controlSet; }
//_____________________________________________________________________________
/**
 * Default constructor
 */

ControlSetController::ControlSetController()
        : Controller(), _controlsFileName(_controlsFileNameProp.getValueStr()) {
    setNull();
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 */
ControlSetController::ControlSetController(
        const ControlSetController& aController)
        : Controller(aController),
          _controlsFileName(_controlsFileNameProp.getValueStr()) {
    setNull();
    copyData(aController);
}

//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void ControlSetController::setNull() {
    setupProperties();

    _model = NULL;
    _controlSet = NULL;
}
//_____________________________________________________________________________
/**
 * Set name of ControlSet file
 */
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ControlSetController::setControlSetFileName(
        const std::string& controlSetFileName) {
    _controlsFileName = controlSetFileName;
}
void ControlSetController::setupProperties() {
    std::string comment = "A Storage (.sto) or an XML control nodes file "
                          "containing the controls for this controlSet.";
    _controlsFileNameProp.setComment(comment);
    _controlsFileNameProp.setName("controls_file");
    _propertySet.append(&_controlsFileNameProp);
}
//_____________________________________________________________________________
/**
 * Copy the member variables of the specified controller.
 */
void ControlSetController::copyData(const ControlSetController& aController) {
    _controlsFileName = aController._controlsFileName;
}

//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assignment operator.
 */
ControlSetController& ControlSetController::operator=(
        const ControlSetController& aController) {
    // BASE CLASS
    Object::operator=(aController);

    // DATA
    copyData(aController);

    return (*this);
}

//=============================================================================
// GET AND SET
//=============================================================================

//=============================================================================
// CONTROL
//=============================================================================

// compute the control value for all actuators this Controller is responsible
// for
void ControlSetController::computeControls(
        const SimTK::State& s, SimTK::Vector& controls) const {
    SimTK_ASSERT(_controlSet,
            "ControlSetController::computeControls controlSet is NULL");

    std::string actName;
    int index = -1;

    const auto& socket = getSocket<Actuator>("actuators");
    const int na = static_cast<int>(socket.getNumConnectees());
    for (int i = 0; i < na; ++i) {
        const auto& actu = socket.getConnectee(i);
        actName = actu.getName();
        index = _controlSet->getIndex(actName);
        if (index < 0) {
            actName = actName + ".excitation";
            index = _controlSet->getIndex(actName);
        }

        if (index >= 0) {
            SimTK::Vector actControls(
                    1, _controlSet->get(index).getControlValue(s.getTime()));
            actu.addInControls(actControls, controls);
        }
    }
}

double ControlSetController::getFirstTime() const {
    Array<int> controlList;
    SimTK_ASSERT(_controlSet,
            "ControlSetController::getFirstTime controlSet is NULL");

    _controlSet->getControlList("ControlLinear", controlList);

    if (controlList.getSize() < 1) {
        return (-SimTK::Infinity);
    } else {
        ControlLinear& control =
                (ControlLinear&)_controlSet->get(controlList[0]);
        return (control.getFirstTime());
    }
}

double ControlSetController::getLastTime() const {
    Array<int> controlList;
    _controlSet->getControlList("ControlLinear", controlList);

    if (controlList.getSize() < 1) {
        return (SimTK::Infinity);
    } else {
        ControlLinear& control =
                (ControlLinear&)_controlSet->get(controlList[0]);
        return (control.getLastTime());
    }
}

void ControlSetController::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();

    bool hasFile = !_controlsFileName.empty() &&
                   _controlsFileName.compare("Unassigned");

    // The result of default constructing and adding  this to a model
    if (_controlSet == nullptr &&  !hasFile) {
        log_warn("ControlSetController::extendFinalizeFromProperties '{}' unassigned.", 
            _controlsFileNameProp.getName());
        log_warn("No ControlSet loaded or set. Use "
                 "ControSetController::setControlSetFileName()"
                 "to specify file and try again.");
        setEnabled(false);
        return;
    }

    ControlSet* loadedControlSet = nullptr;
    if (hasFile) {
        try {
            if (_controlsFileName.rfind(".sto") != std::string::npos)
                loadedControlSet = new ControlSet(Storage(_controlsFileName));
            else
                loadedControlSet = new ControlSet(_controlsFileName);
        }
        // Should only catch an "UnaccessibleFileException" since we would want
        // to know if the file was corrupt or in the wrong format
        catch (const Exception& e) {
            // throw Exception(msg);
            // TODO: Should throw a specific "UnaccessibleControlFileException"
            // testSerializeOpenSimObjects should not expect to just add garbage
            // filled objects (components) to a model and expect to serialize-
            // must be changed!
            log_error("ControlSetController::extendFinalizeFromProperties: "
                      "Unable to load control set file '{}'.\nDetails: {}",
                    _controlsFileName, e.getMessage());
        }
    }

    if (loadedControlSet && _controlSet) {
        log_warn("ControlSetController::extendFinalizeFromProperties '{}' "
                 "loaded and will replace existing ControlSet '{}'.",
                _controlsFileName, _controlSet->getName());
        delete _controlSet;
    }

    if (loadedControlSet) {
        // Now set the current control set from what was loaded
        _controlSet = loadedControlSet;
        setEnabled(true);
    }
}

void ControlSetController::extendConnectToModel(Model& model) {
    const auto& socket = updSocket<Actuator>("actuators");
    std::string ext = ".excitation";
    for (int i = 0; _controlSet != nullptr && i < _controlSet->getSize(); ++i) {
        std::string actName = _controlSet->get(i).getName();
        if (actName.length() > ext.length() &&
                !actName.compare(
                        actName.length() - ext.length(), ext.length(), ext)) {
            actName.erase(actName.length() - ext.length(), ext.length());
        }

        // Check that the actuator is connected to the controller.
        bool isConnected = false;
        for (int iactu = 0; iactu < (int)socket.getNumConnectees(); ++iactu) {
            if (socket.getConnectee(iactu).getName() == actName) {
                log_cout("ControlSetController::extendConnectToModel "
                         "Actuator '{}' already connected to "
                         "ControlSetController '{}'.",
                        actName, getName());
                isConnected = true;
                break;
            }
        }

        // If not already connected, try to connect to an actuator in the model.
        if (!isConnected) {
            for (const auto& actu : model.getComponentList<Actuator>()) {
                if (actu.getName() == actName) {
                    log_cout("ControlSetController::extendConnectToModel "
                             "Connecting ControlSetController '{}' to Actuator "
                             "'{}'.",
                            getName(), actu.getName());
                    addActuator(actu);
                    isConnected = true;
                    break;
                }
            }
        }
        updSocket<Actuator>("actuators").finalizeConnection(model);

        OPENSIM_THROW_IF_FRMOBJ(!isConnected, Exception,
            "Control with name '{}' provided in the ControlSet '{}', but no "
            "matching Actuator was found in the model.",
            actName, _controlSet->getName());
    }
}
