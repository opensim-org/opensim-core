/* -------------------------------------------------------------------------- *
 *                     OpenSim:  PrescribedController.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2023 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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
#include "PrescribedController.h"
#include <memory>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/PiecewiseConstantFunction.h>
#include <OpenSim/Common/CommonUtilities.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Actuator.h>

using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
PrescribedController::PrescribedController() : Controller() {
    setNull();
    constructProperties();
}

PrescribedController::PrescribedController(const std::string& controlsFileName,
        int interpMethodType) : Controller() {
    setNull();
    constructProperties();
    set_controls_file(controlsFileName);
    set_interpolation_method(interpMethodType);
}

PrescribedController::~PrescribedController() = default;

void PrescribedController::setNull() {
    setAuthors("Ajay Seth");
}

void PrescribedController::constructProperties()
{
    constructProperty_ControlFunctions(FunctionSet());
    constructProperty_controls_file();
    constructProperty_interpolation_method();
}

//=============================================================================
// MODEL COMPONENT INTERFACE
//=============================================================================
void PrescribedController::updateFromXMLNode(SimTK::Xml::Element& node,
                                   int versionNumber) {
    if (versionNumber < 40600) {
        int iactu = 0;
        if (node.hasElement("actuator_list")) {
            auto actuators = node.getRequiredElement("actuator_list");
            std::string values = actuators.getValueAs<std::string>();
            std::istringstream iss(values);
            auto actuatorNamesFromXML = std::vector<std::string>{
                    std::istream_iterator<std::string>{iss},
                    std::istream_iterator<std::string>{}};
            for (const auto& actuName : actuatorNamesFromXML) {
                _actuLabelsToControlFunctionIndexMap[actuName] = iactu++;
            }
        }
    } else {
        int iactu = 0;
        if (node.hasElement("socket_actuators")) {
            auto actuators = node.getRequiredElement("socket_actuators");
            std::string values = actuators.getValueAs<std::string>();
            std::istringstream iss(values);
            auto actuatorNamesFromXML = std::vector<std::string>{
                    std::istream_iterator<std::string>{iss},
                    std::istream_iterator<std::string>{}};
            for (const auto& actuName : actuatorNamesFromXML) {
                _actuLabelsToControlFunctionIndexMap[actuName] = iactu++;
            }
        }
    }

    Super::updateFromXMLNode(node, versionNumber);
}

void PrescribedController::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);
    auto& socket = updSocket<Actuator>("actuators");

    // Verify that all connected actuators have a control function.
    const FunctionSet& controlFuncs = get_ControlFunctions();
    OPENSIM_THROW_IF_FRMOBJ(
        controlFuncs.getSize() != (int)socket.getNumConnectees(), Exception, 
        "The number of control functions ({}) does not match the "
        "number of actuators ({}) connected to the controller.",
        controlFuncs.getSize(), socket.getNumConnectees());

    // If the 'ALL' keyword was provided via the actuator list (pre-4.6), then 
    // populate the control function index map with all actuators in 
    // connectee order. This also handles the case where control functions are
    // provided via the ControlFunctions property directly (i.e., the index map 
    // is empty).
    for (const auto& kv : _actuLabelsToControlFunctionIndexMap) {
        if (IO::Uppercase(kv.first) == "ALL") {
            _actuLabelsToControlFunctionIndexMap.clear();
            break;
        }
    }
    if (_actuLabelsToControlFunctionIndexMap.empty()) {
        for (int i = 0; i < (int)socket.getNumConnectees(); ++i) {
            const auto& connectee = socket.getConnectee(i);
            const auto& path = connectee.getAbsolutePathString();
            _actuLabelsToControlFunctionIndexMap[path] = i;
        }
    }

    // If a controls file was specified, load it and create control functions
    // for any actuators that do not already have one.
    if (!getProperty_controls_file().empty()) {

        // Load the controls file and find the time column and column labels.
        const Storage controls(get_controls_file());
        const Array<string>& columnLabels = controls.getColumnLabels();
        int tcol = columnLabels.findIndex("time");
        if (tcol < 0) {
            tcol = columnLabels.findIndex("t");
            OPENSIM_THROW_IF_FRMOBJ(tcol < 0, Exception, "Prescribed controls "
                "file was not specified as a function of time.")
        }
        int nrows = controls.getSize();
        Array<double> time(0.0, nrows);
        Array<double> data(0.0, nrows);
        controls.getTimeColumn(time);

        for (int i = 0; i < columnLabels.getSize(); ++i) {
            // Skip the time column.
            if (i == tcol) continue;

            // If this column does not have an associated control function, we
            // need to create one.
            const string& columnLabel = columnLabels[i];
            if (!_actuLabelsToControlFunctionIndexMap.count(columnLabel)) {

                // See if the column label matches a connected actuator. If not,
                // find and add the actuator to the controller.
                int actuIndex = getActuatorIndexFromLabel(columnLabel);
                if (actuIndex < 0) {
                    bool foundActuator = false;
                    for (const auto& actu : model.getComponentList<Actuator>()) {
                        if (actu.getName() == columnLabel) {
                            addActuator(actu);
                            foundActuator = true;
                            break;
                        }
                        if (actu.getAbsolutePathString() == columnLabel) {
                            addActuator(actu);
                            foundActuator = true;
                            break;
                        }
                    }
                    OPENSIM_THROW_IF_FRMOBJ(!foundActuator, Exception,
                        "Control provided from file with label {}, but no "
                        "matching Actuator was found in the model.",
                            columnLabel)

                    // If we found a matching actuator, call
                    // finalizeConnection() to sync the connectee path names
                    // with the Actuator connectee.
                    socket.finalizeConnection(model);
                }

                // Create the control function and assign it to the actuator.
                controls.getDataColumn(
                        controls.getStateIndex(columnLabel), data);
                std::unique_ptr<Function> controlFunction = 
                        createFunctionFromData(columnLabel, time, data);
                prescribeControlForActuator(columnLabel, *controlFunction);
            }
        }
    }

    // Populate the actuator index to control function index map.
    std::unordered_map<int, int> actuToControlFunctionIndexMap;
    for (const auto& pair : _actuLabelsToControlFunctionIndexMap) {
        int actuIndex = getActuatorIndexFromLabel(pair.first);
        OPENSIM_THROW_IF_FRMOBJ(actuIndex < 0, Exception,
            "Actuator {} was not found in the model.", pair.first)
        
        OPENSIM_THROW_IF_FRMOBJ(
            actuToControlFunctionIndexMap.count(actuIndex), Exception, 
            "Expected actuator {} to have one control function "
            "assigned, but multiple control functions were detected. "
            "This may have occurred because a control function was "
            "specified by actuator name and by actuator path.",
            socket.getConnectee(actuIndex).getAbsolutePathString())

        actuToControlFunctionIndexMap[actuIndex] = pair.second;
    }

    // Reorder the control functions to match the order of the actuators. We 
    // must do this so that the actuator connectee order matches the control
    // function order during serialization.
    FunctionSet controlFuncsCopy = get_ControlFunctions();
    for (int i = 0; i < (int)socket.getNumConnectees(); ++i) {
        const int controlFuncIndex = actuToControlFunctionIndexMap.at(i);
        upd_ControlFunctions().set(i, controlFuncsCopy.get(controlFuncIndex));
    }
}

//=============================================================================
// CONTROLLER INTERFACE
//=============================================================================
void PrescribedController::computeControls(const SimTK::State& s,
        SimTK::Vector& controls) const {
    SimTK::Vector actControls(1, 0.0);
    SimTK::Vector time(1, s.getTime());

    const auto& socket = getSocket<Actuator>("actuators");
    for(int i = 0; i < (int)socket.getNumConnectees(); ++i){
        actControls[0] = get_ControlFunctions()[i].calcValue(time);
        socket.getConnectee(i).addInControls(actControls, controls);
    }
}

//=============================================================================
// GET AND SET
//=============================================================================
void PrescribedController::prescribeControlForActuator(
        const std::string& actuLabel, const Function& prescribedFunction) {
    
    FunctionSet& controlFuncs = upd_ControlFunctions();
    if (_actuLabelsToControlFunctionIndexMap.count(actuLabel)) {
        const int index = _actuLabelsToControlFunctionIndexMap.at(actuLabel);
        controlFuncs.set(index, prescribedFunction);
    } else {
        const int size = controlFuncs.getSize();
        controlFuncs.setSize(size + 1);
        controlFuncs.set(size, prescribedFunction);
        _actuLabelsToControlFunctionIndexMap[actuLabel] = size;
    }
}

//=============================================================================
// UTILITY
//=============================================================================
std::unique_ptr<Function> PrescribedController::createFunctionFromData(
        const std::string& name, const Array<double>& time, 
        const Array<double>& data) const {
    int method = 1;
    if(!getProperty_interpolation_method().empty()) {
        method = get_interpolation_method();
    }

    if(method > 0) {
        return std::make_unique<GCVSpline>(method, time.getSize(), 
                &time[0], &data[0], name);
    }

    if(method == 0) {
        return std::make_unique<PiecewiseConstantFunction>(
                time.getSize(), &time[0], &data[0], name);
    }

    OPENSIM_THROW_FRMOBJ(Exception, "Invalid interpolation method.");
}

int PrescribedController::getActuatorIndexFromLabel(
        const std::string& actuLabel) const {
    const auto& socket = getSocket<Actuator>("actuators");
    for (int i = 0; i < (int)socket.getNumConnectees(); ++i) {
        const Actuator& actu = socket.getConnectee(i);

        // Check the actuator name.
        if (actu.getName() == actuLabel) {
            return i;
        }

        // Check the actuator path.
        if (actu.getAbsolutePathString() == actuLabel) {
            return i;
        }
    }

    return -1;
}
