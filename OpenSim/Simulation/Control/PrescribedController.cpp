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
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/PiecewiseConstantFunction.h>
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
void PrescribedController::extendConnectToModel(Model& model)
{
    Super::extendConnectToModel(model);
    const auto& socket = getSocket<Actuator>("actuators");

    // If a controls file was specified, load it and create control functions
    // for any actuators that do not already have one.
    if(!getProperty_controls_file().empty()) {

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

        const FunctionSet& controlFuncs = get_ControlFunctions();
        for (int i = 0; i < columnLabels.getSize(); ++i) {
            // Skip the time column.
            if (i == tcol) continue;

            // If this column does not have an associated control function, we
            // need to create one.
            const string& columnLabel = columnLabels[i];
            if (!_actuLabelsToControlFunctionIndexMap.count(columnLabel)) {

                // Search for the column label in the model's actuators.
                int actuIndex = getActuatorIndexFromLabel(columnLabel);
                OPENSIM_THROW_IF_FRMOBJ(actuIndex < 0, Exception,
                    "The controls file contains column {}, but no Actuator "
                    "with this label is connected to the controller.",
                    columnLabel);

                // Create the control function and assign it to the actuator.
                controls.getDataColumn(
                    controls.getStateIndex(columnLabel), data);
                Function* controlFunction = createFunctionFromData(columnLabel,
                    time, data);
                prescribeControlForActuator(columnLabel, controlFunction);
            }
        }
    }

    // Populate the _actuIndexToControlFunctionIndexMap.
    for (const auto& pair : _actuLabelsToControlFunctionIndexMap) {
        int actuIndex = getActuatorIndexFromLabel(pair.first);
        if (actuIndex < 0) {
            OPENSIM_THROW_FRMOBJ(Exception,
                "Actuator {} was not found in the model.", pair.first)
        }
        _actuIndexToControlFunctionIndexMap[actuIndex] = pair.second;
    }

    // Check for actuators with multiple control functions.
    std::vector<int> uniqueValues;
    for (const auto& pair : _actuIndexToControlFunctionIndexMap) {
        int value = pair.second;
        if (std::find(uniqueValues.begin(), uniqueValues.end(), value) !=
                uniqueValues.end()) {
            OPENSIM_THROW_FRMOBJ(Exception,
                    "Expected actuator {} to have one control function "
                    "assigned, but multiple control functions were detected. "
                    "This may have occurred because a control function was "
                    "specified by actuator name and by actuator path.",
                    socket.getConnectee(pair.first).getAbsolutePathString())
        } else {
            uniqueValues.push_back(value);
        }
    }

    // Verify that all actuators have a control function.
    const FunctionSet& controlFuncs = get_ControlFunctions();
    OPENSIM_THROW_IF_FRMOBJ(controlFuncs.getSize() != socket.getNumConnectees(),
        Exception, "The number of control functions ({}) does not match the "
        "number of actuators ({}) connected to the controller.",
        controlFuncs.getSize(), socket.getNumConnectees());
}

//=============================================================================
// CONTROLLER INTERFACE
//=============================================================================
void PrescribedController::computeControls(const SimTK::State& s,
        SimTK::Vector& controls) const {
    SimTK::Vector actControls(1, 0.0);
    SimTK::Vector time(1, s.getTime());

    const auto& socket = getSocket<Actuator>("actuators");
    for(int i = 0; i < socket.getNumConnectees(); ++i){
        actControls[0] = get_ControlFunctions()[i].calcValue(time);
        socket.getConnectee(i).addInControls(actControls, controls);
    }
}

//=============================================================================
// GET AND SET
//=============================================================================
void PrescribedController::prescribeControlForActuator(
        const std::string& actuLabel, Function* prescribedFunction) {
    prescribedFunction->setName(actuLabel);
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
Function* PrescribedController::createFunctionFromData(const std::string& name,
        const Array<double>& time, const Array<double>& data) const {
    int method = 1;
    if(!getProperty_interpolation_method().empty()) {
        method = get_interpolation_method();
    }

    if(method > 0) {
        return new GCVSpline(method, time.getSize(), &time[0], &data[0], name);
    }

    if(method == 0) {
        return new PiecewiseConstantFunction(time.getSize(), 
                                                    &time[0], &data[0], name);
    }

    OPENSIM_THROW_FRMOBJ(Exception, "Invalid interpolation method.");
}

int PrescribedController::getActuatorIndexFromLabel(
        const std::string& actuLabel) const {
    const auto& socket = getSocket<Actuator>("actuators");
    for (int i = 0; i < socket.getNumConnectees(); ++i) {
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
