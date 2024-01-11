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

    // Add prescribed functions for any actuators that were specified by name.
    const auto& socket = getSocket<Actuator>("actuators");
    for (const auto& pair : m_prescribedFunctionPairs) {
        // Check if the name in the pair is the name or path of an actuator in
        // the list Socket.
        int actuIndex = getActuatorIndexFromLabel(pair.first);
        OPENSIM_THROW_IF_FRMOBJ(actuIndex < 0, Exception,
            "Actuator {} is not connected the controller.", pair.first);

        prescribeControlForActuator(actuIndex, pair.second);
    }

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
            if (!controlFuncs.contains(columnLabel)) {

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
                prescribeControlForActuator(actuIndex, controlFunction);
            }
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
void PrescribedController::prescribeControlForActuator(int index,
        Function* prescribedFunction) {
    OPENSIM_THROW_IF_FRMOBJ(index < 0,  
            Exception, "Index was " + std::to_string(index) +
                       " but must be nonnegative." );

    if (index >= get_ControlFunctions().getSize()) {
        upd_ControlFunctions().setSize(index + 1);
    }
    upd_ControlFunctions().set(index, prescribedFunction);  
}

void PrescribedController::prescribeControlForActuator(
        const std::string& actuLabel, Function* prescribedFunction) {
    m_prescribedFunctionPairs.emplace_back(actuLabel, prescribedFunction);
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
