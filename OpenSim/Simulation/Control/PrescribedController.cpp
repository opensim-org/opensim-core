/* -------------------------------------------------------------------------- *
 *                     OpenSim:  PrescribedController.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

//=============================================================================
// STATICS
//=============================================================================

// This command indicates that any identifier (class, variable, method, etc.)
// defined within the OpenSim namespace can be used in this file without the
// "OpenSim::" prefix.
using namespace OpenSim;
using namespace std;



//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
/*
 * Default constructor.
 */
PrescribedController::PrescribedController() :
    Controller()
{
    setNull();
    constructProperties();
}

/*
 * Convenience constructor.
 */
PrescribedController::
    PrescribedController(const std::string& controlsFileName, 
                         int interpMethodType) : Controller()
{
    setNull();
    constructProperties();
    set_controls_file(controlsFileName);
    set_interpolation_method(interpMethodType);
}

/*
 * Destructor.
 */
PrescribedController::~PrescribedController()
{
}

/*
 * Set NULL values for all member variables.
 */
void PrescribedController::setNull()
{
    setAuthors("Ajay Seth");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void PrescribedController::constructProperties()
{
    constructProperty_ControlFunctions(FunctionSet());
    constructProperty_controls_file();
    constructProperty_interpolation_method();
}


void PrescribedController::extendConnectToModel(Model& model)
{
    Super::extendConnectToModel(model);
    if(!getProperty_controls_file().empty()){
        Storage controls(get_controls_file());
        const Array<string>& columns = controls.getColumnLabels();

        int ncols = columns.getSize();

        int tcol = columns.findIndex("time");
        if(tcol < 0){
            tcol = columns.findIndex("t");
            if(tcol < 0){
                throw Exception("PrescribedController::connectToModel prescribed "
                "controls file was not specified as functions of time.",
                    __FILE__, __LINE__);
            }
        }
        int nrows = controls.getSize();
        Array<double> time(0.0, nrows);
        Array<double> data(0.0, nrows);
        controls.getTimeColumn(time);

        FunctionSet& controlFuncs = upd_ControlFunctions();
        const Set<Actuator>& modelActuators = getModel().getActuators();

        Set<Actuator>& controllerActuators = updActuators();

        for(int i=0; i<ncols; ++i){
            if(i == tcol) continue;
            const string& actName = columns[i];
            int found = controlFuncs.getIndex(actName);
            // if the columns is for a control already part of the set,
            // or is time, ignore it.
            if(found < 0){  // not found in the controllers set of functions
                // find a corresponding actuator in the model
                found = modelActuators.getIndex(actName);
                if(found >= 0){ // found a corresponding actuator
                    controls.getDataColumn(controls.getStateIndex(actName), 
                                            data);
                    Function* pfunc=createFunctionFromData(actName, time, data);
                    //if not already assigned to this controller, assign it
                    int inC = controllerActuators.getIndex(actName);
                    if(inC >= 0)
                        prescribeControlForActuator(inC, pfunc);
                    else{ // add the actuator to the controller's list
                        updProperty_actuator_list().appendValue(actName);
                        controllerActuators.adoptAndAppend(&modelActuators[found]);
                        prescribeControlForActuator(actName, pfunc);
                    }
                }
                else{
                    cout << "PrescribedController::extendConnectToModel() could not "
                        "find actuator '" << actName << "' in the model." <<endl;
                }
            }// if found in functions, it has already been prescribed
        }// end looping through columns
    }// if no controls storage specified, do nothing
}


// compute the control value for an actuator
void PrescribedController::computeControls(const SimTK::State& s, SimTK::Vector& controls) const
{
    SimTK::Vector actControls(1, 0.0);
    SimTK::Vector time(1, s.getTime());

    for(int i=0; i<getActuatorSet().getSize(); i++){
        actControls[0] = get_ControlFunctions()[i].calcValue(time);
        getActuatorSet()[i].addInControls(actControls, controls);
    }  
}


//=============================================================================
// GET AND SET
//=============================================================================

void PrescribedController::
    prescribeControlForActuator(int index, OpenSim::Function *prescribedFunction)
{
    SimTK_ASSERT( index < getActuatorSet().getSize(), 
        "PrescribedController::computeControl:  index > number of actuators" );
    SimTK_ASSERT( index >= 0,  
        "PrescribedController::computeControl:  index < 0" );
    if(index >= get_ControlFunctions().getSize())
        upd_ControlFunctions().setSize(index+1);
    upd_ControlFunctions().set(index, prescribedFunction);  
}

void PrescribedController::prescribeControlForActuator(const std::string actName, 
    OpenSim::Function *prescribedFunction)
{
    int index = getProperty_actuator_list().findIndex(actName);
    if(index < 0 )
        throw Exception("PrescribedController does not have "+actName+" in its list of actuators to control.");
    prescribeControlForActuator(index, prescribedFunction);
}

// utility
OpenSim::Function* PrescribedController::createFunctionFromData(const std::string& name,
    const Array<double>& time, const Array<double>& data)
{
    int method = 1;
    if(!getProperty_interpolation_method().empty())
        method = get_interpolation_method();

    if(method > 0)
        return new GCVSpline(method, time.getSize(), &time[0], &data[0], name);
    else if(method ==0)
        return new PiecewiseConstantFunction(time.getSize(), 
                                                    &time[0], &data[0], name);
    else
        throw Exception("PrescribedController- Invalid interpolation method.");
}