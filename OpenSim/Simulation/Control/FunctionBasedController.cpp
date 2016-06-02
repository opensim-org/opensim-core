/* -------------------------------------------------------------------------- *
 *                     OpenSim:  FunctionBasedController.cpp                  *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Ayman Habib                                          *
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
#include "FunctionBasedController.h"
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
/*
 * Default constructor.
 */
FunctionBasedController::FunctionBasedController() :
    Controller()
{
    setNull();
    constructInfrastructure();
}

/*
 * Destructor.
 */
FunctionBasedController::~FunctionBasedController()
{
}

/*
 * Set NULL values for all member variables.
 */
void FunctionBasedController::setNull()
{
    setAuthors("Ajay Seth");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void FunctionBasedController::constructProperties()
{
    constructProperty_ControlFunctions(FunctionSet());
}


// compute the control value for an actuator
void FunctionBasedController::computeControls(const SimTK::State& s, SimTK::Vector& controls) const
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

void FunctionBasedController::
    prescribeControlForActuator(int index, Function *prescribedFunction)
{
    SimTK_ASSERT( index < getActuatorSet().getSize(), 
        "FunctionBasedController::computeControl:  index > number of actuators" );
    SimTK_ASSERT( index >= 0,  
        "FunctionBasedController::computeControl:  index < 0" );
    if(index >= get_ControlFunctions().getSize())
        upd_ControlFunctions().setSize(index+1);
    upd_ControlFunctions().set(index, prescribedFunction);  
}

void FunctionBasedController::
    prescribeControlForActuator(const std::string actName, 
                                Function *prescribedFunction)
{
    int index = getProperty_actuator_list().findIndex(actName);
    if(index < 0 )
        throw Exception("FunctionBasedController does not have "+actName+" in its list of actuators to control.");
    prescribeControlForActuator(index, prescribedFunction);
}

void FunctionBasedController::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();
    auto& columnOutput = updOutput("control");
    columnOutput.clearChannels();
    for (int i = 0; i< getActuatorSet().getSize(); i++)
        columnOutput.addChannel(getActuatorSet().get(i).getName());
}


SimTK::Vector FunctionBasedController::
getControlAtTime(const SimTK::State& s, const std::string& actName) const {
    SimTK::Vector actControls(1, 0.0);
    SimTK::Vector time(1, s.getTime());
    // Add in code to actually get the control!
    int index = getProperty_actuator_list().findIndex(actName);
    if (index < 0)
        throw Exception("FunctionBasedController does not have " + actName + " in its list of actuators to control.");
    actControls[0] = get_ControlFunctions()[index].calcValue(time);
    return actControls;
}