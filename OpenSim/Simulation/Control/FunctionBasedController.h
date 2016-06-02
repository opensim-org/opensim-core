#ifndef OPENSIM_FUNCTION_BASED_CONTROLLER_H_
#define OPENSIM_FUNCTION_BASED_CONTROLLER_H_


/* -------------------------------------------------------------------------- *
 *                      OpenSim:  FunctionBasedController.h                   *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
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

#include "Controller.h"
#include <OpenSim/Common/FunctionSet.h>


namespace OpenSim { 

class Function;

//=============================================================================
//=============================================================================
/**
 * FunctionBasedController is a concrete Controller that specifies functions that 
 * prescribe the control values of its actuators as a function of time.
 *
 * @authors  Ajay Seth, Ayman Habib
 */
//=============================================================================

class OSIMSIMULATION_API FunctionBasedController : public Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(FunctionBasedController, Controller);

//=============================================================================
// DATA
//=============================================================================
public:
    /** FunctionSet of prescribed controls associated with each actuator  */
    OpenSim_DECLARE_PROPERTY(ControlFunctions, FunctionSet,
        "Functions (one per control) describing the controls for actuators"
        "specified for this controller." );
    /** Output is a list of "control" outputs, available at stage "Time" 
    (can depend only on time) and obtained by calling  getControlAtTime */
    OpenSim_DECLARE_LIST_OUTPUT(control, SimTK::Vector, getControlAtTime,
        SimTK::Stage::Time);

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION AND DESTRUCTION
    //--------------------------------------------------------------------------
public:
    /** Default constructor */
    FunctionBasedController();


    /** Destructor */
    virtual ~FunctionBasedController();

    //--------------------------------------------------------------------------
    // CONTROL
    //--------------------------------------------------------------------------
    /**
     * Compute the control values for all actuators under the control of this
     * Controller.
     *
     * @param s             system state 
     * @param controls      model controls  
     */
    void computeControls(const SimTK::State& s, 
                         SimTK::Vector& controls) const override;

    /**
     *  Assign a prescribe control function for the desired actuator identified 
     *  by its index. Controller takes ownership of the function.
     *  @param index                the actuator's index in the controller's set
     *  @param prescribedFunction   the actuator's control function
     */
    void prescribeControlForActuator(int index, Function *prescribedFunction);

    /**
     *  Assign a prescribe control function for the desired actuator identified
     *  by its name. Controller takes ownership of the function.
     *  @param actName                the actuator's name in the controller's set
     *  @param prescribedFunction     the actuator's control function
     */
    void prescribeControlForActuator(const std::string actName,
                                     Function *prescribedFunction);

    /**
     * Return a List output with one vector of control values per Actuator. Size of the List/Vector
     * is the same as the number of Actuators controlled by this controller, each entry is a Vector of 
     * size 1 in case of Scalar control*/
    SimTK::Vector getControlAtTime(const SimTK::State& s, const std::string& actuatorName) const;

    /**
     * Populate channels of "control" output, one per actuator
     */
    void extendFinalizeFromProperties() override;

private:
    // construct and initialize properties
    void constructProperties() override;

    // This method sets all member variables to default (e.g., NULL) values.
    void setNull();

//=============================================================================
};  // END of class FunctionBasedController

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_FUNCTION_BASED_CONTROLLER_H_


