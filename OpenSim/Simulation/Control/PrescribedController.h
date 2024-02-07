#ifndef OPENSIM_PRESCRIBED_CONTROLLER_H_
#define OPENSIM_PRESCRIBED_CONTROLLER_H_
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  PrescribedController.h                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2024 Stanford University and the Authors                *
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

#include "Controller.h"
#include <OpenSim/Common/FunctionSet.h>


namespace OpenSim { 

class Function;

/**
 * PrescribedController is a concrete Controller that specifies functions that 
 * prescribe the control values of its actuators as a function of time.
 *
 * @note Prior to OpenSim 4.6, PrescribedController support setting a prescribed
 *       control based on the actuator's index in the `ControlFunctions`
 *       property. This interface is deprecated and will be removed in a future
 *       release.
 *
 * @author  Ajay Seth
 */
class OSIMSIMULATION_API PrescribedController : public Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(PrescribedController, Controller);

public:
//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_PROPERTY(ControlFunctions, FunctionSet,
        "Functions (one per control) describing the controls for actuators "
        "specified for this controller." );

    OpenSim_DECLARE_OPTIONAL_PROPERTY(controls_file, std::string,
        "Controls storage (.sto) file containing controls for individual "
        "actuators in the model. Each column label must be either the name "
        "of an actuator in the model's ForceSet or the absolute path to an "
        "actuator anywhere in the model.");

    OpenSim_DECLARE_OPTIONAL_PROPERTY(interpolation_method, int,
        "Interpolate the controls file data using piecewise: '0-constant', "
        "'1-linear', '3-cubic' or '5-quintic' functions.");

//=============================================================================
// METHODS
//=============================================================================

    // CONSTRUCTION AND DESTRUCTION
    /** Default constructor */
    PrescribedController();

    /** Convenience constructor get controls from file
     * @param controlsFileName  string containing the controls storage (.sto) 
     * @param interpMethodType  int 0-constant, 1-linear, 3-cubic, 5-quintic
     *                          defaults to linear.
     */
    PrescribedController(const std::string& controlsFileName, 
                         int interpMethodType = 1);

    /** Destructor */
    ~PrescribedController() override;

    // CONTROLLER INTERFACE
    /**
     * Compute the control values for all actuators under the control of this
     * Controller.
     *
     * @param s             system state 
     * @param controls      model controls  
     */
    void computeControls(const SimTK::State& s, 
                         SimTK::Vector& controls) const override;

    // GET AND SET
    /**
     *  Assign a prescribed control function for the desired actuator identified
     *  by the provided label. The label can be either the name of the actuator,
     *  or the absolute path to the actuator in the model. Controller takes
     *  ownership of the function.
     *  @param actuLabel            label for the actuator in the controller
     *  @param prescribedFunction   the actuator's control function
     */
    void prescribeControlForActuator(const std::string& actuLabel,
                                     Function* prescribedFunction);

    [[deprecated("Use prescribeControlForActuator(const std::string&, Function*) instead")]]
    void prescribeControlForActuator(int index, Function* prescribedFunction) {
        OPENSIM_THROW_FRMOBJ(Exception,
            "PrescribedController::prescribeControlForActuator(int, Function*) "
            "is deprecated. Use prescribeControlForActuator(const std::string&, "
            "Function*) instead.");
    }

protected:
    // MODEL COMPONENT INTERFACE
    void extendConnectToModel(Model& model) override;

private:
    // Construct and initialize properties.
    void constructProperties();

    // Utility functions.
    Function* createFunctionFromData(const std::string& name,
        const Array<double>& time, const Array<double>& data) const;
    int getActuatorIndexFromLabel(const std::string& actuLabel) const;

    // This method sets all member variables to default (e.g., NULL) values.
    void setNull();

    // Member variables.
    std::unordered_map<std::string, int> _actuLabelsToControlFunctionIndexMap;
    std::unordered_map<int, int> _actuIndexToControlFunctionIndexMap;

};  // class PrescribedController

} // namespace OpenSim

#endif // OPENSIM_PRESCRIBED_CONTROLLER_H_


