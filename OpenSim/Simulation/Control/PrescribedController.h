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
 * Contributor(s): Nicholas Bianco                                            *
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
 * The control functions are specified in the `ControlFunctions` property. The
 * number and order of functions must match the number and order of actuators 
 * connected to the controller. Use `prescribeControlForActuator()` to assign a
 * control function to an actuator based on the name or path of the actuator;
 * after connecting the controller to the model, the added control function will
 * be placed at the correct index in the `ControlFunctions` property.
 *
 * A controls storage file can be specified in the `controls_file` property.
 * Each column must be either the name or path of an actuator in the model. If
 * the actuator name is used as the column label, the first actuator with a 
 * matching name will be connected to the controller and assigned a control 
 * function based on the column data. Using actuator paths in the column labels
 * is recommended to avoid ambiguity. Finally, any actuators with existing 
 * control functions will be ignored when setting controls from file.
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
        "specified for this controller. The control function set must match "
        "the number and order of connected actuators." );

    OpenSim_DECLARE_OPTIONAL_PROPERTY(controls_file, std::string,
        "Controls storage (.sto) file containing controls for individual "
        "actuators in the model. Each column label must be either the name "
        "or path to an actuator in the model.");

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
     *  or the absolute path to the actuator in the model.
     *  @param actuLabel            label for the actuator in the controller
     *  @param prescribedFunction   the actuator's control function
     *
     *  @note As of OpenSim 4.6, PrescribedController no longer takes ownership
     *        of the passed in Function and instead makes a copy.
     */
    void prescribeControlForActuator(const std::string& actuLabel,
                                     const Function& prescribedFunction);

    [[deprecated("Use prescribeControlForActuator(const std::string&, const Function&) instead")]]
    void prescribeControlForActuator(int index, Function* prescribedFunction) {
        OPENSIM_THROW_FRMOBJ(Exception,
            "PrescribedController::prescribeControlForActuator(int, Function*) "
            "is deprecated. Use prescribeControlForActuator(const std::string&, "
            "const Function&) instead.");
    }

protected:
    // MODEL COMPONENT INTERFACE
    void extendConnectToModel(Model& model) override;
    void updateFromXMLNode(SimTK::Xml::Element& node,
                           int versionNumber) override;

private:
    // Construct and initialize properties.
    void constructProperties();

    // Utility functions.
    std::unique_ptr<Function> createFunctionFromData(const std::string& name,
        const Array<double>& time, const Array<double>& data) const;
    int getActuatorIndexFromLabel(const std::string& actuLabel) const;

    // This method sets all member variables to default (e.g., NULL) values.
    void setNull();

    // Member variables.
    std::unordered_map<std::string, int> _actuLabelsToControlFunctionIndexMap;

};  // class PrescribedController

} // namespace OpenSim

#endif // OPENSIM_PRESCRIBED_CONTROLLER_H_


