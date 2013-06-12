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

#include "Controller.h"
#include <OpenSim/Common/FunctionSet.h>


namespace OpenSim { 

class Function;

//=============================================================================
//=============================================================================
/**
 * PrescribedController is a concrete Controller that specifies functions that 
 * prescribe the control values of its actuators as a function of time.
 *
 * @author  Ajay Seth
 */
//=============================================================================

class OSIMSIMULATION_API PrescribedController : public Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(PrescribedController, Controller);

//=============================================================================
// DATA
//=============================================================================
public:
	/** FunctionSet of prescribed controls associated with each actuator  */
	OpenSim_DECLARE_PROPERTY(ControlFunctions, FunctionSet,
		"Functions (one per control) describing the controls for actuators"
		"specified for this controller." );

	/** (Optional) prescribed controls from a storage file  */
	OpenSim_DECLARE_OPTIONAL_PROPERTY(controls_file, std::string,
		"Controls storage (.sto) file containing controls for individual "
		"actuators in the model. Column labels must match actuator names.");

	/** (Optional) interpolation method for controls in storage.  */
	OpenSim_DECLARE_OPTIONAL_PROPERTY(interpolation_method, int,
		"Interpolate the controls file data using piecewise: '0-constant', "
		"'1-linear', '3-cubic' or '5-quintic' functions.");

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION AND DESTRUCTION
	//--------------------------------------------------------------------------
public:
	/** Default constructor */
	PrescribedController();

	/** Convenience constructor get controls from file
	 * @param controlsFileName  string containing the controls storage (.sto) 
	 * @param interpMethodType	int 0-constant, 1-linear, 3-cubic, 5-quintic
	 *                          defaults to linear.
	 */
	PrescribedController(const std::string& controlsFileName, 
						 int interpMethodType = 1);

	/** Destructor */
	virtual ~PrescribedController();

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
						 SimTK::Vector& controls) const OVERRIDE_11;

	/**
	 *	Assign a prescribe control function for the desired actuator identified 
	 *  by its index. Controller takes ownership of the function.
	 *  @param index                the actuator's index in the controller's set
	 *  @param prescribedFunction   the actuator's control function
	 */
	void prescribeControlForActuator(int index, Function *prescribedFunction);

	/**
	 *	Assign a prescribe control function for the desired actuator identified
	 *  by its name. Controller takes ownership of the function.
	 *  @param actName                the actuator's name in the controller's set
	 *  @param prescribedFunction     the actuator's control function
	 */
	void prescribeControlForActuator(const std::string actName,
									 Function *prescribedFunction);

protected:
	/** Model component interface */
	void connectToModel(Model& model) OVERRIDE_11;
private:
	// construct and initialize properties
	void constructProperties();

	// utility
	Function* createFunctionFromData(const std::string& name,
		const Array<double>& time, const Array<double>& data);

	// This method sets all member variables to default (e.g., NULL) values.
	void setNull();

//=============================================================================
};	// END of class PrescribedController

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_PRESCRIBED_CONTROLLER_H_


