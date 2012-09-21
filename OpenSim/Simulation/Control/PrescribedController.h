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
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifndef _PrescribedController_h_
#define _PrescribedController_h_

//============================================================================
// INCLUDE
//============================================================================
#include "Controller.h"
#include "OpenSim/Common/PropertyObj.h"
#include "OpenSim/Common/FunctionSet.h"

//=============================================================================
//=============================================================================
/**
 * PrescribedController is a concrete class that uses functions to prescribe
 * control values to actuators in its set of actuators.
 *
 * @author  Ajay Seth
 * @version 1.0
 */

namespace OpenSim { 


class OSIMSIMULATION_API PrescribedController : public Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(PrescribedController, Controller);

//=============================================================================
// DATA
//=============================================================================

protected:

	/*
	 * Functions of controls associated with each actuator
	 */
	PropertyObj _prescribedControlFunctionsProp;
	FunctionSet &_prescribedControlFunctions;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION AND DESTRUCTION
	//--------------------------------------------------------------------------
public:
	/**
	 * Default constructor.
	 */
	PrescribedController();

	/**
	 * Copy constructor. 
	 *
	 * @param aController The controller to be copied.
	 */
	PrescribedController(const PrescribedController &PrescribedController);

	/**
	 * Destructor.  This method should be a member of any subclass of the
	 * Controller class.  It will be called automatically whenever an
	 * instance of the subclass is deleted from memory.
	 */
	virtual ~PrescribedController();

private:
	/**
	 * This method sets all member variables to default (e.g., NULL) values.
	 */
	void setNull();

protected:

	/**
	 * Connect properties to local pointers.  
	 */
	virtual void setupProperties();

	/**
	 * Copy the member variables of the specified controller.  This method is
	 * called by the copy constructor of the Controller class.
	 *
	 * @param aController The controller whose data is to be copied.
	 */
	void copyData(const PrescribedController &aController);


	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:

#ifndef SWIG

	/**
	 * Assignment operator.  
	 *
	 * @param aController The controller to be copied.
	 * @return Reference to the altered object.
	 */
	PrescribedController& operator=(const PrescribedController &PrescribedController);

#endif

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------

	
	/**
	 * Set this class's pointer to the set containing functions of the
	 * desired controls to be executed by this Controller.
	 *
	 * @param functions Pointer to a Set of Functions- one per actuator
	 */
	//void setPrescribedControlFunctions(Array<OpenSim::Function> *functions);
	//const Array<OpenSim::Function>& getPrescribedControlFunctions() const; 

	// ON/OFF

	//--------------------------------------------------------------------------
	// CONTROL
	//--------------------------------------------------------------------------
	/**
	 * Compute the control values for all actuators under the control of this
	 * Controller
	 *
	 * @param s system state 
	 * @param model controls  
	 */
	virtual void computeControls(const SimTK::State& s, SimTK::Vector& controls) const;

	/**
	 *	Assign a prescribe control function for the desired actuator identified by its index.
	 *  @param index,  the actuator's index in the controller's actuator set
	 *  @param prescribedFunction, the actuator's control function
	 */
	void prescribeControlForActuator(int index, OpenSim::Function *prescribedFunction);

	/**
	 *	Assign a prescribe control function for the desired actuator identified by its name.
	 *  @param name,  the actuator's name to be found in the controller's actuator set
	 *  @param prescribedFunction, the actuator's control function
	 */
	void prescribeControlForActuator(const std::string actName, OpenSim::Function *prescribedFunction);


//=============================================================================
};	// END of class PrescribedController

}; //namespace
//=============================================================================
//=============================================================================

#endif // __PrescribedController_h__


