#ifndef OPENSIM_CONTROLLER_H_
#define OPENSIM_CONTROLLER_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Controller.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Frank C. Anderson, Chand T. John, Samuel R. Hamner   *
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

//============================================================================
// INCLUDE
//============================================================================
// These files contain declarations and definitions of variables and methods
// that will be used by the Controller class.
#include <OpenSim/Simulation/Model/Actuator.h>
#include <OpenSim/Common/Set.h>

namespace OpenSim { 

// Forward declarations of classes that are used by the controller implementation
class Model;

/**
 * Controller is an abstract ModelComponent that defines the interface for   
 * an OpenSim Controller. A controller computes and sets the values of the  
 * controls for the actuators under its control.
 * The defining method of a Controller is its computeControls() method.
 * @see computeControls()
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API Controller : public ModelComponent {
OpenSim_DECLARE_ABSTRACT_OBJECT(Controller, ModelComponent);

public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations 
    These are the serializable properties associated with a Controller. **/
    /**@{**/

	OpenSim_DECLARE_PROPERTY(isDisabled, bool, 
		"Flag (true or false) indicating whether or not the controller is disabled." );

	OpenSim_DECLARE_LIST_PROPERTY(actuator_list, std::string,
		"The list of model actuators that this controller will control."
        "The keyword ALL indicates the controller will controll all the acuators in the model" );

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION AND DESTRUCTION
	//--------------------------------------------------------------------------
public:

	/** Default constructor. */
	Controller();

	// Uses default (compiler-generated) destructor, copy constructor and copy 
    // assignment operator.

	//--------------------------------------------------------------------------
	// Controller Interface
	//--------------------------------------------------------------------------
	/** Get whether or not this controller is disabled.
	 * @return true when controller is disabled.
	 */
	bool isDisabled() const;

	/** Disable this controller.
	 * @param disableFlag Disable if true.
	 */
	void setDisabled(bool disableFlag);

	/** replace the current set of actuators with the provided set */
    void setActuators(const Set<Actuator>& actuators );
	/** add to the current set of actuators */
	void addActuator(const Actuator& actuator);
	/** get a const reference to the current set of actuators */
	const Set<Actuator>& getActuatorSet() const;
	/** get a writable reference to the set of actuators for this controller */
	Set<Actuator>& updActuators();

	/** Compute the control for actuator
	 *  This method defines the behavior for any concrete controller 
	 *  and therefore must be implemented by concrete subclasses.
	 *
	 * @param s			system state 
	 * @param controls	writable model controls (all actuators)
	 */
	virtual void computeControls(const SimTK::State& s,
								 SimTK::Vector &controls) const = 0;

	int getNumControls() const {return _numControls;}

protected:

	/** Model component interface that permits the controller to be "wired" up
	   to its actuators. Subclasses can override to perform additional setup. */
	void connectToModel(Model& model) OVERRIDE_11;  

	/** Model component interface that creates underlying computational components
	    in the SimTK::MultibodySystem. This includes adding states, creating 
		measures, etc... required by the controller. */
	void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;

	/** Only a Controller can set its number of controls based on its actuators */
	void setNumControls(int numControls) {_numControls = numControls; }

private:
	// number of controls this controller computes 
    int _numControls;

	// the (sub)set of Model actuators that this controller controls */ 
	Set<Actuator> _actuatorSet;

	// construct and initialize properties
	void constructProperties();

	//friend class ControlSet;
	friend class ControllerSet;

//=============================================================================
};	// END of class Controller

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_CONTROLLER_H_


