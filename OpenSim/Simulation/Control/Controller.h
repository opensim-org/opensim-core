#ifndef OPENSIM_CONTROLLER_H_
#define OPENSIM_CONTROLLER_H_

// Controller.h
/*
* Copyright (c)  2010, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* 
 * Author: Ajay Seth, Frank C. Anderson, Chand T. John, Samuel R. Hamner
 */


//============================================================================
// INCLUDE
//============================================================================
// These files contain declarations and definitions of variables and methods
// that will be used by the Controller class.
#include <OpenSim/Simulation/Model/Actuator.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Common/Set.h>

namespace OpenSim { 

// Forward declarations of classes that are used by the controller implementation
class Model;

/**
 * Controller is an abstract ModelComponent that defines the Controller  
 * in OpenSim.
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

    /** number of controls this controller computes */
    int _numControls;

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

	/** Compute the control for actuator
	 *  This method defines the behavior for any concrete controller 
	 *  and therefore must be implemented by concrete subclasses.
	 *
	 * @param s			system state 
	 * @param controls	writable model controls (all actuators)
	 */
	virtual void computeControls(const SimTK::State& s, SimTK::Vector &controls) const = 0;

	/** replace the current set of actuators with the provided set */
    void setActuators(const Set<Actuator>& actuators );
	/** add to the current set of actuators */
	void addActuator(const Actuator& actuator);
	/** get a const reference to the current set of actuators */
	const Set<Actuator>& getActuatorSet() const;
	/** get a writable reference to the set of actuators for this controller */
	Set<Actuator>& updActuators();

protected:

	/** Model component interface that permits the controller to be "wired" up
	   to its actuators. Subclasses can override to perform additional setup. */
	void connectToModel(Model& model) OVERRIDE_11;  

	/** Model component interface that creates underlying computational components
	    in the SimTK::MultibodySystem. This includes adding states, creating 
		measures, etc... required by the controller. */
	void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;

private:
	// the (sub)set of Model actuators that this controller controls */ 
	Set<Actuator> _actuatorSet;

	// construct and initialize properties
	virtual void constructProperties();

	//friend class ControlSet;
	friend class ControllerSet;

//=============================================================================
};	// END of class Controller

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_CONTROLLER_H_


