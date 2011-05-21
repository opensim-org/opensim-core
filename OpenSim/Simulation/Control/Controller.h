// Controller.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

#ifndef _Controller_h_
#define _Controller_h_

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

//=============================================================================
//=============================================================================
/**
 * Controller specifies the interface of a controller in OpenSim 
  *
 * @author Ajay Seth, Frank C. Anderson, Chand T. John, Samuel R. Hamner 
 * @version 2.0
 */

namespace OpenSim { 

// Forward declarations of classes that are used by the controller implementation
class Model;
class Manager;
class Storage;
class ForceSet;

// A Controller is a ModelComponent
// The interface (defined herein) is  exported by the osimSimulation dll
class OSIMSIMULATION_API Controller : public ModelComponent
{
//=============================================================================
// DATA
//=============================================================================
// These are the member variables of the Controller class.  
protected:
     PropertyBool _isControllerEnabledProp;
	 bool _isControllerEnabled;

    /** number of controls this controller computes */
    int _numControls;

    /** list of actuator names to be controlled */
    PropertyStrArray _actuatorNameListProp;
    Array<std::string>& _actuatorNameList;

    /** set of actuators that the controller controls */ 
	Set<Actuator> _actuatorSet;


//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION AND DESTRUCTION
	//--------------------------------------------------------------------------
public:

	/** Default constructor. */
	Controller();

	/** A convenience constructor.
	 * @param aModel The model that is to be controlled by this Controller. */
	Controller(Model& aModel);

	/** Constructor from an XML Document.
	 *
	 * @param aFileName The XML file in which this a controller is defined.
	 * @param aUpdateFromXMLNode A flag indicating whether or not to call
	 * updateFromXMLNode() from this constructor.  This method is only necessary
	 * if changes to the XML format (i.e. tag names) are made and future
	 * versions must convert old syntax to the latest.
	 */
	Controller(const std::string &aFileName, bool aUpdateFromXMLNode = true);

	/** Copy constructor.  
	 * @param aController The controller to be copied.
	 */
	Controller(const Controller &aController);

	/** Default destructor.	 */
	virtual ~Controller();

	/** Public model component interface to obtain the number of state variables
	    introduced by the controller. */
	virtual int getNumStateVariables() const { return 0; };

protected:

	/** Copy the member variables of the specified controller.  This method is
	 * called by the copy constructor of the Controller class.
	 *
	 * @param aController The controller whose data is to be copied.
	 */
	void copyData(const Controller &aController);

	/** Model component interface that permits the controller to be "wired" up
	   to its actuators. Subclasses can override to perform additional setup. */
	virtual void setup(Model& model);  

	/** Model component interface that creates underlying computational components
	    in the SimTK::MultibodySystem. This includes adding states, creating 
		measures, etc... required by the controller. */
	virtual void createSystem(SimTK::MultibodySystem& system) const;

	/** Model component interface to initialize states introduced by the controller */
	virtual void initState( SimTK::State& s) const {};

	/** Model component interface to initialize defualt values of the controller
	    from/based on the given state. */
	virtual void setDefaultsFromState(const SimTK::State& state) {};

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:

#ifndef SWIG

	/**
	 * Assignment operator.  This method is called automatically whenever a
	 * command of the form "controller1 = controller2;" is made, where both
	 * controller1 and controller2 are both of type Controller.  Although
	 * Controller cannot be instantiated directly, a subclass of Controller
	 * could implement its own operator= method that calls Controller's
	 * operator= method.  If the subclass does not implement its own operator=
	 * method, then when a command of the form "controller1 = controller2" is
	 * made, where both controller1 and controller2 are instants of the
	 * subclass, the Controller class's operator= method will be called
	 * automatically.
	 *
	 * @param aController The controller to be copied.
	 * @return Reference to the altered object.
	 */
	Controller& operator=(const Controller &aController);

#endif

	//--------------------------------------------------------------------------
	// Controller Interface
	//--------------------------------------------------------------------------
	/** Get whether or not this controller is on.
	 *
	 * @return true if on, false if off.
	 */
	bool getIsEnabled() const;

	/** Turn this controller on or off.
	 *
	 * @param aTrueFalse Turns controller on if "true" and off if "false".
	 */
	void setIsEnabled(bool aTrueFalse);

	/** Compute the control for actuator
	 *  This method defines the behavior for any concrete controller 
	 *  and therefore must be implemented by concrete subclasses.
	 *
	 * @param s system state 
	 * @param controls writable model controls
	 */
	virtual void computeControls(const SimTK::State& s, SimTK::Vector &controls) const = 0;

	/** replace the current set of actuators with the provided set */
    virtual void setActuators( Set<Actuator>& actuators );
	/** add to the current set of actuators */
	void addActuator(Actuator *actuator);
	/** get a const reference to the current set of actuators */
	virtual const Set<Actuator>& getActuatorSet() const;
	/** get a writable reference to the set of actuators for this controller */
	virtual Set<Actuator>& updActuators();
	/** get the names of the actuators being controlled */
	virtual const Array<std::string>& getActuatorNames() const { return _actuatorNameList; }


   /** 
    * return the min an max times that a controller knows how to supply controlls for 
    */ 
	virtual double getFirstTime() const;
	virtual double getLastTime() const;


private:
	// This method sets all pointer member variables to NULL. 
	void setNull();
	// Connect properties to local pointers.  */
	virtual void setupProperties();

	//friend class ControlSet;
	friend class ControllerSet;

//=============================================================================
};	// END of class Controller

}; //namespace
//=============================================================================
//=============================================================================

#endif // __Controller_h__


