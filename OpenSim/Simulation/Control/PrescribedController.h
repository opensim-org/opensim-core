// PrescribedController.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2009, Stanford University. All rights reserved. 
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
*
* Author:  Ajay Seth
*/
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


class OSIMSIMULATION_API PrescribedController : public Controller
{

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
	 * Constructor to create and add to a Model.
	 * @param aModel The model that has actuators being controlled by this Controller.
	 */
	PrescribedController(Model& aModel);

	/**
	 * Constructor from an XML Document.
	 *
	 * @param aFileName: The XML file in which this Controller is defined
	 * @param aUpdateFromXMLNode: A flag indicating whether or not to call
	 * updateFromXMLNode() from this constructor.
	 */
	PrescribedController(const std::string &aFileName, bool aUpdateFromXMLNode = true);

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

    virtual Object* copy() const;

	OPENSIM_DECLARE_DERIVED(PrescribedController, Controller);

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


