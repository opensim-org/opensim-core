// ControlSetController.h
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
*/

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson, Chand T. John, Samuel R. Hamner, Ajay Seth
 */
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The entire definition of the Controller class is contained inside this
// #ifndef-#define-#endif _Controller_h_ block.  This is done to ensure that,
// if someone were to include Controller.h more than once into a single file
// (such as including Controller.h as well as including another file that also
// includes Controller.h), then the Controller class will not be defined more
// than once.  "#ifndef" means "if not defined."
#ifndef _Control_Set_Controller_h_
#define _Control_Set_Controller_h_

//============================================================================
// INCLUDE
//============================================================================
// These files contain declarations and definitions of variables and methods
// that will be used by the Controller class.
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Set.h>
#include <OpenSim/Simulation/Model/Actuator.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyObjPtr.h>
#include "Controller.h"
#include "SimTKsimbody.h"

//=============================================================================
//=============================================================================
/**
 * ControllerSetController is a controller that uses a ControlSet
 * to supply controls to actuators
 * @author Jack Middleton 
 * @version 1.0
 */

namespace OpenSim { 

class ControlSet;

// The entire definition of the ControlSetController class is contained inside
// this code block.  The identifier OSIMSIMULATION_API tells C++ that
// the Controller class will be part of the library files exported by
// the osimSimulation project.  The terms "public Object" tell C++
// that the ControlSetController class is a subclass (child) of the Object class
// in OpenSim.
class OSIMSIMULATION_API ControlSetController : public Controller
{

//=============================================================================
// DATA
//=============================================================================
protected:
    ControlSet* _controlSet;

    /** Name of the controls file. */
    PropertyStr _controlsFileNameProp;
    std::string &_controlsFileName;



//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION AND DESTRUCTION
	//--------------------------------------------------------------------------
public:
	ControlSetController();

	/**
	 * Constructor from an XML Document.
	 *
	 * @param aFileName The name of the XML file in which this Controller is
	 * defined.
	 * @param aUpdateFromXMLNode A flag indicating whether or not to call
	 * updateFromXMLNode() from this constructor.  If true, the method will
	 * be called from this class.  Typically, the flag should be true for this
	 * class, but in the member initializer list for this constructor, this
	 * class's parent class's constructor with the same parameters will be
	 * called, but with aUpdateFromXMLNode set to false.
	 */
	ControlSetController(const std::string &aFileName, bool aUpdateFromXMLNode = true);

	/**
	 * Copy constructor.  This constructor is called by any code that contains
	 * a command of the form "Controller newController(oldController);".
	 *
	 * @param aController The controller to be copied.
	 */
	ControlSetController(const ControlSetController &aController);

	/**
	 * Destructor.  This method should be a member of any subclass of the
	 * Controller class.  It will be called automatically whenever an
	 * instance of the subclass is deleted from memory.
	 */
	virtual ~ControlSetController();
    virtual Object* copy() const;

	
	const ControlSet *getControlSet() {return _controlSet;} 
	ControlSet *updControlSet() {return _controlSet;}

	void setControlSet(ControlSet *aControlSet) {_controlSet = aControlSet;}


	
private:
	// A "private" method is one that can be called only by this class,
	// and not even by subclasses of this class.

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
	void copyData(const ControlSetController &aController);

	// for any post XML deseraialization intialization
	virtual void setup(Model& model);

	// for adding any components to the model
	virtual void createSystem( SimTK::MultibodySystem& system) const; 

	// for any intialization requiring a state or the complete system 
	virtual void initState( SimTK::State& s) const;

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
	ControlSetController& operator=(const ControlSetController &aController);

#endif

	//--------------------------------------------------------------------------
	// CONTROL
	//--------------------------------------------------------------------------

	/**
	 * Compute the control value for an acuator 
	 *
	 * @param s system state 
	 * @param index  the id the controller uses to determin which actuator 
	 *   
	 */
     virtual double computeControl(const SimTK::State& s, int index) const;

    virtual void setActuators( Set<Actuator>& actuators );

    virtual void setControlSetFileName( const std::string&  controlSetFileName );
	const std::string& getControlSetFileName() const {
		return _controlsFileName;
	}

   /** 
    *   return the min an max times that a controller knows how to supply controlls for 
    */ 
   virtual double getFirstTime() const;
   virtual double getLastTime() const;

//=============================================================================
};	// END of class ControlSetController

}; //namespace
//=============================================================================
//=============================================================================

#endif // __Control_Set_Controller_h__


