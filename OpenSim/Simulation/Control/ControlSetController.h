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

//=============================================================================
//=============================================================================
/**
 * ControllerSetController that simply assigns controls from a ControlSet
 * @author Jack Middleton, Ajay Seth 
 * @version 1.0
 */

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



namespace OpenSim { 

class ControlSet;

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
	 * @param aFileName The name of the XML file in which this Controller is
	 * defined.
	 * @param aUpdateFromXMLNode A flag indicating whether or not to call
	 * updateFromXMLNode(SimTK::Xml::Element& aNode) from this constructor.  
	 */
	ControlSetController(const std::string &aFileName, bool aUpdateFromXMLNode = true);

	/**
	 * Copy constructor.  This constructor is called by any code that contains
	 * a command of the form "Controller newController(oldController);".
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
	 * Assignment operator.  
	 */
	ControlSetController& operator=(const ControlSetController &aController);

#endif

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


