// TrackingController.h
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

#ifndef _TrackingController_h_
#define _TrackingController_h_

//============================================================================
// INCLUDE
//============================================================================
#include "Controller.h"

//=============================================================================
//=============================================================================
/**
 * TrackingController is an abstract class from which all tracking type of
 * controllers should be derived. This class implements convenience methods
 * to construct dsired trajectories of desired model state values, like joint 
 * angles, from Storage (file) or user-supplied functions and provides methods 
 * for obtaining the error between model and desired state values.
 *
 * Derive classes need only to implement the tracking control law based on 
 * the error signals computed by this base class.
 * 
 *
 * @author  Ajay Seth
 * @version 1.0
 */

namespace OpenSim { 

class TrackingTask;

class OSIMSIMULATION_API TrackingController : public Controller
{

//=============================================================================
// DATA
//=============================================================================

protected:
	/**
	 *   storage object containing the desired trajectory
     */
	const Storage* _desiredStatesStorage;

	/*
	 * Tracking tasks define the relative weighting and gains on the
	 * tracking errors in computing the controls.
	 */
	Array<double> *_trackingTasks;

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
	TrackingController();

	/**
	 * Another constructor.
	 *
	 * @param aModel The model that has actualtors being controlled by this Controller.
	 */
	TrackingController(Model& aModel);

	/**
	 * Constructor from an XML Document.
	 *
	 * @param aFileName: The XML file in which this Controller is defined
	 * @param aUpdateFromXMLNode: A flag indicating whether or not to call
	 * updateFromXMLNode() from this constructor.
	 */
	TrackingController(const std::string &aFileName, bool aUpdateFromXMLNode = true);

	/**
	 * Copy constructor. 
	 *
	 * @param aController The controller to be copied.
	 */
	TrackingController(const TrackingController &TrackingController);

	/**
	 * Destructor.  This method should be a member of any subclass of the
	 * Controller class.  It will be called automatically whenever an
	 * instance of the subclass is deleted from memory.
	 */
	virtual ~TrackingController();

private:
	// A "private" method is one that can be called only by this class,
	// and not even by subclasses of this class.

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
	void copyData(const TrackingController &aController);


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
	TrackingController& operator=(const TrackingController &TrackingController);

#endif

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------

	
	/**
	 * Set this class's pointer to the storage object containing
	 * desired model states to point to the storage object passed into
	 * this method.  This method is currently implemented only by the
	 * CorrectionController class, which is a subclass of Controller.
	 *
	 * @param aYDesStore Pointer to a Storage object containing the
	 * desired states of the model for the controller to achieve during
	 * simulation.
	 */
	virtual void setDesiredStatesStorage(const Storage* aYDesStore);
    virtual const Storage& getDesiredStatesStorage() const; 

	// ON/OFF

	//--------------------------------------------------------------------------
	// CONTROL
	//--------------------------------------------------------------------------

	/**
	 *
	 * Note that this method is "pure virtual", which means that the Controller
	 * class does not implement it, and that subclasses must implement it.
	 *
	 * @param s system state 
	 * @param rControlSet Control set used for the simulation.  This method
	 * alters the control set in order to control the simulation.
	 */
	virtual double computeControl(const SimTK::State& s, int index) const = 0;

   friend class ControllerSet;

//=============================================================================
};	// END of class Controller

}; //namespace
//=============================================================================
//=============================================================================

#endif // __Controller_h__


