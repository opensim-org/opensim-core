/* -------------------------------------------------------------------------- *
 *                       OpenSim:  TrackingController.h                       *
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
class Storage;

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
	 * Copy constructor. 
	 *
	 * @param TrackingController The controller to be copied.
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
	 * @param TrackingController The controller to be copied.
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
	 * Controller interface
	 */
	virtual void computeControls(const SimTK::State& s, SimTK::Vector& controls) const =0;

	friend class ControllerSet;

//=============================================================================
};	// END of class Controller

}; //namespace
//=============================================================================
//=============================================================================

#endif // __Controller_h__


