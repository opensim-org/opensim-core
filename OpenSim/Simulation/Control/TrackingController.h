#ifndef OPENSIM_TRACKING_CONTROLLER_H_
#define OPENSIM_TRACKING_CONTROLLER_H_

/* -------------------------------------------------------------------------- *
 *                       OpenSim:  TrackingController.h                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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



//============================================================================
// INCLUDE
//============================================================================
#include "Controller.h"

//=============================================================================
//=============================================================================

namespace OpenSim { 

class TrackingTask;
class Storage;

/**
 * TrackingController is an abstract class from which all tracking type of
 * controllers should be derived. This class implements convenience methods
 * to construct desired trajectories of desired model state values, like joint 
 * angles, from Storage (file) or user-supplied functions and provides methods 
 * for obtaining the error between model and desired state values.
 *
 * Derive classes need only to implement the tracking control law based on 
 * the error signals computed by this base class.
 * 
 * @author  Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API TrackingController : public Controller
{

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
     * Destructor.  
     */
    virtual ~TrackingController();

    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    /**
     * %Set this class's pointer to the storage object containing
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
    //void computeControls(const SimTK::State& s, SimTK::Vector& controls) const =0;

private:
    // A "private" method is one that can be called only by this class,
    // and not even by subclasses of this class.

    /**
     * This method sets all member variables to default (e.g., NULL) values.
     */
    void setNull();




//=============================================================================
// DATA
//=============================================================================

private:
    /**
     *   storage object containing the desired trajectory
     */
    mutable SimTK::ReferencePtr<const Storage> _desiredStatesStorage;


    friend class ControllerSet;

//=============================================================================
};  // END of class Controller

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_TRACKING_CONTROLLER_H_


