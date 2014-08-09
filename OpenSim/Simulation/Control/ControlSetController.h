/* -------------------------------------------------------------------------- *
 *                      OpenSim:  ControlSetController.h                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Jack Middleton, Ajay Seth                                       *
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

class OSIMSIMULATION_API ControlSetController : public Controller {
    OpenSim_DECLARE_CONCRETE_OBJECT(ControlSetController, Controller);

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

    const ControlSet *getControlSet() {
        return _controlSet;
    }
    ControlSet *updControlSet() {
        return _controlSet;
    }

    void setControlSet(ControlSet *aControlSet) {
        _controlSet = aControlSet;
    }



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

    // for any post XML deserialization intialization
    void connectToModel(Model& model) override;

    // for adding any components to the model
    void addToSystem( SimTK::MultibodySystem& system) const override;

    // for any intialization requiring a state or the complete system
    void initStateFromProperties( SimTK::State& s) const override;

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
     * @param s         system state
     * @param controls  return control values
     */
    virtual void computeControls(const SimTK::State& s, SimTK::Vector& controls) const;

    virtual void setControlSetFileName( const std::string&  controlSetFileName );
    const std::string& getControlSetFileName() const {
        return _controlsFileName;
    }

    /**
     *   return the min an max times that a controller knows how to supply controlls for
     */
    double getFirstTime() const;
    double getLastTime() const;

//=============================================================================
};	// END of class ControlSetController

}; //namespace
//=============================================================================
//=============================================================================

#endif // __Control_Set_Controller_h__


