#ifndef OPENSIM_CONTROLLER_SET_H_
#define OPENSIM_CONTROLLER_SET_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  ControllerSet.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2018 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Peter Loan, Jack Middleton, Ajay Seth        *
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

// INCLUDES
#include "OpenSim/Common/TimeSeriesTable.h"
#include <OpenSim/Simulation/Control/Controller.h>
#include <OpenSim/Simulation/Model/ModelComponentSet.h>

namespace OpenSim {

class Storage;

//=============================================================================
//=============================================================================
/**
 * A class for holding and managing a set of controllers for a model.
 *
 * @authors Jack Middleton, Ajay Seth 
 * @version 2.0
 */

//=============================================================================
class OSIMSIMULATION_API ControllerSet : public ModelComponentSet<Controller> {
OpenSim_DECLARE_CONCRETE_OBJECT(ControllerSet, ModelComponentSet<Controller>);

//=============================================================================
// METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    /** Use Super's constructors. @see ModelComponentSet */
    using Super::Super;

    void constructStorage();
    void storeControls( const SimTK::State& s, int step );
    void printControlStorage( const std::string& fileName) const;
    TimeSeriesTable getControlTable() const;
    void setActuators(Set<Actuator>& actuators);

    void setDesiredStates( Storage* yStore); 

    // Controller interface
    virtual void computeControls(const SimTK::State& s, SimTK::Vector &controls) const; 

    virtual void printInfo() const;

private:

    SimTK::ClonePtr<Storage> _controlStore;

    // Set of actuators controlled by the set of controllers.
    SimTK::ReferencePtr<Set<Actuator> > _actuatorSet;
//=============================================================================
};  // END of class ControllerSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim


#endif // OPENSIM_CONTROLLER_SET_H_


