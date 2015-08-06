#ifndef __ControllerSet_h__
#define __ControllerSet_h__
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  ControllerSet.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Simulation/Control/Controller.h>
#include <OpenSim/Simulation/Model/ModelComponentSet.h>
#include "SimTKsimbody.h"

#include <memory>

namespace OpenSim {

class Model;
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
    ControllerSet() {}
    ControllerSet(Model& model);
    ControllerSet(const ControllerSet &aControllerSet);
    ControllerSet(Model& model, const std::string &aFileName,  bool aUpdateFromXMLNode = true);
#ifndef SWIG
    ~ControllerSet() override = default;
#endif

    void copyData(const ControllerSet &aAbsControllerSet);


    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
#ifndef SWIG
    ControllerSet& operator=(const ControllerSet &aSet);
#endif
    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------

    bool set(int aIndex, Controller *aController);
    bool addController(Controller *aController);


    void constructStorage();
    void storeControls( const SimTK::State& s, int step );
    void printControlStorage( const std::string& fileName) const;
    void setActuators(Set<Actuator>& actuators);

    void setDesiredStates( Storage* yStore); 

    // Controller interface
    virtual void computeControls(const SimTK::State& s, SimTK::Vector &controls) const; 

    virtual void printInfo() const;

private:

    std::unique_ptr<Storage> _controlStore;

    // Set of actuators controlled by the set of controllers.
    SimTK::ReferencePtr<Set<Actuator> > _actuatorSet;
//=============================================================================
};  // END of class ControllerSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim


#endif // __ControllerSet_h__


