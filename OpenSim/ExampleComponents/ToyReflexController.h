#ifndef OPENSIM_ToyReflexController_H_
#define OPENSIM_ToyReflexController_H_
/* -------------------------------------------------------------------------- *
 *                      OpenSim: ToyReflexController.h                        *
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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied    *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */


//============================================================================
// INCLUDE
//============================================================================
#include "osimExampleComponentsDLL.h"
#include "OpenSim/Simulation/Control/Controller.h"


namespace OpenSim { 

//=============================================================================
//=============================================================================
/**
 * ToyReflexController is a concrete controller that excites muscles in response
 * to muscle lengthening to simulate a simple stretch reflex. This controller 
 * is meant to serve as an example how to implement a controller in
 * OpenSim. It is intended for demonstrative purposes only. 
 *
 * @author  Ajay Seth
 */
class OSIMEXAMPLECOMPONENTS_API ToyReflexController : public Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(ToyReflexController, Controller);

public:
//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_PROPERTY(gain, double, 
        "Factor by which the stretch reflex is scaled." );

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION AND DESTRUCTION
    //--------------------------------------------------------------------------
    /** Default constructor. */
    ToyReflexController();

    // Uses default (compiler-generated) destructor, copy constructor and copy 
    // assignment operator.

    /** Convenience constructor 
    * @param gain       gain on the stretch response
    */
    ToyReflexController(double gain);

    /** Compute the controls for actuators (muscles)
     *  This method defines the behavior of the ToyReflexController 
     *
     * @param s         system state 
     * @param controls  writable model controls
     */
    void computeControls(const SimTK::State& s,
                         SimTK::Vector &controls) const override;


private:
    // Connect properties to local pointers.  */
    void constructProperties();
    // ModelComponent interface to connect this component to its model
    void extendConnectToModel(Model& aModel) override;

    //=========================================================================
};  // END of class ToyReflexController

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_ToyReflexController_H_


