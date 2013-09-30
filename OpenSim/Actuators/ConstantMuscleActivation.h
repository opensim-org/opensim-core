#ifndef OPENSIM_CONSTANT_MUSCLE_ACTIVATION_H_
#define OPENSIM_CONSTANT_MUSCLE_ACTIVATION_H_
/* -------------------------------------------------------------------------- *
 *                    OpenSim:  ConstantMuscleActivation.h                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2013 Stanford University and the Authors                *
 * Author(s): Thomas Uchida, Ajay Seth                                        *
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

#include "Simbody.h"
#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Actuators/MuscleActivationDynamics.h>

namespace OpenSim {
/** This constant muscle activation model simply holds activation at the value
    that was provided most recently, and can be used to generate an activation
    signal without relying on an excitation signal. Three properties are
    inherited from the base MuscleActivationDynamics class: \c
    minimum_activation, \c maximum_activation, and \c default_activation.

    @author Thomas Uchida, Ajay Seth
**/

class OSIMACTUATORS_API ConstantMuscleActivation :
    public MuscleActivationDynamics {
    OpenSim_DECLARE_CONCRETE_OBJECT(ConstantMuscleActivation,
                                    MuscleActivationDynamics);
public:

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** @name Constructors **/
    //@{

    /** Creates a constant activation model with the default property values and
        assigns it a default name. This activation dynamic model does not
        require an %ExcitationGetter. **/
    ConstantMuscleActivation();

    /** Creates a constant activation model with the default property values,
        the specified name, and the specified %ExcitationGetter. Takes ownership
        of the %ExcitationGetter object. **/
    ConstantMuscleActivation(const std::string& name, ExcitationGetter* getter);

    //@}

    //--------------------------------------------------------------------------
    // MODELCOMPONENT INTERFACE REQUIREMENTS
    //--------------------------------------------------------------------------
    /** @name ModelComponent Interface Requirements **/
    //@{

    /** Allocates a cache variable for storing the current activation. **/
    void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;

    /** Initializes the activation cache variable to \c default_activation and
        marks it valid. **/
    void initStateFromProperties(SimTK::State& s) const OVERRIDE_11;

    /** Sets \c default_activation to the current value of the activation cache
        variable. **/
    void setPropertiesFromState(const SimTK::State& s) OVERRIDE_11;

    //@}

    //--------------------------------------------------------------------------
    // STATE-DEPENDENT METHODS
    //--------------------------------------------------------------------------
    /** Get the current activation from the cache. **/
    double getActivation(const SimTK::State& s) const OVERRIDE_11;

    /** %Set activation cache variable to the value provided. **/
    void setActivation(SimTK::State& s, double activation) const OVERRIDE_11;

//==============================================================================
// PRIVATE METHODS
//==============================================================================
private:
    void setNull();

    static const std::string CACHE_NAME_ACTIVATION;

}; // end of class ConstantMuscleActivation
}  // end of namespace OpenSim

#endif //OPENSIM_CONSTANT_MUSCLE_ACTIVATION_H_
