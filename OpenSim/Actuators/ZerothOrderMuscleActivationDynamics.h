#ifndef OPENSIM_ZEROTH_ORDER_MUSCLE_ACTIVATION_DYNAMICS_H_
#define OPENSIM_ZEROTH_ORDER_MUSCLE_ACTIVATION_DYNAMICS_H_
/* -------------------------------------------------------------------------- *
 *              OpenSim:  ZerothOrderMuscleActivationDynamics.h               *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <OpenSim/Actuators/MuscleActivationDynamics.h>

namespace OpenSim {
/** This zeroth-order muscle activation dynamic model simply sets activation to
    excitation, and can be used when activation dynamics are being ignored.
    Three properties are inherited from the base MuscleActivationDynamics class:
    \c minimum_activation, \c maximum_activation, and \c default_activation.

    @author Thomas Uchida, Ajay Seth
**/

class OSIMACTUATORS_API ZerothOrderMuscleActivationDynamics :
    public MuscleActivationDynamics {
    OpenSim_DECLARE_CONCRETE_OBJECT(ZerothOrderMuscleActivationDynamics,
                                    MuscleActivationDynamics);
public:

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** @name Constructors **/
    //@{

    /** Creates a zeroth-order activation dynamic model with the default
        property values and assigns it a default name. An %ExcitationGetter must
        be created for obtaining muscle excitation. **/
    ZerothOrderMuscleActivationDynamics();

    /** Creates a zeroth-order activation dynamic model with the default
        property values, the specified name, and the specified
        %ExcitationGetter. Takes ownership of the %ExcitationGetter object. **/
    ZerothOrderMuscleActivationDynamics(const std::string& name,
                                        ExcitationGetter* getter);
    //@}

    //--------------------------------------------------------------------------
    // STATE-DEPENDENT METHODS
    //--------------------------------------------------------------------------
    /** Get the current activation, which is simply the excitation. **/
    double getActivation(const SimTK::State& s) const override;

    /** Displays a warning, since there is no activation variable to set. **/
    void setActivation(SimTK::State& s, double activation) const override;

//==============================================================================
// PRIVATE METHODS
//==============================================================================
private:
    void setNull();

}; // end of class ZerothOrderMuscleActivationDynamics
}  // end of namespace OpenSim

#endif //OPENSIM_ZEROTH_ORDER_MUSCLE_ACTIVATION_DYNAMICS_H_
