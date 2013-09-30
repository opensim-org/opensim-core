#ifndef OPENSIM_FIRST_ORDER_MUSCLE_ACTIVATION_DYNAMICS_H_
#define OPENSIM_FIRST_ORDER_MUSCLE_ACTIVATION_DYNAMICS_H_
/* -------------------------------------------------------------------------- *
 *               OpenSim:  FirstOrderMuscleActivationDynamics.h               *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2013 Stanford University and the Authors                *
 * Author(s): Thomas Uchida, Matthew Millard, Ajay Seth                       *
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
/** Computes muscle activation using a modified version of the first-order
    dynamic models used by Thelen (2003) and Winters (1995). The time derivative
    of activation (\f$da/dt\f$) is calculated as follows:
    \f[ \frac{da}{dt} = \frac{u-a}{\tau(u,a)} \f]
    where \f$u\f$ is excitation, \f$a\f$ is activation, and \f$\tau(u,a)\f$ is
    an excitation- and activation-dependent time constant:
    \f[ \tau(u,a) = \tau_{\rm{act}} (0.5 + 1.5a) \quad {\rm{if}}\ u > a \f]
    \f[ \tau(u,a) = \tau_{\rm{deact}} / (0.5 + 1.5a) \quad {\rm{otherwise}} \f]
    Typical values for activation (\f$\tau_{\rm{act}}\f$) and deactivation
    (\f$\tau_{\rm{deact}}\f$) time constants are 10 ms and 40 ms, respectively.

    Three properties are inherited from the base MuscleActivationDynamics class:
    \c minimum_activation, \c maximum_activation, and \c default_activation.
    Note that equilibrium muscle models typically have a numerical singularity
    in their state equations when activation is zero, so care should be taken
    when setting \c minimum_activation to zero. Minimum activation values
    between 0.01 and 0.1 are typically used with equilibrium muscle models.

    <b>Properties</b>
    \li \c activation_time_constant: Activation time constant (in seconds).
    \li \c deactivation_time_constant: Deactivation time constant (in seconds).

    <b>Conditions</b>
    \verbatim
    activation_time_constant > 0
    deactivation_time_constant > 0
    \endverbatim

    <b>Default %Property Values</b>
    \verbatim
    activation_time_constant ...... 0.010
    deactivation_time_constant .... 0.040
    minimum_activation ............ 0.01
    maximum_activation ............ 1
    default_activation ............ 0.5
    \endverbatim

    <b>References</b>
    \li Thelen, D.G. (2003) Adjustment of muscle mechanics model parameters to
        simulate dynamic contractions in older adults. ASME Journal of
        Biomechanical Engineering 125(1):70--77.
    \li Winters, J.M. (1995) An improved muscle-reflex actuator for use in
        large-scale neuromusculoskeletal models. Annals of Biomedical
        Engineering 23(4):359--374.

    @author Thomas Uchida, Matthew Millard, Ajay Seth
**/

class OSIMACTUATORS_API FirstOrderMuscleActivationDynamics :
    public MuscleActivationDynamics {
    OpenSim_DECLARE_CONCRETE_OBJECT(FirstOrderMuscleActivationDynamics,
                                    MuscleActivationDynamics);
public:

//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property Declarations
        These are the serializable properties associated with this class. **/
    //@{
    OpenSim_DECLARE_PROPERTY(activation_time_constant, double,
        "Activation time constant (in seconds)");
    OpenSim_DECLARE_PROPERTY(deactivation_time_constant, double,
        "Deactivation time constant (in seconds)");
    //@}

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** @name Constructors **/
    //@{

    /** Creates a first-order activation dynamic model with the default property
        values and assigns it a default name. An %ExcitationGetter must be
        created for obtaining muscle excitation. **/
    FirstOrderMuscleActivationDynamics();

    /** Creates a first-order activation dynamic model with the default property
        values, the specified name, and the specified %ExcitationGetter. Takes
        ownership of the %ExcitationGetter object. **/
    FirstOrderMuscleActivationDynamics(const std::string& name,
                                       ExcitationGetter* getter);
    //@}

    //--------------------------------------------------------------------------
    // ACCESSORS AND MUTATORS
    //--------------------------------------------------------------------------
    /** @name Accessors and Mutators **/
    //@{

    /** Get the activation time constant. **/
    double getActivationTimeConstant() const;
    /** %Set the activation time constant to a value greater than zero. **/
    void setActivationTimeConstant(double activationTimeConstant);

    /** Get the deactivation time constant. **/
    double getDeactivationTimeConstant() const;
    /** %Set the deactivation time constant to a value greater than zero. **/
    void setDeactivationTimeConstant(double deactivationTimeConstant);

    //@}

    //--------------------------------------------------------------------------
    // MODELCOMPONENT INTERFACE REQUIREMENTS
    //--------------------------------------------------------------------------
    /** @name ModelComponent Interface Requirements **/
    //@{

    /** Adds activation to the state. **/
    void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;

    /** Initializes the activation state variable to \c default_activation. **/
    void initStateFromProperties(SimTK::State& s) const OVERRIDE_11;

    /** Sets \c default_activation to the current value of the activation state
        variable. **/
    void setPropertiesFromState(const SimTK::State& s) OVERRIDE_11;

    /** Calculates the time derivative of activation using a first-order dynamic
        model. Respects the lower bound on activation while preserving the
        expected steady-state value. **/
    SimTK::Vector computeStateVariableDerivatives(const SimTK::State& s)
        const OVERRIDE_11;

    //@}

    //--------------------------------------------------------------------------
    // STATE-DEPENDENT METHODS
    //--------------------------------------------------------------------------
    /** Get the current activation from the state. **/
    double getActivation(const SimTK::State& s) const OVERRIDE_11;

    /** %Set activation state variable to the value provided. **/
    void setActivation(SimTK::State& s, double activation) const OVERRIDE_11;

//==============================================================================
// PRIVATE METHODS
//==============================================================================
private:
    void setNull();
    void constructProperties();

    /** Implementation of a first-order activation dynamic model that respects
        the lower bound on activation while preserving the expected steady-state
        value. **/
    double calcActivationDerivative(double excitation, double activation) const;

    static const std::string STATE_NAME_ACTIVATION;

}; // end of class FirstOrderMuscleActivationDynamics
}  // end of namespace OpenSim

#endif //OPENSIM_FIRST_ORDER_MUSCLE_ACTIVATION_DYNAMICS_H_
