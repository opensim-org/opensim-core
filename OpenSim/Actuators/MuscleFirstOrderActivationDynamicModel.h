#ifndef OPENSIM_MUSCLEFIRSTORDERACTIVATIONDYNAMICMODEL_H_
#define OPENSIM_MUSCLEFIRSTORDERACTIVATIONDYNAMICMODEL_H_
/* -------------------------------------------------------------------------- *
 *             OpenSim:  MuscleFirstOrderActivationDynamicModel.h             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Matthew Millard                                                 *
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
#include <OpenSim/Simulation/Model/ModelComponent.h>

namespace OpenSim {
/** This is a muscle modeling utility class that computes the time derivative of
    activation using a first-order dynamic model. This activation model is a
    modification of those used by Thelen (2003) and Winters (1995). The time
    derivative of activation (\f$da/dt\f$) is calculated as follows:
    \f[ \frac{da}{dt} = \frac{u-a}{\tau(u,a)} \f]
    where \f$u\f$ is excitation, \f$a\f$ is activation, and \f$\tau(u,a)\f$ is a
    variable time constant:
    \f[ \tau(u,a) = t_{\rm{act}} (0.5 + 1.5a) \quad {\rm{if}}\ u > a \f]
    \f[ \tau(u,a) = t_{\rm{deact}} / (0.5 + 1.5a) \quad {\rm{otherwise}} \f]

    Since equilibrium muscle models typically have a numerical singularity in
    their state equations when activation is zero, we apply a lower activation
    bound (\f$a_{\rm{min}}\f$) to both activation and excitation.

    @param tauActivation
        Activation time constant. A typical value is 0.010 s (10 ms).
    @param tauDeactivation
        Deactivation time constant. A typical value is 0.040 s (40 ms).
    @param minActivation
        The minimum permissible activation. To avoid a numerical singularity at
        a = 0, this value is typically set to between 0.01 and 0.1 for use with
        an equilibrium muscle model.
    @param muscleName
        The name of the muscle to which this activation dynamic model belongs.
        This string is used for reporting meaningful error messages.

    <B>Conditions</B>
    \verbatim
    tauActivation > 0
    tauDeactivation > 0
    0 <= minActivation < 1
    \endverbatim

    <B>Default Parameter Values</B>
    \verbatim
    tauActivation ...... 0.010
    tauDeactivation .... 0.040
    minActivation ...... 0.01
    \endverbatim

    <B>References</B>
    \li Thelen, D.G. (2003) Adjustment of muscle mechanics model parameters to
        simulate dynamic contractions in older adults. ASME Journal of
        Biomechanical Engineering 125(1):70--77.
    \li Winters, J.M. (1995) An improved muscle-reflex actuator for use in
        large-scale neuromusculoskeletal models. Annals of Biomedical
        Engineering 23(4):359--374.

    @author Matt Millard
*/
class OSIMACTUATORS_API MuscleFirstOrderActivationDynamicModel : public ModelComponent{
OpenSim_DECLARE_CONCRETE_OBJECT(MuscleFirstOrderActivationDynamicModel, ModelComponent);
public:

//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(activation_time_constant, double,
        "Activation time constant, in seconds (overridden when this is a subcomponent of a Muscle)");
    OpenSim_DECLARE_PROPERTY(deactivation_time_constant, double,
        "Deactivation time constant, in seconds (overridden when this is a subcomponent of a Muscle)");
    OpenSim_DECLARE_PROPERTY(minimum_activation, double,
        "Lower bound on activation (overridden when this is a subcomponent of a Muscle)");

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** The default constructor creates an activation dynamic model with the
    default property values and assigns it a default name. **/
    MuscleFirstOrderActivationDynamicModel();

    /** Creates an activation dynamic model using the provided parameters. */
    MuscleFirstOrderActivationDynamicModel(double tauActivation,
                                           double tauDeactivation,
                                           double minActivation,
                                           const std::string& muscleName);

    /**
    @returns Activation clamped to the range [minActivation, 1.0].
    */
    double clampActivation(double activation) const;

    /** Calculates the time derivative of activation. */
    double calcDerivative(double activation, double excitation) const;

protected:
    // Component interface.
    void extendFinalizeFromProperties() override;

private:
    void setNull();
    void constructProperties();

};

}
#endif //OPENSIM_MUSCLEFIRSTORDERACTIVATIONDYNAMICMODEL_H_
