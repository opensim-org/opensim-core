#ifndef OPENSIM_FATIGUE_ACTIVATION_DYNAMICS_H_
#define OPENSIM_FATIGUE_ACTIVATION_DYNAMICS_H_
/* -------------------------------------------------------------------------- *
 *               OpenSim:  FatigueMuscleActivationDynamics.h               *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2013 Stanford University and the Authors                *
 * Author(s): Jennifer Yong, Apoorva Rajagopal                       *
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
    
    class OSIMACTUATORS_API FatigueMuscleActivationDynamics :
    public MuscleActivationDynamics {
        OpenSim_DECLARE_CONCRETE_OBJECT(FatigueMuscleActivationDynamics,
                                        MuscleActivationDynamics);
    public:
        
        //==============================================================================
        // PROPERTIES
        //==============================================================================
        OpenSim_DECLARE_PROPERTY(activation_time_constant, double,
                                 "Activation time constant (in seconds)");
        OpenSim_DECLARE_PROPERTY(deactivation_time_constant, double,
                                 "Deactivation time constant (in seconds)");
        
        OpenSim_DECLARE_PROPERTY(fatigue_factor, double, "percentage of active motor units that fatigue in unit time");
        OpenSim_DECLARE_PROPERTY(recovery_factor, double, "percentage of fatigued motor units that recover in unit time");
        OpenSim_DECLARE_PROPERTY(recruitment_from_resting_time_constant, double, "constant that scales the rate of recruitment from resting motor units to active motor units");
        
        OpenSim_DECLARE_PROPERTY(default_active_motor_units, double, "default state value for the fraction of motor units that are active");
        OpenSim_DECLARE_PROPERTY(default_fatigue_motor_units, double, "default state value for the fraction of motor units that are fatigued");
        OpenSim_DECLARE_PROPERTY(default_resting_motor_units, double, "default state value for the fraction of motor units that are resting");
        
        //==============================================================================
        // INPUTS
        //==============================================================================
        OpenSim_DECLARE_INPUT(excitation, double, SimTK::Stage::Time, "excitation signal to the activation dynamics model that will be used to compute the activation and fatigue states");
        
        //==============================================================================
        // OUTPUTS
        //==============================================================================
        OpenSim_DECLARE_OUTPUT(fatigue_activation, double, getActivation, SimTK::Stage::Time);
        OpenSim_DECLARE_OUTPUT(target_activation, double, getTargetActivation, SimTK::Stage::Time);
        OpenSim_DECLARE_OUTPUT(fatigue_motor_units, double, getFatigueMotorUnits, SimTK::Stage::Time);
        OpenSim_DECLARE_OUTPUT(active_motor_units, double, getActiveMotorUnits, SimTK::Stage::Time);
        OpenSim_DECLARE_OUTPUT(resting_motor_units, double, getRestingMotorUnits, SimTK::Stage::Time);
        
        
        
        //==============================================================================
        // PUBLIC METHODS
        //==============================================================================
        /** @name Constructors **/
        //@{
        
        /** Creates a fatigue activation dynamic model with the default property
         values and assigns it a default name. An %ExcitationGetter must be
         created for obtaining muscle excitation. **/
        FatigueMuscleActivationDynamics();
        
        /** Creates a fatigue activation dynamic model with the default property
         values for activation and deactivation time constants and the specified 
         fatigue and recovery factors. An %ExcitationGetter must be
         created for obtaining muscle excitation. **/
        FatigueMuscleActivationDynamics(double fatigueFactor, double recoveryFactor, double recruitmentFromRestingTimeConstant);
        
        /** Creates a fatigue activation dynamic model with the default property
         values, the specified name, and the specified %ExcitationGetter. Takes
         ownership of the %ExcitationGetter object. **/
        FatigueMuscleActivationDynamics(const std::string& name,
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
        
        /** Get the fatigue factor. **/
        double getFatigueFactor() const;
        /** %Set the fatigue factor to a value greater than zero. **/
        void setFatigueFactor(double fatigueFactor);
        
        /** Get the recovery factor. **/
        double getRecoveryFactor() const;
        /** %Set the recovery factor to a value greater than zero. **/
        void setRecoveryFactor(double recoveryFactor);
        
        /** Get the recruitment tiem constant. **/
        double getRecruitmentFromRestingTimeConstant() const;
        /** %Set the recruitment time constant to a value greater than zero. **/
        void setRecruitmentFromRestingTimeConstant(double aRecruitmentFromRestingTimeConstant);
        
        /** Get the default fatigue motor units. **/
        double getDefaultFatigueMotorUnits() const;
        /** %Set the default fatigue motor units to a value greater than zero. **/
        void setDefaultFatigueMotorUnits(double aDefaultFatigueMotorUnits);

        /** Get the default active motor units. **/
        double getDefaultActiveMotorUnits() const;
        /** %Set the default active motor units to a value greater than zero. **/
        void setDefaultActiveMotorUnits(double aDefaultActiveMotorUnits);

        /** Get the default resting motor units. **/
        double getDefaultRestingMotorUnits() const;
        /** %Set the default resting motor units to a value greater than zero. **/
        void setDefaultRestingMotorUnits(double aDefaultRestingMotorUnits);
        
        
        
        //@}
        
        //--------------------------------------------------------------------------
        // MODELCOMPONENT INTERFACE REQUIREMENTS
        //--------------------------------------------------------------------------
        /** @name ModelComponent Interface Requirements **/
        //@{
        
        /** Adds activation and fatigue/active/resting motor units to the state. **/
        void extendAddToSystem(SimTK::MultibodySystem& system) const override;
        
        /** Initializes the activation and fatigue/active/resting motor units state variable to \c default_activation, \c default_fatigued_motor_units, \c default_active_motor_units, \c default_resting_motor_units. **/
        void extendInitStateFromProperties(SimTK::State& s) const override;
        
        /** Sets \c default_activation, \c default_fatigued_motor_units, \c default_active_motor_units, and \c default_resting_motor_units to the current value of the respective state variables. **/
        void extendSetPropertiesFromState(const SimTK::State& s) override;
        
        /** Calculates the time derivative of state variables using a first-order dynamic model. **/
        void computeStateVariableDerivatives(const SimTK::State& s) const override;
        
        //@}
        
        //--------------------------------------------------------------------------
        // STATE-DEPENDENT METHODS
        //--------------------------------------------------------------------------
        /** Get the current activation from the state. **/
        double getActivation(const SimTK::State& s) const override;
        
        /** %Set activation state variable to the value provided. **/
        void setActivation(SimTK::State& s, double activation) const override;
        
        /** Get the current target activation from the state (fatigue state will lower this to the actual activation). **/
        double getTargetActivation(const SimTK::State& s) const;

        /** Set the current target activation from the state (fatigue state will lower this to the actual activation). **/
        void setTargetActivation(SimTK::State& s, double targetActivation) const;
        
        /** Get the current fatigue motor units from the state. **/
        double getFatigueMotorUnits(const SimTK::State& s) const;
        
        /** %Set fatigue motor units variable to the value provided. **/
        void setFatigueMotorUnits(SimTK::State& s, double fatigueMotorUnits) const;
        
        /** Get the current resting motor units from the state. **/
        double getRestingMotorUnits(const SimTK::State& s) const;
        
        /** %Set resting motor units to the value provided. **/
        void setRestingMotorUnits(SimTK::State& s, double restingMotorUnits) const;
        
        /** Get the current active motor units from the state. **/
        double getActiveMotorUnits(const SimTK::State& s) const;
        
        /** %Set active motor units to the value provided. **/
        void setActiveMotorUnits(SimTK::State& s, double activeMotorUnits) const;
        
    
        
        //==============================================================================
        // PRIVATE METHODS
        //==============================================================================
    private:
        void setNull();
        void constructProperties();
        
        /** Implementation of a first-order activation dynamic model that respects
         the lower bound on activation while preserving the expected steady-state
         value. **/
        double calcTargetActivationDerivative(double excitation, double targetActivation) const;
        double calcActivationDerivative(double excitation, double targetActivation, double fatigueMotorUnits, double activemotorUnits) const;
        
        
        double calcFatigueMotorUnitsDeriv(double fatigueMotorUnits, double activeMotorUnits) const;
        double calcActiveMotorUnitsDeriv(double excitation, double targetActivation, double activeMotorUnits, double fatigueMotorUnits) const;
        double calcRestingMotorUnitsDeriv(double fatigueMotorUnitsDeriv, double activeMotorUnitsDeriv) const;
        
        double calcRestingMotorUnits(double fatigueMotorUnits, double activeMotorUnits) const;
        double calcRecruitmentOfResting(double excitation, double targetActivation, double activeMotorUnits, double fatigueMotorUnits) const;
        
        
        static const std::string STATE_NAME_ACTIVATION;
        static const std::string STATE_NAME_TARGETACTIVATION;
        static const std::string STATE_NAME_FATIGUEMOTORUNITS;
        static const std::string STATE_NAME_ACTIVEMOTORUNITS;
        static const std::string STATE_NAME_RESTINGMOTORUNITS;
        
    }; // end of class FatigueMuscleActivationDynamics
}  // end of namespace OpenSim

#endif //OPENSIM_FIRST_ORDER_MUSCLE_ACTIVATION_DYNAMICS_H_
