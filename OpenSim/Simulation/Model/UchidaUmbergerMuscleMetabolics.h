/* -------------------------------------------------------------------------- *
 *                 OpenSim:  UchidaUmbergerMuscleMetabolics.h                 *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
 * Author(s): Chris Dembia                                                    *
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

#include "ModelComponent.h"
#include "Muscle.h"
#include <Simbody.h>

namespace OpenSim {

class Umberger2010MuscleMetabolicsProbe;

class OSIMSIMULATION_API UchidaUmbergerMuscleMetabolics : public ModelComponent
{
    OpenSim_DECLARE_CONCRETE_OBJECT(UchidaUmbergerMuscleMetabolics, ModelComponent);
public:

    class MuscleRep;
    
//==============================================================================
// PROPERTIES
//==============================================================================
    
    OpenSim_DECLARE_LIST_PROPERTY(muscle_reps, MuscleRep,
        "TODO");

    /** Enabled by default. **/
    OpenSim_DECLARE_PROPERTY(enforce_minimum_heat_rate_per_muscle, 
        bool,
        "Specify whether the total heat rate for a muscle will be clamped to a "
        "minimum value of 1.0 W/kg (true/false).");

    // TODO
    /** Default value = 1.5. **/
    OpenSim_DECLARE_PROPERTY(aerobic_factor, 
        double,
        "Aerobic scale factor (S=1.0 for primarily anaerobic conditions and S=1.5 "
        "for primarily aerobic conditions. See Umberger et al., (2003).");
    
    // TODO
    /** Default value = 1.2. **/
    OpenSim_DECLARE_PROPERTY(basal_coefficient, double,
        "Basal metabolic coefficient.");

    // TODO
    /** Default value = 1.0. **/
    OpenSim_DECLARE_PROPERTY(basal_exponent, double,
        "Basal metabolic exponent.");

    // TODO
    /** Enabled by default. **/
    OpenSim_DECLARE_PROPERTY(use_Bhargava_recruitment_model, bool,
        "Specify whether the recruitment model described by Bhargava et al. "
        "(2004) will used to determine the slow-twitch fiber ratio "
        "(true/false). Disable to use the model as published in Umberger "
        "(2010).");

    // TODO
    /** Enabled by default. **/
    OpenSim_DECLARE_PROPERTY(include_negative_mechanical_work,
        bool,
        "Specify whether negative mechanical work will be included in Wdot and "
        "a coefficient of 4.0 will be used to calculate alpha_L (true/false). "
        "Disable to use the model as published in Umberger (2010).");

    // TODO
    /** Enabled by default. **/
    OpenSim_DECLARE_PROPERTY(forbid_negative_total_power,
        bool,
        "Specify whether the total power for each muscle must remain positive "
        "(true/false). Disable to use the model as published in Umberger "
        "(2010).");
    
//==============================================================================
// OUTPUTS
//==============================================================================
    
    // TODO is mass available by model stage?
    OpenSim_DECLARE_OUTPUT(basal_rate, double, calcBasalRate,
                           SimTK::Stage::Instance);
    OpenSim_DECLARE_OUTPUT(whole_body_rate, double, calcWholeBodyRate,
                           SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(sum_total_rate, double, calcSumTotalRate,
                           SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(sum_activation_maintenace_rate, double,
                           calcSumActivationMaintenanceRate,
                           SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(sum_shortening_rate, double, calcSumShorteningRate,
                           SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(sum_mechanical_work_rate, double,
                           calcSumMechanicalWorkRate, SimTK::Stage::Dynamics);
    // TODO add total heat rate?
    OpenSim_DECLARE_OUTPUT(instantaneous_efficiency, double,
                           calcInstantaneousEfficiency, SimTK::Stage::Dynamics);
    
//==============================================================================
// METHODS
//==============================================================================
    
    UchidaUmbergerMuscleMetabolics();

    double calcBasalRate(const SimTK::State& s) const;
    double calcWholeBodyRate(const SimTK::State& s) const;
    double calcSumTotalRate(const SimTK::State& s) const;
    double calcSumActivationMaintenanceRate(const SimTK::State& s) const;
    double calcSumShorteningRate(const SimTK::State& s) const;
    double calcSumMechanicalWorkRate(const SimTK::State& s) const;
    double calcInstantaneousEfficiency(const SimTK::State& s) const;
    
private:
    void constructProperties() override;
    
public:
    static UchidaUmbergerMuscleMetabolics createFromProbe(
            const Umberger2010MuscleMetabolicsProbe& probe);

};

/** TODO total rate is NOT AMdot + Sdot + Wdot, since heat rate is capped. */
typedef UchidaUmbergerMuscleMetabolics::MuscleRep UchidaUmbergerMuscleMetabolics_MuscleRep;

class OSIMSIMULATION_API UchidaUmbergerMuscleMetabolics::MuscleRep
        : public ModelComponent {
    // We use the typedef in this macro so that this name is used
    // in XML; "::"'s cause problems in XML tag names.
    OpenSim_DECLARE_CONCRETE_OBJECT(UchidaUmbergerMuscleMetabolics_MuscleRep,
                                    ModelComponent);
public:
//==============================================================================
// PROPERTIES
//==============================================================================

    // TODO only connect inputs from the muscle connector if the user requests it?
    
    // TODO should constant values be inputs as well?
    OpenSim_DECLARE_PROPERTY(specific_tension, double,
        "The specific tension of the muscle (Pascals (N/m^2)).");
    OpenSim_DECLARE_PROPERTY(density, double,
        "The density of the muscle (kg/m^3).");
    OpenSim_DECLARE_PROPERTY(ratio_slow_twitch_fibers, double,
        "Ratio of slow twitch fibers in the muscle (must be between 0 and 1).");
    OpenSim_DECLARE_PROPERTY(use_provided_muscle_mass, bool,
        "An optional flag that allows the user to explicitly specify a muscle mass. "
        "If set to true, the <provided_muscle_mass> property must be specified.");
    OpenSim_DECLARE_PROPERTY(provided_muscle_mass, double,
        "The user specified muscle mass (kg).");
    
//==============================================================================
// INPUTS
//==============================================================================
    // TODO
    OpenSim_DECLARE_INPUT(activation, double, SimTK::Stage::Dynamics, "TODO");
    OpenSim_DECLARE_INPUT(excitation, double, SimTK::Stage::Dynamics, "TODO");
    OpenSim_DECLARE_INPUT(active_fiber_force, double, SimTK::Stage::Dynamics, "TODO");
    OpenSim_DECLARE_INPUT(normalized_fiber_length, double, SimTK::Stage::Dynamics, "TODO");
    OpenSim_DECLARE_INPUT(fiber_velocity, double, SimTK::Stage::Dynamics, "TODO");
    OpenSim_DECLARE_INPUT(active_force_length_multiplier, double, SimTK::Stage::Dynamics, "TODO");
    // TODO use the muscle's fiber work rate output instead of computing it internally?

//==============================================================================
// OUTPUTS
//==============================================================================
    OpenSim_DECLARE_OUTPUT(total_rate, double,
                           calcTotalRate, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(total_heat_rate, double,
                           calcTotalHeatRate, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(activation_maintenance_rate, double,
                           calcActivationMaintenanceRate,
                           SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(shortening_rate, double,
                           calcShorteningRate, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(mechanical_work_rate, double,
                           calcMechanicalWorkRate, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(instantaneous_efficiency, double,
                           calcInstantaneousEfficiency, SimTK::Stage::Dynamics);
    
//==============================================================================
// METHODS
//==============================================================================

    MuscleRep();
    
    // TODO cache this.
    // TODO accounts for "enforce_minimum_heat_rate", while
    // the individual heat rates do not.
    double calcTotalRate(const SimTK::State& s) const;
    double calcTotalHeatRate(const SimTK::State& s) const;
    double calcActivationMaintenanceRate(const SimTK::State& s) const;
    double calcShorteningRate(const SimTK::State& s) const;
    double calcMechanicalWorkRate(const SimTK::State& s) const;
    double calcInstantaneousEfficiency(const SimTK::State& s) const;
    /// Only available after the model is connected.
    double getMuscleMass() const;
    
protected:
    /// Connects this component's inputs to the connected muscle's outputs.
    void extendFinalizeFromProperties() override;
    /// Compute muscle mass from the connected muscle
    /// (if use_provided_muscle_mass is false).
    void extendConnectToModel(OpenSim::Model& model) override;
    void extendAddToSystem(SimTK::MultibodySystem& sys) const override;
    
    const UchidaUmbergerMuscleMetabolics& getParent() const;
    
    double calcUnclampedShorteningRate(const SimTK::State& s) const;
    
    /// Computes the quantity "A" in Umberger, 2003.
    double findActivationDependenceScalingFactor(const double& excitation,
                                                 const double& activation) const;
            
    double findSlowTwitchRatio(const double& excitation,
                               const double& activation) const;
    
private:
    void constructProperties() override;
    void constructConnectors() override {
        constructConnector<Muscle>("muscle");
    }
    
    // Cache the muscle mass based on the connected muscle.
    // Since connections are cleared on copy, we should also clear this cache.
    SimTK::ReinitOnCopy<double> _muscMass { SimTK::NaN };
};

} // namespace OpenSim