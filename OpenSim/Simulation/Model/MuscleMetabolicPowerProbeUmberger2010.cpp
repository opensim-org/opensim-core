/* -------------------------------------------------------------------------- *
 *            OpenSim:  MuscleMetabolicPowerProbeUmberger2010.cpp             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Tim Dorn                                                        *
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
// INCLUDES and STATICS
//=============================================================================
#include "MuscleMetabolicPowerProbeUmberger2010.h"


using namespace std;
using namespace SimTK;
using namespace OpenSim;


//=============================================================================
// CONSTRUCTOR(S) AND SETUP
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
MuscleMetabolicPowerProbeUmberger2010::MuscleMetabolicPowerProbeUmberger2010() : Probe()
{
    setNull();
    constructProperties();
}

//_____________________________________________________________________________
/** 
 * Convenience constructor
 */
MuscleMetabolicPowerProbeUmberger2010::MuscleMetabolicPowerProbeUmberger2010(bool activation_maintenance_rate_on, 
    bool shortening_rate_on, bool basal_rate_on, bool work_rate_on) : Probe()
{
    setNull();
    constructProperties();

    set_activation_maintenance_rate_on(activation_maintenance_rate_on);
    set_shortening_rate_on(shortening_rate_on);
    set_basal_rate_on(basal_rate_on);
    set_mechanical_work_rate_on(work_rate_on);
}


//_____________________________________________________________________________
/**
 * Set the data members of this MuscleMetabolicPowerProbeUmberger2010 to their null values.
 */
void MuscleMetabolicPowerProbeUmberger2010::setNull()
{
	setAuthors("Tim Dorn");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void MuscleMetabolicPowerProbeUmberger2010::constructProperties()
{
    constructProperty_activation_maintenance_rate_on(true);
    constructProperty_shortening_rate_on(true);
    constructProperty_basal_rate_on(true);
    constructProperty_mechanical_work_rate_on(true);
    constructProperty_enforce_minimum_heat_rate_per_muscle(true);

    constructProperty_scaling_factor(1.5);      // default value is for aerobic activities.
    constructProperty_basal_coefficient(1.2);   // default value for standing (Umberger, 2003, p105)
    constructProperty_basal_exponent(1.0);
    constructProperty_MetabolicMuscleParameterSet(MetabolicMuscleParameterSet());
}



//=============================================================================
// MODEL COMPONENT METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this MuscleMetabolicPowerProbeUmberger2010.
 */
void MuscleMetabolicPowerProbeUmberger2010::connectToModel(Model& aModel)
{
    // Connect all MetabolicMuscleParameter objects to the model as subcomponents
    for (int i=0; i<get_MetabolicMuscleParameterSet().getSize(); ++i) {
        includeAsSubComponent(&upd_MetabolicMuscleParameterSet().get(i));
    }

    Super::connectToModel(aModel);
}



//=============================================================================
// COMPUTATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute muscle metabolic power.
 * Units = W.
 * Note: for muscle velocities, Vm, we define Vm<0 as shortening and Vm>0 as lengthening.
 */
SimTK::Vector MuscleMetabolicPowerProbeUmberger2010::computeProbeInputs(const State& s) const
{
    // Initialize metabolic energy rate values
    double AMdot, Sdot, Bdot, Wdot;
    AMdot = Sdot = Bdot = Wdot = 0;

    // BASAL METABOLIC RATE (W) (based on whole body mass, not muscle mass)
    // so do outside of muscle loop.
    // ------------------------------------------------------------------
    if (get_basal_rate_on()) {
        Bdot = get_basal_coefficient() 
            * pow(_model->getMatterSubsystem().calcSystemMass(s), get_basal_exponent());
        if (Bdot == NaN)
            cout << "WARNING::" << getName() << ": Bdot = NaN!" << endl;
    }
    

    // Loop through each muscle in the MetabolicMuscleParameterSet
    int nM = get_MetabolicMuscleParameterSet().getSize();
    Vector Edot(nM);
    for (int i=0; i<nM; i++)
    {
        // Get a pointer to the current muscle in the model
        MetabolicMuscleParameter mm = get_MetabolicMuscleParameterSet().get(i);
        Muscle* m = mm.getMuscle();

        // Get some muscle properties at the current time state
        const double max_isometric_force = m->getMaxIsometricForce();
        const double max_shortening_velocity = m->getMaxContractionVelocity();
        const double activation = m->getActivation(s);
        const double excitation = m->getControl(s);
        const double fiber_force_passive = m->getPassiveFiberForce(s);
        const double fiber_force_active = m->getActiveFiberForce(s);
        const double fiber_force_total = m->getFiberForce(s);
        const double fiber_length_normalized = m->getNormalizedFiberLength(s);
        const double fiber_velocity = m->getFiberVelocity(s);
        double A;

        // Umberger defines fiber_velocity_normalized as Vm/LoM, not Vm/Vmax (p101, top left, Umberger(2003))
        //const double fiber_velocity_normalized = m->getNormalizedFiberVelocity(s);
        const double fiber_velocity_normalized = fiber_velocity / m->getOptimalFiberLength();


        // ---------------------------------------------------------------------------
        // NOT USED FOR THIS IMPLEMENTATION
        //const double slow_twitch_excitation = mm.getRatioSlowTwitchFibers() * sin(Pi/2 * excitation);
        //const double fast_twitch_excitation = (1 - mm.getRatioSlowTwitchFibers()) * (1 - cos(Pi/2 * excitation));

        // Set normalized hill constants: A_rel and B_rel
        //const double A_rel = 0.1 + 0.4*(1 - mm.getRatioSlowTwitchFibers());
        //const double B_rel = A_rel * max_shortening_velocity;
        // ---------------------------------------------------------------------------


        // Set activation dependence scaling parameter: A
        if (excitation > activation)
            A = excitation;
        else
            A = (excitation + activation) / 2;

        // Get the normalized active fiber force, F_iso, that 'would' be developed at the current activation
        // and fiber length under isometric conditions (i.e. Vm=0)
        //double F_iso = (fiber_force_active/m->getForceVelocityMultiplier(s)) / max_isometric_force;
        double F_iso = m->getActivation(s) * m->getActiveForceLengthMultiplier(s);


        // DEBUG
        //cout << "fiber_velocity_normalized = " << fiber_velocity_normalized << endl;
        //cout << "fiber_velocity_multiplier = " << m->getForceVelocityMultiplier(s) << endl;
        //cout << "fiber_force_active = " << fiber_force_active << endl;
        //cout << "fiber_force_total = " << fiber_force_total << endl;
        //cout << "max_isometric_force = " << max_isometric_force << endl;
        //cout << "F_iso = " << F_iso << endl;
        //system("pause");


        

        // Warnings
        if (fiber_length_normalized < 0)
            cout << "WARNING: (t = " << s.getTime() 
            << "), muscle '" << m->getName() 
            << "' has negative normalized fiber-length." << endl; 



        // ACTIVATION & MAINTENANCE HEAT RATE for muscle i (W/kg)
        // --> depends on the normalized fiber length of the contractile element
        // -----------------------------------------------------------------------
        if (get_activation_maintenance_rate_on())
        {
            const double unscaledAMdot = 128*(1 - mm.getRatioSlowTwitchFibers()) + 25;

            if (fiber_length_normalized <= 1.0)
                AMdot = get_scaling_factor() * std::pow(A, 0.6) * unscaledAMdot;
            else
                AMdot = get_scaling_factor() * std::pow(A, 0.6) * ((0.4 * unscaledAMdot) + (0.6 * unscaledAMdot * F_iso));
        }



        // SHORTENING HEAT RATE for muscle i (W/kg)
        // --> depends on the normalized fiber length of the contractile element
        // --> note that we define Vm<0 as shortening and Vm>0 as lengthening
        // -----------------------------------------------------------------------
        if (get_shortening_rate_on())
        {
            const double Vmax_fasttwitch = max_shortening_velocity;
            const double Vmax_slowtwitch = max_shortening_velocity / 2.5;
            const double alpha_shortening_fasttwitch = 153 / Vmax_fasttwitch;
            const double alpha_shortening_slowtwitch = 100 / Vmax_slowtwitch;
            double unscaledSdot, tmp_slowTwitch, tmp_fastTwitch;

            if (fiber_velocity_normalized <= 0)    // concentric contraction, Vm<0
            {
                const double maxShorteningRate = 100.0;    // (W/kg)

                tmp_slowTwitch = -alpha_shortening_slowtwitch * fiber_velocity_normalized;

                // Apply upper limit to the unscaled slow twitch shortening rate.
                if (tmp_slowTwitch > maxShorteningRate) {
                    //cout << "WARNING: " << getName() << "  (t = " << s.getTime() << 
                    //    "Slow twitch shortening heat rate exceeds the max value of " << maxShorteningRate << 
                    //    " W/kg. Setting to " << maxShorteningRate << " W/kg." << endl; 
                    tmp_slowTwitch = maxShorteningRate;
                }

                tmp_fastTwitch = alpha_shortening_fasttwitch * fiber_velocity_normalized * (1-mm.getRatioSlowTwitchFibers());
                unscaledSdot = (tmp_slowTwitch * mm.getRatioSlowTwitchFibers()) - tmp_fastTwitch;   // unscaled shortening heat rate: muscle shortening
                Sdot = get_scaling_factor() * std::pow(A, 2.0) * unscaledSdot;                      // scaled shortening heat rate: muscle shortening
            }

            else	// eccentric contraction, Vm>0
            {
                unscaledSdot = 0.3 * alpha_shortening_slowtwitch * fiber_velocity_normalized;  // unscaled shortening heat rate: muscle lengthening
                Sdot = get_scaling_factor() * A * unscaledSdot;                                // scaled shortening heat rate: muscle lengthening
            }


            // Fiber length dependance on scaled shortening heat rate
            // (for both concentric and eccentric contractions).
            if (fiber_length_normalized > 1.0)
                Sdot *= F_iso;  
        }
        


        // MECHANICAL WORK RATE for the contractile element of muscle i (W/kg).
        // --> note that we define Vm<0 as shortening and Vm>0 as lengthening.
        // -------------------------------------------------------------------
        if (get_mechanical_work_rate_on())
        {
            if (fiber_velocity <= 0)    // concentric contraction, Vm<0
                Wdot = -fiber_force_active*fiber_velocity;
            else						// eccentric contraction, Vm>0
                Wdot = 0;

            Wdot /= mm.getMuscleMass();
        }


        // NAN CHECKING
        // ------------------------------------------
        if (AMdot == NaN)
            cout << "WARNING::" << getName() << ": AMdot (" << m->getName() << ") = NaN!" << endl;
        if (Sdot == NaN)
            cout << "WARNING::" << getName() << ": Sdot (" << m->getName() << ") = NaN!" << endl;
        if (Wdot == NaN)
            cout << "WARNING::" << getName() << ": Wdot (" << m->getName() << ") = NaN!" << endl;


        // This check is from Umberger(2003), page 104: the total heat rate 
        // (i.e., AMdot + Sdot) for a given muscle cannot fall below 1.0 W/kg.
        // -----------------------------------------------------------------------
        double totalHeatRate = AMdot + Sdot;

        if(get_enforce_minimum_heat_rate_per_muscle() && totalHeatRate < 1.0 
            && get_activation_maintenance_rate_on() 
            && get_shortening_rate_on()) {
                //cout << "WARNING: " << getName() 
                //    << "  (t = " << s.getTime() 
                //    << "), the muscle '" << mm.getName() 
                //    << "' has a net metabolic energy rate of less than 1.0 W/kg." << endl; 
                totalHeatRate = 1.0;			// not allowed to fall below 1.0 W.kg-1
        }


        // TOTAL METABOLIC ENERGY RATE for muscle i
        // UNITS: W
        // ------------------------------------------
        Edot(i) = (totalHeatRate + Wdot) * mm.getMuscleMass();


        
        

        // DEBUG
        // ----------
        const bool debug = false;
        if(debug) {
            cout << "muscle_mass = " << mm.getMuscleMass() << endl;
            cout << "ratio_slow_twitch_fibers = " << mm.getRatioSlowTwitchFibers() << endl;
            cout << "bodymass = " << _model->getMatterSubsystem().calcSystemMass(s) << endl;
            cout << "max_isometric_force = " << max_isometric_force << endl;
            cout << "activation = " << activation << endl;
            cout << "excitation = " << excitation << endl;
            cout << "fiber_force_total = " << fiber_force_total << endl;
            cout << "fiber_force_active = " << fiber_force_active << endl;
            cout << "fiber_length_normalized = " << fiber_length_normalized << endl;
            cout << "fiber_velocity = " << fiber_velocity << endl;
            //cout << "slow_twitch_excitation = " << slow_twitch_excitation << endl;
            //cout << "fast_twitch_excitation = " << fast_twitch_excitation << endl;
            cout << "max shortening velocity = " << max_shortening_velocity << endl;
            //cout << "A_rel = " << A_rel << endl;
            //cout << "B_rel = " << B_rel << endl;
            cout << "AMdot = " << AMdot << endl;
            cout << "Sdot = " << Sdot << endl;
            cout << "Bdot = " << Bdot << endl;
            cout << "Wdot = " << Wdot << endl;
            cout << "Edot = " << Edot(i) << endl;
			std::cin.get();
        }
    }

    SimTK::Vector EdotTotal(1, Edot.sum() + Bdot);
    return EdotTotal;
}


//_____________________________________________________________________________
/** 
 * Returns the number of probe inputs in the vector returned by computeProbeInputs().
 */
int MuscleMetabolicPowerProbeUmberger2010::getNumProbeInputs() const
{
    return 1;
}


//_____________________________________________________________________________
/** 
 * Provide labels for the probe values being reported.
 */
Array<string> MuscleMetabolicPowerProbeUmberger2010::getProbeOutputLabels() const 
{
    Array<string> labels;
    labels.append(getName());
    return labels;
}
