/* -------------------------------------------------------------------------- *
 *            OpenSim:  MuscleMetabolicPowerProbeBhargava2004.cpp             *
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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied    *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */


//=============================================================================
// INCLUDES and STATICS
//=============================================================================
#include "MuscleMetabolicPowerProbeBhargava2004.h"


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
MuscleMetabolicPowerProbeBhargava2004::MuscleMetabolicPowerProbeBhargava2004() : Probe()
{
    setNull();
    constructProperties();
}

//_____________________________________________________________________________
/** 
 * Convenience constructor
 */
MuscleMetabolicPowerProbeBhargava2004::MuscleMetabolicPowerProbeBhargava2004(bool activation_rate_on, 
    bool maintenance_rate_on, bool shortening_rate_on, bool basal_rate_on, bool work_rate_on) : Probe()
{
    setNull();
    constructProperties();

    set_activation_rate_on(activation_rate_on);
    set_maintenance_rate_on(maintenance_rate_on);
    set_shortening_rate_on(shortening_rate_on);
    set_basal_rate_on(basal_rate_on);
    set_mechanical_work_rate_on(work_rate_on);
}


//_____________________________________________________________________________
/**
 * Set the data members of this MuscleMetabolicPowerProbeBhargava2004 to their null values.
 */
void MuscleMetabolicPowerProbeBhargava2004::setNull()
{
    // no data members
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void MuscleMetabolicPowerProbeBhargava2004::constructProperties()
{
    constructProperty_activation_rate_on(true);
    constructProperty_maintenance_rate_on(true);
    constructProperty_shortening_rate_on(true);
    constructProperty_basal_rate_on(true);
    constructProperty_mechanical_work_rate_on(true);

    int curvePoints = 5;
    double curveX[] = {0.0, 0.5, 1.0, 1.5, 2.0};
    double curveY[] = {0.5, 0.5, 1.0, 0.0, 0.0};
    PiecewiseLinearFunction fiberLengthDepCurveDefault(curvePoints, curveX, curveY, "defaultCurve");
    constructProperty_normalized_fiber_length_dependence_on_maintenance_rate(fiberLengthDepCurveDefault);

    constructProperty_use_force_dependent_shortening_prop_constant(false);
    constructProperty_basal_coefficient(1.51);
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
 * @param aModel OpenSim model containing this MuscleMetabolicPowerProbeBhargava2004.
 */
void MuscleMetabolicPowerProbeBhargava2004::connectToModel(Model& aModel)
{
    Super::connectToModel(aModel);
}



//=============================================================================
// COMPUTATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute muscle metabolic power.
 * Units = W/kg.
 * Note: for muscle velocities, Vm, we define Vm<0 as shortening and Vm>0 as lengthening.
 */
SimTK::Vector MuscleMetabolicPowerProbeBhargava2004::computeProbeInputs(const State& s) const
{
    // Initialize metabolic energy rate values
    double Adot, Mdot, Sdot, Bdot, Wdot;
    Adot = Mdot = Sdot = Bdot = Wdot = 0;

    // BASAL METABOLIC RATE for whole body, so do outside of muscle loop
    // ------------------------------------------------------------------
    if (get_basal_rate_on())
    {
        Bdot = get_basal_coefficient() * pow(_model->getMatterSubsystem().calcSystemMass(s), get_basal_exponent());
    }
    

    // Loop through each muscle in the MetabolicMuscleParameterSet
    int nM = get_MetabolicMuscleParameterSet().getSize();
    Vector Edot(nM);
    for (int i=0; i<nM; i++)
    {
        // Get a pointer to the current muscle in the model
        MetabolicMuscleParameter mm = get_MetabolicMuscleParameterSet().get(i);
        Muscle* m = checkValidMetabolicMuscle(mm);

        // Get important muscle values at the current time state
        double max_isometric_force = m->getMaxIsometricForce();
        double max_shortening_velocity = m->getMaxContractionVelocity();
        double activation = m->getActivation(s);
        double excitation = m->getControl(s);
        double fiber_force_passive = m->getPassiveFiberForce(s);
        double fiber_force_active = m->getActiveFiberForce(s);
        double fiber_force_total = m->getFiberForce(s);
        double fiber_length_normalized = m->getNormalizedFiberLength(s);
        double fiber_velocity = m->getFiberVelocity(s);
        double fiber_velocity_normalized = m->getNormalizedFiberVelocity(s);
        double slow_twitch_excitation = mm.getRatioSlowTwitchFibers() * sin(Pi/2 * excitation);
        double fast_twitch_excitation = (1 - mm.getRatioSlowTwitchFibers()) * (1 - cos(Pi/2 * excitation));
        double alpha, fiber_length_dependence;

        // Get the unnormalized total active force, F_iso that 'would' be developed at the current activation
        // and fiber length under isometric conditions (i.e. Vm=0)
        double F_iso = (fiber_force_active/m->getForceVelocityMultiplier(s));

        // DEBUG
        //cout << "fiber_velocity_normalized = " << fiber_velocity_normalized << endl;
        //cout << "fiber_velocity_multiplier = " << m->getForceVelocityMultiplier(s) << endl;
        //cout << "fiber_force_passive = " << fiber_force_passive << endl;
        //cout << "fiber_force_active = " << fiber_force_active << endl;
        //cout << "fiber_force_total = " << fiber_force_total << endl;
        //cout << "max_isometric_force = " << max_isometric_force << endl;
        //cout << "F_iso = " << F_iso << endl;
        //system("pause");


        // Warnings
        if (fiber_length_normalized < 0)
            cout << "WARNING: " << getName() << "  (t = " << s.getTime() 
            << "), muscle '" << m->getName() 
            << "' has negative normalized fiber-length." << endl; 



        // ACTIVATION HEAT RATE for muscle i
        // ------------------------------------------
        if (get_activation_rate_on())
        {
            const double decay_function_value = 1.0;    // This value is set to 1.0, as used by Anderson & Pandy (1999), however, in
                                                        // Bhargava et al., (2004) they assume a function here. We will ignore this
                                                        // function and use 1.0 for now.
            Adot = mm.getMuscleMass() * decay_function_value * 
                ( (mm.getActivationConstantSlowTwitch() * slow_twitch_excitation) + (mm.getActivationConstantFastTwitch() * fast_twitch_excitation) );
        }



        // MAINTENANCE HEAT RATE for muscle i
        // ------------------------------------------
        if (get_maintenance_rate_on())
        {
            Vector tmp(1, fiber_length_normalized);
            fiber_length_dependence = get_normalized_fiber_length_dependence_on_maintenance_rate().calcValue(tmp);
            
            Mdot = mm.getMuscleMass() * fiber_length_dependence * 
                ( (mm.getMaintenanceConstantSlowTwitch() * slow_twitch_excitation) + (mm.getMaintenanceConstantFastTwitch() * fast_twitch_excitation) );
        }



        // SHORTENING HEAT RATE for muscle i
        // --> note that we define Vm<0 as shortening and Vm>0 as lengthening
        // -----------------------------------------------------------------------
        if (get_shortening_rate_on())
        {
            if (get_use_force_dependent_shortening_prop_constant())
            {
                if (fiber_velocity <= 0)    // concentric contraction, Vm<0
                    alpha = (0.16 * F_iso) + (0.18 * fiber_force_total);
                else						// eccentric contraction, Vm>0
                    alpha = 0.157 * fiber_force_total;
            }
            else
            {
                if (fiber_velocity <= 0)    // concentric contraction, Vm<0
                    alpha = 0.25 * max_isometric_force;
                else						// eccentric contraction, Vm>0
                    alpha = 0.0;
            }
            Sdot = -alpha * fiber_velocity;
        }
        


        // MECHANICAL WORK RATE for muscle i
        // --> note that we define Vm<0 as shortening and Vm>0 as lengthening
        // ------------------------------------------
        if (get_mechanical_work_rate_on())
        {
            if (fiber_velocity <= 0)    // concentric contraction, Vm<0
                Wdot = -fiber_force_active*fiber_velocity;
            else						// eccentric contraction, Vm>0
                Wdot = 0;
        }



        // TOTAL METABOLIC ENERGY RATE for muscle i
        // ------------------------------------------
        Edot(i) = Adot + Mdot + Sdot + Wdot;



        // DEBUG
        // ----------
        bool debug = false;
        if(debug) {
            cout << "muscle_mass = " << mm.getMuscleMass() << endl;
            cout << "ratio_slow_twitch_fibers = " << mm.getRatioSlowTwitchFibers() << endl;
            cout << "activation_constant_slow_twitch = " << mm.getActivationConstantSlowTwitch() << endl;
            cout << "activation_constant_fast_twitch = " << mm.getActivationConstantFastTwitch() << endl;
            cout << "maintenance_constant_slow_twitch = " << mm.getMaintenanceConstantSlowTwitch() << endl;
            cout << "maintenance_constant_fast_twitch = " << mm.getMaintenanceConstantFastTwitch() << endl;
            cout << "bodymass = " << _model->getMatterSubsystem().calcSystemMass(s) << endl;
            cout << "max_isometric_force = " << max_isometric_force << endl;
            cout << "activation = " << activation << endl;
            cout << "excitation = " << excitation << endl;
            cout << "fiber_force_total = " << fiber_force_total << endl;
            cout << "fiber_force_active = " << fiber_force_active << endl;
            cout << "fiber_length_normalized = " << fiber_length_normalized << endl;
            cout << "fiber_length_dependence = " << fiber_length_dependence << endl;
            cout << "fiber_velocity = " << fiber_velocity << endl;
            cout << "fiber_velocity_normalized = " << fiber_velocity_normalized << endl;
            cout << "slow_twitch_excitation = " << slow_twitch_excitation << endl;
            cout << "fast_twitch_excitation = " << fast_twitch_excitation << endl;
            cout << "max shortening velocity = " << max_shortening_velocity << endl;
            cout << "alpha = " << alpha << endl;
            cout << "Adot = " << Adot << endl;
            cout << "Mdot = " << Mdot << endl;
            cout << "Sdot = " << Sdot << endl;
            cout << "Bdot = " << Bdot << endl;
            cout << "Wdot = " << Wdot << endl;
            cout << "Edot = " << Edot(i) << endl;
            int res = system("pause");
        }
    }

    SimTK::Vector EdotTotal(1, Edot.sum() + Bdot);

    if(EdotTotal(0) < 1.0 
        && get_activation_rate_on() 
        && get_maintenance_rate_on() 
        && get_shortening_rate_on() 
        && get_mechanical_work_rate_on()) {
            cout << "WARNING: " << getName() 
                << "  (t = " << s.getTime() 
                << "), the model has a net metabolic energy rate of less than 1.0 W.kg-1." << endl; 
            EdotTotal(0) = 1.0;			// not allowed to fall below 1.0 W.kg-1
    }

    return EdotTotal;
}


//_____________________________________________________________________________
/** 
 * Returns the number of probe inputs in the vector returned by computeProbeInputs().
 */
int MuscleMetabolicPowerProbeBhargava2004::getNumProbeInputs() const
{
    return 1;
}


//_____________________________________________________________________________
/** 
 * Provide labels for the probe values being reported.
 */
Array<string> MuscleMetabolicPowerProbeBhargava2004::getProbeOutputLabels() const 
{
    Array<string> labels;
    labels.append(getName());
    return labels;
}


//_____________________________________________________________________________
/**
 * Check that the MetabolicMuscleParameter is a valid object.
 * If all tests pass, a pointer to the muscle in the model is returned.
 *
 * @param mm MetabolicMuscleParameter object to check
 * @return *musc Muscle object in model
 */
Muscle* MuscleMetabolicPowerProbeBhargava2004::checkValidMetabolicMuscle(MetabolicMuscleParameter mm) const
{
    stringstream errorMessage;
    Muscle* musc;

    // check that the muscle exists
    int k = _model->getMuscles().getIndex(mm.getName());
    if( k < 0 )	{
        errorMessage << "MetabolicMuscleParameter: Invalid muscle '" << mm.getName() << "' specified." << endl;
        throw (Exception(errorMessage.str()));
    }
    else {
        musc = &_model->updMuscles().get(k);
    }

    // error checking: muscle_mass
    if (mm.getMuscleMass() <= 0) {
        errorMessage << "MetabolicMuscleParameter: Invalid muscle_mass for muscle: " 
            << mm.getName() << ". muscle_mass must be positive." << endl;
        throw (Exception(errorMessage.str()));
    }

    // error checking: ratio_slow_twitch_fibers
    if (mm.getRatioSlowTwitchFibers() < 0 || mm.getRatioSlowTwitchFibers() > 1)	{
        errorMessage << "MetabolicMuscleParameter: Invalid ratio_slow_twitch_fibers for muscle: " 
            << mm.getName() << ". ratio_slow_twitch_fibers must be between 0 and 1." << endl;
        throw (Exception(errorMessage.str()));
    }

    //cout << "VALID muscle: " << mm.getName() << endl;
    return musc;
}
