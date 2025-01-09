/* -------------------------------------------------------------------------- *
 *              OpenSim:  Umberger2010MuscleMetabolicsProbe.cpp               *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Tim Dorn                                                        *
 * Contributor(s): Thomas Uchida                                              *
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
#include "Umberger2010MuscleMetabolicsProbe.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Muscle.h>
//#define DEBUG_METABOLICS

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
Umberger2010MuscleMetabolicsProbe::Umberger2010MuscleMetabolicsProbe() : Probe()
{
    setNull();
    constructProperties();
}

//_____________________________________________________________________________
/** 
 * Convenience constructor
 */
Umberger2010MuscleMetabolicsProbe::Umberger2010MuscleMetabolicsProbe(
    const bool activation_maintenance_rate_on, 
    const bool shortening_rate_on, 
    const bool basal_rate_on, 
    const bool work_rate_on) : Probe()
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
 * Set the data members of this Umberger2010MuscleMetabolicsProbe 
 * to their null values.
 */
void Umberger2010MuscleMetabolicsProbe::setNull()
{
    setAuthors("Tim Dorn");
    setReferences("Umberger, B. R. (2010). Stance and swing phase costs in "
    "human walking. J R Soc Interface 7, 1329-40.");
    _muscleMap.clear();
}

//_____________________________________________________________________________
/**
 * Construct and initialize object properties.
 */
void Umberger2010MuscleMetabolicsProbe::constructProperties()
{
    constructProperty_activation_maintenance_rate_on(true);
    constructProperty_shortening_rate_on(true);
    constructProperty_basal_rate_on(true);
    constructProperty_mechanical_work_rate_on(true);
    constructProperty_enforce_minimum_heat_rate_per_muscle(true);

    constructProperty_aerobic_factor(1.5);      // default value is for aerobic activities.
    constructProperty_basal_coefficient(1.2);   // default value for standing (Umberger, 2003, p105)
    constructProperty_basal_exponent(1.0);
    constructProperty_muscle_effort_scaling_factor(1.0);
    constructProperty_use_Bhargava_recruitment_model(true);
    constructProperty_include_negative_mechanical_work(true);
    constructProperty_forbid_negative_total_power(true);
    constructProperty_report_total_metabolics_only(true);
    constructProperty_Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameterSet
       (Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameterSet());
}




//=============================================================================
// MODEL COMPONENT METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this Umberger2010MuscleMetabolicsProbe.
 */
void Umberger2010MuscleMetabolicsProbe::extendConnectToModel(Model& aModel)
{
    Super::extendConnectToModel(aModel);
    if (!isEnabled()) return;   // Nothing to connect

    const int nM = 
        get_Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameterSet().getSize();
    for (int i=0; i<nM; ++i) {
        connectIndividualMetabolicMuscle(aModel, 
            upd_Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameterSet()[i]);
    }
}

//_____________________________________________________________________________
/**
 * Connect an individual metabolic muscle to the model.
 * Check that the muscles in the MetabolicMuscleParameterSet exist in
 * the model and create the MuscleMap between the muscle name and the
 * muscle pointer.
 *
 */
void Umberger2010MuscleMetabolicsProbe::connectIndividualMetabolicMuscle(
    Model& aModel, 
    Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter& mm)
{
    stringstream errorMessage;

    int k = aModel.getMuscles().getIndex(mm.getName());
    if( k < 0 ) {
        log_warn("Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter: "
                "Muscle '{}' not found in model. Ignoring...",
                mm.getName());
        setEnabled(false);
        return;
    }
    else {
        mm.setMuscle(&aModel.updMuscles()[k]);  // Set internal muscle pointer
        _muscleMap[mm.getName()] = &mm;         // Add parameters to the _muscleMap
    }


    // -----------------------------------------------------------------------
    // Set the muscle mass internal member variable: _muscMass based on
    // whether the <use_provided_muscle_mass> property is true or false.
    // -----------------------------------------------------------------------
    if (mm.get_use_provided_muscle_mass()) {
            
        // Check that the <provided_muscle_mass> has been correctly specified.
        if (mm.get_provided_muscle_mass() <= 0) {
            errorMessage << "ERROR: Negative <provided_muscle_mass> specified for " 
                << mm.getName() 
                << ". <provided_muscle_mass> must be a positive number (kg)." << endl;
            log_warn("{} Probe will be disabled.", errorMessage.str());
            setEnabled(false);
        }
        else if (isNaN(mm.get_provided_muscle_mass())) {
            errorMessage << "ERROR: No <provided_muscle_mass> specified for " 
                << mm.getName() 
                << ". <provided_muscle_mass> must be a positive number (kg)." << endl;
            log_warn("{} Probe will be disabled.", errorMessage.str());
            setEnabled(false);
        }
    }

    else {

        // Check that <specific_tension> and <density> have been correctly specified.
        if (mm.get_specific_tension() <= 0) {
            errorMessage << "ERROR: Negative <specific_tension> specified for " 
                << mm.getName() 
                << ". <specific_tension> must be a positive number (N/m^2)." << endl;
            log_warn("{} Probe will be disabled.", errorMessage.str());
            setEnabled(false);
        }
        if (mm.get_density() <= 0) {
            errorMessage << "ERROR: Negative <density> specified for " 
                << mm.getName() 
                << ". <density> must be a positive number (kg/m^3)." << endl;
            log_warn("{} Probe will be disabled.", errorMessage.str());
            setEnabled(false);
        }
    }


    // -----------------------------------------------------------------------
    // Check that <ratio_slow_twitch_fibers> is between 0 and 1.
    // -----------------------------------------------------------------------
    if (mm.get_ratio_slow_twitch_fibers() < 0 || mm.get_ratio_slow_twitch_fibers() > 1) {
        errorMessage << "MetabolicMuscleParameter: Invalid ratio_slow_twitch_fibers for muscle: " 
            << getName() << ". ratio_slow_twitch_fibers must be between 0 and 1." << endl;
        log_warn("{} Probe will be disabled.", errorMessage.str());
        setEnabled(false);
    }


    // -----------------------------------------------------------------------
    // Set the mass used for this muscle.
    // -----------------------------------------------------------------------
    mm.setMuscleMass();
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
SimTK::Vector Umberger2010MuscleMetabolicsProbe::computeProbeInputs(const State& s) const
{
    // Initialize metabolic energy rate values.
    double AMdot, Sdot, Bdot, Wdot;
    AMdot = Sdot = Bdot = Wdot = 0;
    Vector EdotOutput(getNumProbeInputs());
    EdotOutput = 0;


    // BASAL METABOLIC RATE (W) (based on whole body mass, not muscle mass)
    // so do outside of muscle loop.
    // TODO: system mass should be precalculated.
    // ------------------------------------------------------------------
    if (get_basal_rate_on()) {
        Bdot = get_basal_coefficient() 
            * pow(_model->getMatterSubsystem().calcSystemMass(s), get_basal_exponent());
        if (isNaN(Bdot)) 
            log_warn("{} : Bdot = NaN!", getName());
    }
    EdotOutput(0) += Bdot;       // TOTAL metabolic power storage
    
    if (!get_report_total_metabolics_only())
        EdotOutput(1) = Bdot;    // BASAL metabolic power storage
    

    // Loop through each muscle in the MetabolicMuscleParameterSet
    const int nM = 
        get_Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameterSet()
        .getSize();
    for (int i=0; i<nM; ++i)
    {
        // Get the current muscle parameters from the MetabolicMuscleParameterSet
        // and the corresponding OpenSim::Muscle pointer from the muscleMap.
        Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter& mm = 
            get_Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameterSet()[i];
        const Muscle* m = mm.getMuscle();

        // Get some muscle properties at the current time state
        //const double max_isometric_force = m->getMaxIsometricForce();
        const double max_shortening_velocity = m->getMaxContractionVelocity();
        const double activation = get_muscle_effort_scaling_factor()
                                  * m->getActivation(s);
        const double excitation = get_muscle_effort_scaling_factor()
                                  * m->getControl(s);
        double fiber_force_active = get_muscle_effort_scaling_factor()
                                    * m->getActiveFiberForce(s);
        const double fiber_length_normalized = m->getNormalizedFiberLength(s);
        const double fiber_velocity = m->getFiberVelocity(s);
        double A;

        // Umberger defines fiber_velocity_normalized as Vm/LoM, not Vm/Vmax (p101, top left, Umberger(2003))
        //const double fiber_velocity_normalized = m->getNormalizedFiberVelocity(s);
        const double fiber_velocity_normalized = fiber_velocity / m->getOptimalFiberLength();


        // ---------------------------------------------------------------------------
        // NOT USED FOR THIS IMPLEMENTATION
        //const double slow_twitch_excitation = mm.get_ratio_slow_twitch_fibers() * sin(Pi/2 * excitation);
        //const double fast_twitch_excitation = (1 - mm.get_ratio_slow_twitch_fibers()) * (1 - cos(Pi/2 * excitation));

        // Set normalized hill constants: A_rel and B_rel
        //const double A_rel = 0.1 + 0.4*(1 - mm.get_ratio_slow_twitch_fibers());
        //const double B_rel = A_rel * max_shortening_velocity;
        // ---------------------------------------------------------------------------


        // Set activation dependence scaling parameter: A
        if (excitation > activation)
            A = excitation;
        else
            A = (excitation + activation) / 2;

        // Normalized contractile element force-length curve
        const double F_iso = m->getActiveForceLengthMultiplier(s);   

        // Warnings
        if (fiber_length_normalized < 0)
            log_warn("t = {}), muscle '{}' has negative normalized fiber-length.",
                    s.getTime(), m->getName()); 



        // ACTIVATION & MAINTENANCE HEAT RATE for muscle i (W/kg)
        // --> depends on the normalized fiber length of the contractile element
        // -----------------------------------------------------------------------
        double slowTwitchRatio = mm.get_ratio_slow_twitch_fibers();
        if (get_use_Bhargava_recruitment_model()) {
            const double uSlow = slowTwitchRatio * sin(0.5*Pi * excitation);
            const double uFast = (1 - slowTwitchRatio)
                                 * (1 - cos(0.5*Pi * excitation));
            slowTwitchRatio = (excitation == 0) ? 1.0 : uSlow / (uSlow + uFast);
        }

        if (get_forbid_negative_total_power() ||
            get_activation_maintenance_rate_on())
        {
            const double unscaledAMdot = 128*(1 - slowTwitchRatio) + 25;

            if (fiber_length_normalized <= 1.0)
                AMdot = get_aerobic_factor() * std::pow(A, 0.6) * unscaledAMdot;
            else
                AMdot = get_aerobic_factor() * std::pow(A, 0.6) * ((0.4 * unscaledAMdot) + (0.6 * unscaledAMdot * F_iso));
        }



        // SHORTENING HEAT RATE for muscle i (W/kg)
        // --> depends on the normalized fiber length of the contractile element
        // --> note that we define Vm<0 as shortening and Vm>0 as lengthening
        // -----------------------------------------------------------------------
        if (get_forbid_negative_total_power() || get_shortening_rate_on())
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

                tmp_fastTwitch = alpha_shortening_fasttwitch * fiber_velocity_normalized * (1-slowTwitchRatio);
                unscaledSdot = (tmp_slowTwitch * slowTwitchRatio) - tmp_fastTwitch;   // unscaled shortening heat rate: muscle shortening
                Sdot = get_aerobic_factor() * std::pow(A, 2.0) * unscaledSdot;                      // scaled shortening heat rate: muscle shortening
            }

            else    // eccentric contraction, Vm>0
            {
                unscaledSdot =
                    (get_include_negative_mechanical_work() ? 4.0 : 0.3)
                    * alpha_shortening_slowtwitch * fiber_velocity_normalized;  // unscaled shortening heat rate: muscle lengthening
                Sdot = get_aerobic_factor() * A * unscaledSdot;                                // scaled shortening heat rate: muscle lengthening
            }


            // Fiber length dependence on scaled shortening heat rate
            // (for both concentric and eccentric contractions).
            if (fiber_length_normalized > 1.0)
                Sdot *= F_iso;  
        }
        


        // Clamp fiber force. THIS SHOULD NEVER HAPPEN...
        if (fiber_force_active < 0)
            fiber_force_active = 0.0;




        // MECHANICAL WORK RATE for the contractile element of muscle i (W/kg).
        // --> note that we define Vm<0 as shortening and Vm>0 as lengthening.
        // -------------------------------------------------------------------
        if (get_forbid_negative_total_power() || get_mechanical_work_rate_on())
        {
            if (get_include_negative_mechanical_work() || fiber_velocity <= 0)
                Wdot = -fiber_force_active*fiber_velocity;
            else
                Wdot = 0;

            Wdot /= mm.getMuscleMass();
        }


        // If necessary, increase the shortening heat rate so that the total
        // power is non-negative.
        if (get_forbid_negative_total_power()) {
            const double Edot_Wkg_beforeClamp = AMdot + Sdot + Wdot;
            if (Edot_Wkg_beforeClamp < 0)
                Sdot -= Edot_Wkg_beforeClamp;
        }


        // NAN CHECKING
        // ------------------------------------------
        if (isNaN(AMdot))
            log_warn("{}  : AMdot ({}) = NaN!", getName(), m->getName());
        if (isNaN(Sdot))
            log_warn("{}  : Sdot ({}) = NaN!", getName(), m->getName());
        if (isNaN(Wdot))
            log_warn("{}  : Wdot ({}) = NaN!", getName(), m->getName());

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
                totalHeatRate = 1.0;            // not allowed to fall below 1.0 W.kg-1
        }
        

        // TOTAL METABOLIC ENERGY RATE for muscle i
        // UNITS: W
        // ------------------------------------------
        double Edot = 0;

        if (get_activation_maintenance_rate_on() && get_shortening_rate_on())
            Edot += totalHeatRate;      // May have been clamped to 1.0 W/kg.
        else {
            if (get_activation_maintenance_rate_on())
                Edot += AMdot;
            if (get_shortening_rate_on())
                Edot += Sdot;
        }
        if (get_mechanical_work_rate_on())
            Edot += Wdot;
        Edot *= mm.getMuscleMass();

        EdotOutput(0) += Edot;       // Add to TOTAL metabolic power storage
        if (!get_report_total_metabolics_only()) {
            // Metabolic power storage for muscle i
            EdotOutput(i+2) = Edot;  
        }                          


        

#ifdef DEBUG_METABOLICS
        cout << "muscle_mass = " << mm.getMuscleMass() << endl;
        cout << "ratio_slow_twitch_fibers = " << slowTwitchRatio << endl;
        cout << "bodymass = " << _model->getMatterSubsystem().calcSystemMass(s) << endl;
        //cout << "max_isometric_force = " << max_isometric_force << endl;
        cout << "activation = " << activation << endl;
        cout << "excitation = " << excitation << endl;
        //cout << "fiber_force_total = " << fiber_force_total << endl;
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
        cout << "Edot = " << Edot << endl;
        std::cin.get();
#endif
    }

    return EdotOutput;
}


//_____________________________________________________________________________
/** 
 * Returns the number of probe inputs in the vector returned by computeProbeInputs().
 * If report_total_metabolics_only = true, then only the TOTAL metabolics will be
 * calculated. If report_total_metabolics_only = false, then the calculation will
 * consist of a TOTAL value, a BASAL value, and each individual muscle
 * contribution.
 */
int Umberger2010MuscleMetabolicsProbe::getNumProbeInputs() const
{
    if (get_report_total_metabolics_only())
        return 1;
    else
        return 2 + getNumMetabolicMuscles();
}


//_____________________________________________________________________________
/** 
 * Provide labels for the probe values being reported.
 * If report_total_metabolics_only = true, then only the TOTAL metabolics will be
 * calculated. If report_total_metabolics_only = false, then the calculation will
 * consist of a TOTAL value, a BASAL value, and each individual muscle
 * contribution.
 */
Array<string> Umberger2010MuscleMetabolicsProbe::getProbeOutputLabels() const 
{
    Array<string> labels;
    labels.append(getName()+"_TOTAL");

    if (get_report_total_metabolics_only())
        return labels;

    labels.append(getName()+"_BASAL");

    for (int i=0; i<getNumMetabolicMuscles(); ++i)
        labels.append(getName()+"_"+get_Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameterSet()[i].getName());

    return labels;
}





//=============================================================================
// MUSCLE METABOLICS INTERFACE
//=============================================================================
//_____________________________________________________________________________
/** 
* Get the number of muscles being analyzed in the metabolic analysis. 
*/
int Umberger2010MuscleMetabolicsProbe::
    getNumMetabolicMuscles() const  
{ 
    return get_Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameterSet().getSize(); 
}


//_____________________________________________________________________________
/**
 * Add a muscle and its parameters so that it can be included in the metabolic analysis
 */
void Umberger2010MuscleMetabolicsProbe::
    addMuscle(const string& muscleName, 
    double ratio_slow_twitch_fibers)
{
    Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter* mm = 
        new Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter(
            muscleName,
            ratio_slow_twitch_fibers);

    upd_Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameterSet()
        .adoptAndAppend(mm);    // add to MetabolicMuscleParameterSet in the model
}

//_____________________________________________________________________________
/**
 * Add a muscle and its parameters so that it can be included in the metabolic analysis
 */
void Umberger2010MuscleMetabolicsProbe::
    addMuscle(const string& muscleName, 
    double ratio_slow_twitch_fibers,
    double muscle_mass)
{
    Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter* mm = 
        new Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter(
            muscleName,
            ratio_slow_twitch_fibers, 
            muscle_mass);

    upd_Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameterSet()
        .adoptAndAppend(mm);    // add to MetabolicMuscleParameterSet in the model
}


//_____________________________________________________________________________
/**
 * Remove a muscle from the MetabolicMuscleParameterSet.
 */
void Umberger2010MuscleMetabolicsProbe::
    removeMuscle(const string& muscleName)
{
    // Step 1: Remove the reference to this MetabolicMuscleParameter
    // from the muscle map.
    // -----------------------------------------------------------------
    _muscleMap.erase(muscleName);


    // Step 2: Remove the MetabolicMuscleParameter object from
    // the MetabolicMuscleParameterSet.
    // -----------------------------------------------------------------
    const int k = get_Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameterSet().getIndex(muscleName);
    if (k<0) {
        log_warn("MetabolicMuscleParameter: Invalid muscle '{}' specified. No "
                 "metabolic muscles removed.",
                muscleName);
        return;
    }
    clearConnections();
    upd_Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameterSet().remove(k);
}


//_____________________________________________________________________________
/**
 * Set an existing muscle in the MetabolicMuscleParameterSet 
 * to use an provided muscle mass.
 */
void Umberger2010MuscleMetabolicsProbe::
    useProvidedMass(const string& muscleName, double providedMass)
{
    Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter* mm = 
        updMetabolicParameters(muscleName);

    mm->set_use_provided_muscle_mass(true);
    mm->set_provided_muscle_mass(providedMass);
    mm->setMuscleMass();      // actual mass used.
}


//_____________________________________________________________________________
/**
 * Set an existing muscle in the MetabolicMuscleParameterSet 
 * to calculate its own mass.
 */
void Umberger2010MuscleMetabolicsProbe::
    useCalculatedMass(const string& muscleName)
{
    Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter* mm = 
        updMetabolicParameters(muscleName);

    mm->set_use_provided_muscle_mass(false);
    mm->setMuscleMass();       // actual mass used.
}


//_____________________________________________________________________________
/**
 * Get whether the muscle mass is being explicitly provided.
 * True means that it is using the property <provided_muscle_mass>
 * False means that the muscle mass is being calculated from muscle properties. 
 */
bool Umberger2010MuscleMetabolicsProbe::
    isUsingProvidedMass(const std::string& muscleName)
{ 
    return getMetabolicParameters(muscleName)->get_use_provided_muscle_mass(); 
}


//_____________________________________________________________________________
/**
 * Get the muscle mass used in the metabolic analysis. 
 */
double Umberger2010MuscleMetabolicsProbe::
    getMuscleMass(const std::string& muscleName) const 
{ 
    return getMetabolicParameters(muscleName)->getMuscleMass();
}


//_____________________________________________________________________________
/**
 * Get the ratio of slow twitch fibers for an existing muscle. 
 */
double Umberger2010MuscleMetabolicsProbe::
    getRatioSlowTwitchFibers(const std::string& muscleName) const 
{ 
    return getMetabolicParameters(muscleName)->get_ratio_slow_twitch_fibers();
}


//_____________________________________________________________________________
/**
 * Set the ratio of slow twitch fibers for an existing muscle. 
 */
void Umberger2010MuscleMetabolicsProbe::
    setRatioSlowTwitchFibers(const std::string& muscleName, const double& ratio) 
{ 
    updMetabolicParameters(muscleName)->set_ratio_slow_twitch_fibers(ratio);
}


//_____________________________________________________________________________
/**
 * Get the density for an existing muscle (kg/m^3).. 
 */
double Umberger2010MuscleMetabolicsProbe::
    getDensity(const std::string& muscleName) const 
{ 
    return getMetabolicParameters(muscleName)->get_density();
}


//_____________________________________________________________________________
/**
 * Set the density for an existing muscle (kg/m^3). 
 */
void Umberger2010MuscleMetabolicsProbe::
    setDensity(const std::string& muscleName, const double& density) 
{ 
    updMetabolicParameters(muscleName)->set_density(density);
}


//_____________________________________________________________________________
/**
 * Get the specific tension for an existing muscle (Pascals (N/m^2)). 
 */
double Umberger2010MuscleMetabolicsProbe::
    getSpecificTension(const std::string& muscleName) const 
{ 
    return getMetabolicParameters(muscleName)->get_specific_tension();
}


//_____________________________________________________________________________
/**
 * Set the specific tension for an existing muscle (Pascals (N/m^2)). 
 */
void Umberger2010MuscleMetabolicsProbe::
    setSpecificTension(const std::string& muscleName, const double& specificTension) 
{ 
    updMetabolicParameters(muscleName)->set_specific_tension(specificTension);
}





//_____________________________________________________________________________
/**
 * PRIVATE: Get const MetabolicMuscleParameter from the MuscleMap using a 
 * string accessor.
 */
const Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter* 
    Umberger2010MuscleMetabolicsProbe::getMetabolicParameters(
    const std::string& muscleName) const
{
    MuscleMap::const_iterator m_i = _muscleMap.find(muscleName);
    if (m_i == _muscleMap.end()) {
        stringstream errorMessage;
        errorMessage << getConcreteClassName() << ": Invalid muscle " 
            << muscleName << " in the MetabolicMuscleParameter map." << endl;
        throw (Exception(errorMessage.str()));
    }
    return m_i->second;
}


//_____________________________________________________________________________
/**
 * PRIVATE: Get writable MetabolicMuscleParameter from the MuscleMap using a 
 * string accessor.
 */
Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter* 
    Umberger2010MuscleMetabolicsProbe::updMetabolicParameters(
    const std::string& muscleName)
{
    MuscleMap::const_iterator m_i = _muscleMap.find(muscleName);
    if (m_i == _muscleMap.end()) {
        stringstream errorMessage;
        errorMessage << getConcreteClassName() << ": Invalid muscle " 
            << muscleName << " in the MetabolicMuscleParameter map." << endl;
        throw (Exception(errorMessage.str()));
    }
    return m_i->second;
}




//==============================================================================
//                          MetabolicMuscleParameter
//==============================================================================
//--------------------------------------------------------------------------
// Constructors
//--------------------------------------------------------------------------
Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter::
Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter() 
{
    setNull();
    constructProperties(); 
}

Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter::
Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter(
    const std::string& muscleName,
    double ratio_slow_twitch_fibers, 
    double muscle_mass)
{
    setNull();
    constructProperties();
    setName(muscleName);
    set_ratio_slow_twitch_fibers(ratio_slow_twitch_fibers);
    if (isNaN(muscle_mass)) {
        set_use_provided_muscle_mass(false);
    }
    else {
        set_use_provided_muscle_mass(true);
        set_provided_muscle_mass(muscle_mass);
    }
    
}


//--------------------------------------------------------------------------
// Set muscle mass
//--------------------------------------------------------------------------
void Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter::
setMuscleMass()    
{ 
    if (get_use_provided_muscle_mass())
        _muscMass = get_provided_muscle_mass();
    else {
        _muscMass = (_musc->getMaxIsometricForce() / get_specific_tension()) 
                    * get_density() 
                    * _musc->getOptimalFiberLength();
        }
}


//--------------------------------------------------------------------------
// Object interface
//--------------------------------------------------------------------------
void Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter::setNull()
{
    setAuthors("Tim Dorn");
    // Actual muscle mass used. If <use_provided_muscle_mass> == true, 
    // this value will set to the property value <muscle_mass> provided by the 
    // user. If <use_provided_muscle_mass> == false, then this value
    // will be set (by the metabolic probes) to the calculated mass based on
    // the muscle's Fmax, optimal fiber length, specific tension & muscle density. 
    _muscMass = SimTK::NaN;
    _musc = NULL;
}

void Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter::
constructProperties()
{
    constructProperty_specific_tension(0.25e6);  // (Pascals (N/m^2)), specific tension of mammalian muscle.
    constructProperty_density(1059.7);           // (kg/m^3), density of mammalian muscle.
    constructProperty_ratio_slow_twitch_fibers(0.5);
    constructProperty_use_provided_muscle_mass(false);
    constructProperty_provided_muscle_mass(SimTK::NaN);
}

