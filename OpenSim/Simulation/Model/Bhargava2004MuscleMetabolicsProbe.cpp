/* -------------------------------------------------------------------------- *
 *              OpenSim:  Bhargava2004MuscleMetabolicsProbe.cpp               *
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
#include "Bhargava2004MuscleMetabolicsProbe.h"
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
Bhargava2004MuscleMetabolicsProbe::Bhargava2004MuscleMetabolicsProbe() : Probe()
{
    setNull();
    constructProperties();
}

//_____________________________________________________________________________
/** 
 * Convenience constructor
 */
Bhargava2004MuscleMetabolicsProbe::Bhargava2004MuscleMetabolicsProbe(
    const bool activation_rate_on, 
    const bool maintenance_rate_on, 
    const bool shortening_rate_on, 
    const bool basal_rate_on, 
    const bool work_rate_on) : Probe()
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
 * Set the data members of this Bhargava2004MuscleMetabolicsProbe to their null values.
 */
void Bhargava2004MuscleMetabolicsProbe::setNull()
{
	setAuthors("Tim Dorn");
	setReferences("Bhargava, L. J., Pandy, M. G. and Anderson, F. C. (2004). " 
		"A phenomenological model for estimating metabolic energy consumption "
		"in muscle contraction. J Biomech 37, 81-8..");
    _muscleMap.clear();
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Bhargava2004MuscleMetabolicsProbe::constructProperties()
{
    constructProperty_activation_rate_on(true);
    constructProperty_maintenance_rate_on(true);
    constructProperty_shortening_rate_on(true);
    constructProperty_basal_rate_on(true);
    constructProperty_mechanical_work_rate_on(true);
    constructProperty_enforce_minimum_heat_rate_per_muscle(true);

    const int curvePoints = 5;
    const double curveX[] = {0.0, 0.5, 1.0, 1.5, 2.0};
    const double curveY[] = {0.5, 0.5, 1.0, 0.0, 0.0};
    PiecewiseLinearFunction fiberLengthDepCurveDefault(curvePoints, curveX, curveY, "defaultCurve");
    constructProperty_normalized_fiber_length_dependence_on_maintenance_rate(fiberLengthDepCurveDefault);

    constructProperty_use_force_dependent_shortening_prop_constant(false);
    constructProperty_basal_coefficient(1.2);  // default value for standing (Umberger, 2003, p105)
    constructProperty_basal_exponent(1.0);
    constructProperty_report_total_metabolics_only(true);
    constructProperty_Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameterSet
       (Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameterSet());
}



//=============================================================================
// MODEL COMPONENT METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this Bhargava2004MuscleMetabolicsProbe.
 */
void Bhargava2004MuscleMetabolicsProbe::connectToModel(Model& aModel)
{
    Super::connectToModel(aModel);
    if (isDisabled()) return;   // Nothing to connect

    const int nM = 
        get_Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameterSet()
        .getSize();
    for (int i=0; i<nM; ++i) {
        connectIndividualMetabolicMuscle(aModel, 
            upd_Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameterSet()[i]);
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
void Bhargava2004MuscleMetabolicsProbe::connectIndividualMetabolicMuscle(
    Model& aModel, 
    Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameter& mm)
{
    stringstream errorMessage;

    int k = aModel.getMuscles().getIndex(mm.getName());
    if( k < 0 )	{
        cout << "WARNING: Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameter: "
            "Muscle '" << mm.getName() << "' not found in model. Ignoring..." << endl;
        setDisabled(true);
        return;

    }
    else {
        mm.setMuscle(&aModel.updMuscles()[k]);  // Set internal muscle pointer
        _muscleMap[mm.getName()] = &mm;          // Add parameters to the _muscleMap
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
             std::cout << "WARNING: " << errorMessage.str() << "Probe will be disabled." << std::endl;
             setDisabled(true);
            //throw (Exception(errorMessage.c_str()));
        }
        else if (isnan(mm.get_provided_muscle_mass())) {
            errorMessage << "ERROR: No <provided_muscle_mass> specified for " 
                << mm.getName() 
                << ". <provided_muscle_mass> must be a positive number (kg)." << endl;
             std::cout << "WARNING: " << errorMessage.str() << "Probe will be disabled." << std::endl;
             setDisabled(true);
            //throw (Exception(errorMessage.c_str()));
        }
    }

    else {

        // Check that <specific_tension> and <density> have been correctly specified.
        if (mm.get_specific_tension() <= 0) {
            errorMessage << "ERROR: Negative <specific_tension> specified for " 
                << mm.getName() 
                << ". <specific_tension> must be a positive number (N/m^2)." << endl;
            std::cout << "WARNING: " << errorMessage.str() << "Probe will be disabled." << std::endl;
            setDisabled(true);
            //throw (Exception(errorMessage.c_str()));

        }
        if (mm.get_density() <= 0) {
            errorMessage << "ERROR: Negative <density> specified for " 
                << mm.getName() 
                << ". <density> must be a positive number (kg/m^3)." << endl;
            std::cout << "WARNING: " << errorMessage.str() << "Probe will be disabled." << std::endl;
            setDisabled(true);
        }
    }


    // -----------------------------------------------------------------------
    // Check that <ratio_slow_twitch_fibers> is between 0 and 1.
    // -----------------------------------------------------------------------
    if (mm.get_ratio_slow_twitch_fibers() < 0 || mm.get_ratio_slow_twitch_fibers() > 1)	{
        errorMessage << "MetabolicMuscleParameter: Invalid ratio_slow_twitch_fibers for muscle: " 
            << getName() << ". ratio_slow_twitch_fibers must be between 0 and 1." << endl;
        std::cout << "WARNING: " << errorMessage.str() << "Probe will be disabled." << std::endl;
        setDisabled(true);
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
SimTK::Vector Bhargava2004MuscleMetabolicsProbe::
computeProbeInputs(const State& s) const
{
    // Initialize metabolic energy rate values
    double Adot, Mdot, Sdot, Bdot, Wdot;
    Adot = Mdot = Sdot = Bdot = Wdot = 0;
    Vector EdotOutput(getNumProbeInputs());
    EdotOutput = 0;


    // BASAL METABOLIC RATE (W) (based on whole body mass, not muscle mass)
    // so do outside of muscle loop.
    // TODO: system mass should be precalculated.
    // ------------------------------------------------------------------
    if (get_basal_rate_on()) {
        Bdot = get_basal_coefficient() 
            * pow(_model->getMatterSubsystem().calcSystemMass(s), get_basal_exponent());
        if (Bdot == NaN)
            cout << "WARNING::" << getName() << ": Bdot = NaN!" << endl;
    }
    EdotOutput(0) += Bdot;       // TOTAL metabolic power storage
    
    if (!get_report_total_metabolics_only())
        EdotOutput(1) = Bdot;    // BASAL metabolic power storage


    // Loop through each muscle in the MetabolicMuscleParameterSet
    const int nM = 
        get_Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameterSet()
        .getSize();
    for (int i=0; i<nM; i++)
    {
        // Get the current muscle parameters from the MetabolicMuscleParameterSet
        // and the corresponding OpenSim::Muscle pointer from the muscleMap.
        Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameter& mm = 
            get_Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameterSet()[i];   
        const Muscle* m = mm.getMuscle();

        // Get important muscle values at the current time state
        const double max_isometric_force = m->getMaxIsometricForce();
        const double max_shortening_velocity = m->getMaxContractionVelocity();
        const double activation = m->getActivation(s);
        const double excitation = m->getControl(s);
        const double fiber_force_passive = m->getPassiveFiberForce(s);
        const double fiber_force_active = m->getActiveFiberForce(s);
        const double fiber_force_total = m->getFiberForce(s);
        const double fiber_length_normalized = m->getNormalizedFiberLength(s);
        const double fiber_velocity = m->getFiberVelocity(s);
        const double fiber_velocity_normalized = m->getNormalizedFiberVelocity(s);
        const double slow_twitch_excitation = mm.get_ratio_slow_twitch_fibers() * sin(Pi/2 * excitation);
        const double fast_twitch_excitation = (1 - mm.get_ratio_slow_twitch_fibers()) * (1 - cos(Pi/2 * excitation));
        double alpha, fiber_length_dependence;

        // Get the unnormalized total active force, F_iso that 'would' be developed at the current activation
        // and fiber length under isometric conditions (i.e. Vm=0)
        const double F_iso = m->getActivation(s) * m->getActiveForceLengthMultiplier(s) * max_isometric_force;

        // Warnings
        if (fiber_length_normalized < 0)
            cout << "WARNING: " << getName() << "  (t = " << s.getTime() 
            << "), muscle '" << m->getName() 
            << "' has negative normalized fiber-length." << endl; 



        // ACTIVATION HEAT RATE for muscle i (W)
        // ------------------------------------------
        if (get_activation_rate_on())
        {
            const double decay_function_value = 1.0;    // This value is set to 1.0, as used by Anderson & Pandy (1999), however, in
                                                        // Bhargava et al., (2004) they assume a function here. We will ignore this
                                                        // function and use 1.0 for now.
            Adot = mm.getMuscleMass() * decay_function_value * 
                ( (mm.get_activation_constant_slow_twitch() * slow_twitch_excitation) + (mm.get_activation_constant_fast_twitch() * fast_twitch_excitation) );
        }



        // MAINTENANCE HEAT RATE for muscle i (W)
        // ------------------------------------------
        if (get_maintenance_rate_on())
        {
            Vector tmp(1, fiber_length_normalized);
            fiber_length_dependence = get_normalized_fiber_length_dependence_on_maintenance_rate().calcValue(tmp);
            
            Mdot = mm.getMuscleMass() * fiber_length_dependence * 
                ( (mm.get_maintenance_constant_slow_twitch() * slow_twitch_excitation) + (mm.get_maintenance_constant_fast_twitch() * fast_twitch_excitation) );
        }



        // SHORTENING HEAT RATE for muscle i (W)
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
                    alpha = 0.25 * fiber_force_total;
                else						// eccentric contraction, Vm>0
                    alpha = 0.0;
            }
            Sdot = -alpha * fiber_velocity;
        }
        


        // MECHANICAL WORK RATE for the contractile element of muscle i (W).
        // --> note that we define Vm<0 as shortening and Vm>0 as lengthening.
        // -------------------------------------------------------------------
        if (get_mechanical_work_rate_on())
        {
            if (fiber_velocity <= 0)    // concentric contraction, Vm<0
                Wdot = -fiber_force_active*fiber_velocity;
            else						// eccentric contraction, Vm>0
                Wdot = 0;
        }


        // NAN CHECKING
        // ------------------------------------------
        if (Adot == NaN)
            cout << "WARNING::" << getName() << ": Adot (" << m->getName() << ") = NaN!" << endl;
        if (Mdot == NaN)
            cout << "WARNING::" << getName() << ": Mdot (" << m->getName() << ") = NaN!" << endl;
        if (Sdot == NaN)
            cout << "WARNING::" << getName() << ": Sdot (" << m->getName() << ") = NaN!" << endl;
        if (Wdot == NaN)
            cout << "WARNING::" << getName() << ": Wdot (" << m->getName() << ") = NaN!" << endl;


        // This check is adapted from Umberger(2003), page 104: the total heat rate 
        // (i.e., Adot + Mdot + Sdot) for a given muscle cannot fall below 1.0 W/kg.
        // -----------------------------------------------------------------------
        double totalHeatRate = Adot + Mdot + Sdot;      // (W)

        if(get_enforce_minimum_heat_rate_per_muscle() && totalHeatRate < 1.0 * mm.getMuscleMass()
            && get_activation_rate_on() 
            && get_maintenance_rate_on() 
            && get_shortening_rate_on()) {
                //cout << "WARNING: " << getName() 
                //    << "  (t = " << s.getTime() 
                //    << "), the muscle '" << mm.getName() 
                //    << "' has a net metabolic energy rate of less than 1.0 W/kg." << endl; 
                totalHeatRate = 1.0 * mm.getMuscleMass();			// not allowed to fall below 1.0 W.kg-1
        }


        // TOTAL METABOLIC ENERGY RATE for muscle i (W)
        // ------------------------------------------
        const double Edot = totalHeatRate + Wdot;
        EdotOutput(0) += Edot;       // Add to TOTAL metabolic power storage
        if (!get_report_total_metabolics_only()) {
            // Metabolic power storage for muscle i
            EdotOutput(i+2) = Edot;  
        }  



#ifdef DEBUG_METABOLICS
        cout << "muscle_mass = " << mm.getMuscleMass() << endl;
        cout << "ratio_slow_twitch_fibers = " << mm.get_ratio_slow_twitch_fibers() << endl;
        cout << "activation_constant_slow_twitch = " << mm.get_activation_constant_slow_twitch() << endl;
        cout << "activation_constant_fast_twitch = " << mm.get_activation_constant_fast_twitch() << endl;
        cout << "maintenance_constant_slow_twitch = " << mm.get_maintenance_constant_slow_twitch() << endl;
        cout << "maintenance_constant_fast_twitch = " << mm.get_maintenance_constant_fast_twitch() << endl;
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
int Bhargava2004MuscleMetabolicsProbe::getNumProbeInputs() const
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
Array<string> Bhargava2004MuscleMetabolicsProbe::getProbeOutputLabels() const 
{
    Array<string> labels;
    labels.append(getName()+"_TOTAL");

    if (get_report_total_metabolics_only())
        return labels;

    labels.append(getName()+"_BASAL");

    for (int i=0; i<getNumMetabolicMuscles(); ++i)
        labels.append(getName()+"_"+get_Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameterSet()[i].getName());

    return labels;
}




//=============================================================================
// MUSCLE METABOLICS INTERFACE
//=============================================================================
//_____________________________________________________________________________
/** 
* Get the number of muscles being analysed in the metabolic analysis. 
*/
const int Bhargava2004MuscleMetabolicsProbe::
	getNumMetabolicMuscles() const  
{ 
	return get_Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameterSet()
        .getSize(); 
}


//_____________________________________________________________________________
/**
 * Add a muscle and its parameters so that it can be included in the
 * metabolic analysis.
 */
void Bhargava2004MuscleMetabolicsProbe::
	addMuscle(const string& muscleName, 
    double ratio_slow_twitch_fibers, 
    double activation_constant_slow_twitch,
    double activation_constant_fast_twitch,
    double maintenance_constant_slow_twitch,
    double maintenance_constant_fast_twitch)
{
    Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameter* mm = 
        new Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameter(
            muscleName,
            ratio_slow_twitch_fibers, 
            activation_constant_slow_twitch, 
            activation_constant_fast_twitch, 
            maintenance_constant_slow_twitch, 
            maintenance_constant_fast_twitch);
        
    connectIndividualMetabolicMuscle(*_model, *mm);          // do checks and add to muscleMap 
    upd_Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameterSet()
        .adoptAndAppend(mm);    // add to MetabolicMuscleParameterSet in the model
}


//_____________________________________________________________________________
/**
 * Add a muscle and its parameters so that it can be included in the
 * metabolic analysis.
 */
void Bhargava2004MuscleMetabolicsProbe::
	addMuscle(const string& muscleName, 
    double ratio_slow_twitch_fibers, 
    double activation_constant_slow_twitch,
    double activation_constant_fast_twitch,
    double maintenance_constant_slow_twitch,
    double maintenance_constant_fast_twitch,
	double muscle_mass)
{
    Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameter* mm = 
        new Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameter(
            muscleName,
            ratio_slow_twitch_fibers, 
            activation_constant_slow_twitch, 
            activation_constant_fast_twitch, 
            maintenance_constant_slow_twitch, 
            maintenance_constant_fast_twitch,
			muscle_mass);
        
    connectIndividualMetabolicMuscle(*_model, *mm);          // do checks and add to muscleMap 
    upd_Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameterSet()
        .adoptAndAppend(mm);    // add to MetabolicMuscleParameterSet in the model
}


//_____________________________________________________________________________
/**
 * Remove a muscle from the MetabolicMuscleParameterSet.
 */
void Bhargava2004MuscleMetabolicsProbe::
	removeMuscle(const string& muscleName)
{
    // Step 1: Remove the reference to this MetabolicMuscleParameter
    // from the muscle map.
    // -----------------------------------------------------------------
    _muscleMap.erase(muscleName);


    // Step 2: Remove the MetabolicMuscleParameter object from
    // the MetabolicMuscleParameterSet.
    // -----------------------------------------------------------------
    const int k = 
        get_Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameterSet()
        .getIndex(muscleName);
    if (k<0) {
        cout << "WARNING: MetabolicMuscleParameter: Invalid muscle '" 
            << muscleName << "' specified. No metabolic muscles removed." << endl;
        return;
    }
    upd_Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameterSet()
        .remove(k);
}


//_____________________________________________________________________________
/**
 * Set an existing muscle in the MetabolicMuscleParameterSet 
 * to use an provided muscle mass.
 */
void Bhargava2004MuscleMetabolicsProbe::
	useProvidedMass(const string& muscleName, double providedMass)
{
    Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameter* mm = 
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
void Bhargava2004MuscleMetabolicsProbe::
	useCalculatedMass(const string& muscleName)
{
    Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameter* mm = 
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
bool Bhargava2004MuscleMetabolicsProbe::
	isUsingProvidedMass(const std::string& muscleName)
{ 
	return getMetabolicParameters(muscleName)->get_use_provided_muscle_mass(); 
}


//_____________________________________________________________________________
/**
 * Get the muscle mass used in the metabolic analysis. 
 */
const double Bhargava2004MuscleMetabolicsProbe::
	getMuscleMass(const std::string& muscleName) const 
{ 
	return getMetabolicParameters(muscleName)->getMuscleMass();
}


//_____________________________________________________________________________
/**
 * Get the ratio of slow twitch fibers for an existing muscle. 
 */
const double Bhargava2004MuscleMetabolicsProbe::
	getRatioSlowTwitchFibers(const std::string& muscleName) const 
{ 
	return getMetabolicParameters(muscleName)->get_ratio_slow_twitch_fibers();
}


//_____________________________________________________________________________
/**
 * Set the ratio of slow twitch fibers for an existing muscle. 
 */
void Bhargava2004MuscleMetabolicsProbe::
	setRatioSlowTwitchFibers(const std::string& muscleName, const double& ratio) 
{ 
	updMetabolicParameters(muscleName)->set_ratio_slow_twitch_fibers(ratio);
}


//_____________________________________________________________________________
/**
 * Get the density for an existing muscle (kg/m^3).. 
 */
const double Bhargava2004MuscleMetabolicsProbe::
	getDensity(const std::string& muscleName) const 
{ 
	return getMetabolicParameters(muscleName)->get_density();
}


//_____________________________________________________________________________
/**
 * Set the density for an existing muscle (kg/m^3). 
 */
void Bhargava2004MuscleMetabolicsProbe::
	setDensity(const std::string& muscleName, const double& density) 
{ 
	updMetabolicParameters(muscleName)->set_density(density);
}


//_____________________________________________________________________________
/**
 * Get the specific tension for an existing muscle (Pascals (N/m^2)). 
 */
const double Bhargava2004MuscleMetabolicsProbe::
	getSpecificTension(const std::string& muscleName) const 
{ 
	return getMetabolicParameters(muscleName)->get_specific_tension();
}


//_____________________________________________________________________________
/**
 * Set the specific tension for an existing muscle (Pascals (N/m^2)). 
 */
void Bhargava2004MuscleMetabolicsProbe::
	setSpecificTension(const std::string& muscleName, const double& specificTension) 
{ 
	updMetabolicParameters(muscleName)->set_specific_tension(specificTension);
}


//_____________________________________________________________________________
/** 
 * Get the activation constant for slow twitch fibers for an existing muscle. 
 */
const double Bhargava2004MuscleMetabolicsProbe::
	getActivationConstantSlowTwitch(const std::string& muscleName) const
{ 
	return getMetabolicParameters(muscleName)->get_activation_constant_slow_twitch(); 
}


//_____________________________________________________________________________
/** 
 * Set the activation constant for slow twitch fibers for an existing muscle. 
 */
void Bhargava2004MuscleMetabolicsProbe::
	setActivationConstantSlowTwitch(const std::string& muscleName, const double& c) 
{ 
	updMetabolicParameters(muscleName)->set_activation_constant_slow_twitch(c); 
}


//_____________________________________________________________________________
/** 
 * Get the activation constant for fast twitch fibers for an existing muscle. 
 */
const double Bhargava2004MuscleMetabolicsProbe::
	getActivationConstantFastTwitch(const std::string& muscleName) const 
{ 
	return getMetabolicParameters(muscleName)->get_activation_constant_fast_twitch(); 
}


//_____________________________________________________________________________
/** 
 * Set the activation constant for fast twitch fibers for an existing muscle. 
 */
void Bhargava2004MuscleMetabolicsProbe::
	setActivationConstantFastTwitch(const std::string& muscleName, const double& c) 
{ 
	updMetabolicParameters(muscleName)->set_activation_constant_fast_twitch(c); 
}


//_____________________________________________________________________________
/** 
 * Get the maintenance constant for slow twitch fibers for an existing muscle. 
 */
const double Bhargava2004MuscleMetabolicsProbe::
	getMaintenanceConstantSlowTwitch(const std::string& muscleName) const 
{ 
	return getMetabolicParameters(muscleName)->get_maintenance_constant_slow_twitch(); 
}


//_____________________________________________________________________________
/** 
 * Set the maintenance constant for slow twitch fibers for an existing muscle. 
 */
void Bhargava2004MuscleMetabolicsProbe::
	setMaintenanceConstantSlowTwitch(const std::string& muscleName, const double& c) 
{ 
	updMetabolicParameters(muscleName)->set_maintenance_constant_slow_twitch(c); 
}


//_____________________________________________________________________________
/** 
 * Get the maintenance constant for fast twitch fibers for an existing muscle. 
 */
const double Bhargava2004MuscleMetabolicsProbe::
	getMaintenanceConstantFastTwitch(const std::string& muscleName) const 
{ 
	return getMetabolicParameters(muscleName)->get_maintenance_constant_fast_twitch(); 
}


//_____________________________________________________________________________
/** 
 * Set the maintenance constant for fast twitch fibers for an existing muscle. 
 */
void Bhargava2004MuscleMetabolicsProbe::
	setMaintenanceConstantFastTwitch(const std::string& muscleName, const double& c) 
{ 
	updMetabolicParameters(muscleName)->set_maintenance_constant_fast_twitch(c);
}


//_____________________________________________________________________________
/**
 * PRIVATE: Get const MetabolicMuscleParameter from the MuscleMap using a 
 * string accessor.
 */
const Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameter* 
    Bhargava2004MuscleMetabolicsProbe::getMetabolicParameters(
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
Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameter* 
    Bhargava2004MuscleMetabolicsProbe::updMetabolicParameters(
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
Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameter::
Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameter() 
{
	setNull();
	constructProperties(); 
}


Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameter::
Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameter(
	const std::string& muscleName,
	double ratio_slow_twitch_fibers, 
	double muscle_mass)
{
	setNull();
	constructProperties();
	setName(muscleName);
	set_ratio_slow_twitch_fibers(ratio_slow_twitch_fibers);

    if (isnan(muscle_mass)) {
	    set_use_provided_muscle_mass(false);
    }
    else {
        set_use_provided_muscle_mass(true);
        set_provided_muscle_mass(muscle_mass);
    }
	
}


Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameter::
Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameter(
	const std::string& muscleName,
	double ratio_slow_twitch_fibers,
	double activation_constant_slow_twitch,
    double activation_constant_fast_twitch,
    double maintenance_constant_slow_twitch,
    double maintenance_constant_fast_twitch,
	double muscle_mass)
{
	setNull();
	constructProperties();
	setName(muscleName);
	set_ratio_slow_twitch_fibers(ratio_slow_twitch_fibers);
	set_activation_constant_slow_twitch(activation_constant_slow_twitch);
    set_activation_constant_fast_twitch(activation_constant_fast_twitch);
    set_maintenance_constant_slow_twitch(maintenance_constant_slow_twitch);
    set_maintenance_constant_fast_twitch(maintenance_constant_fast_twitch);

    if (isnan(muscle_mass)) {
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
void Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameter::
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
void Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameter::setNull()
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

void Bhargava2004MuscleMetabolicsProbe_MetabolicMuscleParameter::
constructProperties()
{
	constructProperty_specific_tension(0.25e6);  // (Pascals (N/m^2)), specific tension of mammalian muscle.
	constructProperty_density(1059.7);           // (kg/m^3), density of mammalian muscle.
	constructProperty_ratio_slow_twitch_fibers(0.5);
	constructProperty_use_provided_muscle_mass(false);
	constructProperty_provided_muscle_mass(SimTK::NaN);

	// defaults from Bhargava., et al (2004).
    constructProperty_activation_constant_slow_twitch(40.0);
    constructProperty_activation_constant_fast_twitch(133.0);
    constructProperty_maintenance_constant_slow_twitch(74.0);
    constructProperty_maintenance_constant_fast_twitch(111.0);   
}

