// MuscleMetabolicPowerProbeBhargava2004.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Tim Dorn
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* Copyright (c)  2012 Stanford University
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


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
    constructProperty_normalize_mechanical_work_rate_by_muscle_mass(false);
    constructProperty_MetabolicMuscleSet(MetabolicMuscleSet());
}


//=============================================================================
// GET METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Get whether to include the Activation Heat Rate in the overall
 * metabolic energy rate calculation (true/false).
 */
bool MuscleMetabolicPowerProbeBhargava2004::isActivationRateOn() const
{
    return get_activation_rate_on();
}

//_____________________________________________________________________________
/**
 * Get whether to include the Maintenance Heat Rate in the overall
 * metabolic energy rate calculation (true/false).
 */
bool MuscleMetabolicPowerProbeBhargava2004::isMaintenanceRateOn() const
{
    return get_maintenance_rate_on();
}

//_____________________________________________________________________________
/**
 * Get whether to include the Shortening Heat Rate in the overall
 * metabolic energy rate calculation (true/false).
 */
bool MuscleMetabolicPowerProbeBhargava2004::isShorteningRateOn() const
{
    return get_shortening_rate_on();
}

//_____________________________________________________________________________
/**
 * Get whether to include the Basal Heat Rate in the overall
 * metabolic energy rate calculation (true/false).
 */
bool MuscleMetabolicPowerProbeBhargava2004::isBasalRateOn() const
{
    return get_basal_rate_on();
}

//_____________________________________________________________________________
/**
 * Get whether to include the Mechanical Work Rate in the overall
 * metabolic energy rate calculation (true/false).
 */
bool MuscleMetabolicPowerProbeBhargava2004::isWorkRateOn() const
{
    return get_mechanical_work_rate_on();
}


//_____________________________________________________________________________
/**
 * Get the PiecewiseLinearFunction that defines the fiber length dependence on
 * the Maintenance Heat Rate.
 */
PiecewiseLinearFunction MuscleMetabolicPowerProbeBhargava2004::getFiberLengthDependenceMaintenanceRateFunction() const
{
    return get_normalized_fiber_length_dependence_on_maintenance_rate();
}

//_____________________________________________________________________________
/**
 * Get whether to use the force dependent shortening proportionality constant
 * in the calculation of the Shortening Heat Rate (true/false).
 */
bool MuscleMetabolicPowerProbeBhargava2004::usingForceDepShorteningPropConstant() const
{
    return get_use_force_dependent_shortening_prop_constant();
}

//_____________________________________________________________________________
/**
 * Get the basal coefficient used in the calculation of the Basal Heat Rate.
 */
double MuscleMetabolicPowerProbeBhargava2004::getBasalCoefficient() const
{
    return get_basal_coefficient();
}

//_____________________________________________________________________________
/**
 * Get the basal exponent used in the calculation of the Basal Heat Rate.
 */
double MuscleMetabolicPowerProbeBhargava2004::getBasalExponent() const
{
    return get_basal_exponent();
}

//_____________________________________________________________________________
/**
 * Get whether the mechanical work rate is to be normalized by muscle mass (true/false).
 */
bool MuscleMetabolicPowerProbeBhargava2004::isMechanicalWorkRateNormalizedToMuscleMass() const
{
    return get_normalize_mechanical_work_rate_by_muscle_mass();
}

//_____________________________________________________________________________
/**
 * Get a const of the MetabolicMuscleSet containing the set of MetabolicMuscle parameters
 * that are specific to the calculation of the total muscle metabolic energy rate.
 */
const MetabolicMuscleSet& MuscleMetabolicPowerProbeBhargava2004::getMetabolicMuscleSet() const
{
    return get_MetabolicMuscleSet();
}

//_____________________________________________________________________________
/**
 * Get an updatable reference to the MetabolicMuscleSet containing the set of 
 * MetabolicMuscle parameters that are specific to the calculation of the total
 * muscle metabolic energy rate.
 */
MetabolicMuscleSet& MuscleMetabolicPowerProbeBhargava2004::updMetabolicMuscleSet()
{
    return upd_MetabolicMuscleSet();
}


//=============================================================================
// SET METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set whether to include the Activation Heat Rate in the overall
 * metabolic energy rate calculation.
 */
void MuscleMetabolicPowerProbeBhargava2004::setActivationRateOn(const bool aActRateOn)
{
    set_activation_rate_on(aActRateOn);
}

//_____________________________________________________________________________
/**
 * Set whether to include the Maintenance Heat Rate in the overall
 * metabolic energy rate calculation.
 */
void MuscleMetabolicPowerProbeBhargava2004::setMaintenanceRateOn(const bool aMainRateOn)
{
    set_maintenance_rate_on(aMainRateOn);
}

//_____________________________________________________________________________
/**
 * Set whether to include the Shortening Heat Rate in the overall
 * metabolic energy rate calculation.
 */
void MuscleMetabolicPowerProbeBhargava2004::setShorteningRateOn(const bool aShortRateOn)
{
    set_shortening_rate_on(aShortRateOn);
}

//_____________________________________________________________________________
/**
 * Set whether to include the Basal Heat Rate in the overall
 * metabolic energy rate calculation.
 */
void MuscleMetabolicPowerProbeBhargava2004::setBasalRateOn(const bool aBasalRateOn)
{
    set_basal_rate_on(aBasalRateOn);
}

//_____________________________________________________________________________
/**
 * Set whether to include the Mechanical Work Rate in the overall
 * metabolic energy rate calculation.
 */
void MuscleMetabolicPowerProbeBhargava2004::setWorkRateOn(const bool aWorkRateOn)
{
    set_mechanical_work_rate_on(aWorkRateOn);
}


//_____________________________________________________________________________
/**
 * Set the PiecewiseLinearFunction that defines the fiber length dependence on
 * the Maintenance Heat Rate.
 */
void MuscleMetabolicPowerProbeBhargava2004::setFiberLengthDependenceMaintenanceRateFunction(const PiecewiseLinearFunction aFunct)
{
    set_normalized_fiber_length_dependence_on_maintenance_rate(aFunct);
}

//_____________________________________________________________________________
/**
 * Set whether to use the force dependent shortening proportionality constant
 * in the calculation of the Shortening Heat Rate.
 */
void MuscleMetabolicPowerProbeBhargava2004::setUsingForceDepShorteningPropConstant(const bool aUseForceDepShortPropConst)
{
    set_use_force_dependent_shortening_prop_constant(aUseForceDepShortPropConst);
}

//_____________________________________________________________________________
/**
 * Set the basal coefficient used in the calculation of the Basal Heat Rate.
 */
void MuscleMetabolicPowerProbeBhargava2004::setBasalCoefficient(const double aBasalCoeff)
{
    set_basal_coefficient(aBasalCoeff);
}

//_____________________________________________________________________________
/**
 * Set the basal exponent used in the calculation of the Basal Heat Rate.
 */
void MuscleMetabolicPowerProbeBhargava2004::setBasalExponent(const double aBasalExp)
{
    set_basal_exponent(aBasalExp);
}

//_____________________________________________________________________________
/**
 * Set whether the mechanical work rate is to be normalized by muscle mass (true/false).
 */
void MuscleMetabolicPowerProbeBhargava2004::setMechanicalWorkRateNormalizedToMuscleMass(const bool normalizeWorkRate)
{
    set_normalize_mechanical_work_rate_by_muscle_mass(normalizeWorkRate);
}

//_____________________________________________________________________________
/**
 * Set the MetabolicMuscleSet containing the set of MetabolicMuscle parameters
 * that are specific to the calculation of the total muscle metabolic energy rate.
 */
void MuscleMetabolicPowerProbeBhargava2004::setMetabolicMuscleSet(const MetabolicMuscleSet mms)
{
    set_MetabolicMuscleSet(mms);
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
    if (isBasalRateOn())
    {
        Bdot = getBasalCoefficient() * pow(_model->getMatterSubsystem().calcSystemMass(s), getBasalExponent());
    }
    

    // Loop through each muscle in the MetabolicMuscleSet
    int nM = getMetabolicMuscleSet().getSize();
    Vector Edot(nM);
    for (int i=0; i<nM; i++)
    {
        // Get a pointer to the current muscle in the model
        MetabolicMuscle mm = getMetabolicMuscleSet().get(i);
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
            cout << "WARNING: " << getName() << "  (t = " << s.getTime() << "), muscle '" << m->getName() << "' has negative normalized fiber-length." << endl; 



        // ACTIVATION HEAT RATE for muscle i
        // ------------------------------------------
        if (isActivationRateOn())
        {
            const double decay_function_value = 1.0;    // This value is set to 1.0, as used by Anderson & Pandy (1999), however, in
                                                        // Bhargava et al., (2004) they assume a function here. We will ignore this
                                                        // function and use 1.0 for now.
            Adot = mm.getMuscleMass() * decay_function_value * 
                ( (mm.getActivationConstantSlowTwitch() * slow_twitch_excitation) + (mm.getActivationConstantFastTwitch() * fast_twitch_excitation) );
        }



        // MAINTENANCE HEAT RATE for muscle i
        // ------------------------------------------
        if (isMaintenanceRateOn())
        {
            Vector tmp(1, fiber_length_normalized);
            fiber_length_dependence = getFiberLengthDependenceMaintenanceRateFunction().calcValue(tmp);
            
            Mdot = mm.getMuscleMass() * fiber_length_dependence * 
                ( (mm.getMaintenanceConstantSlowTwitch() * slow_twitch_excitation) + (mm.getMaintenanceConstantFastTwitch() * fast_twitch_excitation) );
        }



        // SHORTENING HEAT RATE for muscle i
        // --> note that we define Vm<0 as shortening and Vm>0 as lengthening
        // -----------------------------------------------------------------------
        if (isShorteningRateOn())
        {
            if (usingForceDepShorteningPropConstant())
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
        if (isWorkRateOn())
        {
            if (fiber_velocity <= 0)    // concentric contraction, Vm<0
                Wdot = -fiber_force_active*fiber_velocity;
            else						// eccentric contraction, Vm>0
                Wdot = 0;

            if (isMechanicalWorkRateNormalizedToMuscleMass())
                Wdot /= mm.getMuscleMass();
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
            system("pause");
        }
    }

    SimTK::Vector EdotTotal(1, Edot.sum() + Bdot);

	if(EdotTotal(0) < 1.0 && isActivationRateOn() && isMaintenanceRateOn() && isShorteningRateOn() && isWorkRateOn()) {
			cout << "WARNING: " << getName() << "  (t = " << s.getTime() << "), the model has a net metabolic energy rate of less than 1.0 W.kg-1." << endl; 
			EdotTotal(0) = 1.0;			// not allowed to fall below 1.0 W.kg-1
	}

    return EdotTotal;
}


//_____________________________________________________________________________
/** 
 * Provide labels for the probe values being reported.
 */
Array<string> MuscleMetabolicPowerProbeBhargava2004::getProbeLabels() const 
{
    Array<string> labels;

    if (getScaleFactor() != 1.0) {
        char n[10];
        sprintf(n, "%f", getScaleFactor());
        labels.append(getName()+"_SCALED_BY_"+n+"X");
    }
    else
        labels.append(getName()+"_"+getOperation());

    return labels;
}


//_____________________________________________________________________________
/**
 * Check that the MetabolicMuscle is a valid object.
 * If all tests pass, a pointer to the muscle in the model is returned.
 *
 * @param mm MetabolicMuscle object to check
 * @return *musc Muscle object in model
 */
Muscle* MuscleMetabolicPowerProbeBhargava2004::checkValidMetabolicMuscle(MetabolicMuscle mm) const
{
    string errorMessage;
    Muscle* musc;

    // check that the muscle exists
    int k = _model->getMuscles().getIndex(mm.getName());
    if( k < 0 )	{
        errorMessage = "MetabolicMuscle: Invalid muscle '" + mm.getName() + "' specified.";
        throw (Exception(errorMessage.c_str()));
    }
    else {
        musc = &_model->updMuscles().get(k);
    }

    // error checking: muscle_mass
    if (mm.getMuscleMass() <= 0) {
        errorMessage = "MetabolicMuscle: Invalid muscle_mass for muscle: " + mm.getName() + ". muscle_mass must be positive.";
        throw (Exception(errorMessage.c_str()));
    }

    // error checking: ratio_slow_twitch_fibers
    if (mm.getRatioSlowTwitchFibers() < 0 || mm.getRatioSlowTwitchFibers() > 1)	{
        errorMessage = "MetabolicMuscle: Invalid ratio_slow_twitch_fibers for muscle: " + mm.getName() + ". ratio_slow_twitch_fibers must be between 0 and 1.";
        throw (Exception(errorMessage.c_str()));
    }

    //cout << "VALID muscle: " << mm.getName() << endl;
    return musc;
}