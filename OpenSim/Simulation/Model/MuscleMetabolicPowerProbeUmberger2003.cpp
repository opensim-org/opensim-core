// MuscleMetabolicPowerProbeUmberger2003.cpp
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
#include "MuscleMetabolicPowerProbeUmberger2003.h"


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
MuscleMetabolicPowerProbeUmberger2003::MuscleMetabolicPowerProbeUmberger2003() : Probe()
{
    setNull();
    constructProperties();
}

//_____________________________________________________________________________
/** 
 * Convenience constructor
 */
MuscleMetabolicPowerProbeUmberger2003::MuscleMetabolicPowerProbeUmberger2003(bool activation_maintenance_rate_on, 
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
 * Set the data members of this MuscleMetabolicPowerProbeUmberger2003 to their null values.
 */
void MuscleMetabolicPowerProbeUmberger2003::setNull()
{
    // no data members
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void MuscleMetabolicPowerProbeUmberger2003::constructProperties()
{
    constructProperty_activation_maintenance_rate_on(true);
    constructProperty_shortening_rate_on(true);
    constructProperty_basal_rate_on(true);
    constructProperty_mechanical_work_rate_on(true);

    constructProperty_scaling_factor(1.0);
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
 * Get whether to include the Activation & Maintenance Heat Rates in the overall
 * metabolic energy rate calculation (true/false).
 */
bool MuscleMetabolicPowerProbeUmberger2003::isActivationMaintenanceRateOn() const
{
    return get_activation_maintenance_rate_on();
}

//_____________________________________________________________________________
/**
 * Get whether to include the Shortening Heat Rate in the overall
 * metabolic energy rate calculation (true/false).
 */
bool MuscleMetabolicPowerProbeUmberger2003::isShorteningRateOn() const
{
    return get_shortening_rate_on();
}

//_____________________________________________________________________________
/**
 * Get whether to include the Basal Heat Rate in the overall
 * metabolic energy rate calculation (true/false).
 */
bool MuscleMetabolicPowerProbeUmberger2003::isBasalRateOn() const
{
    return get_basal_rate_on();
}

//_____________________________________________________________________________
/**
 * Get whether to include the Mechanical Work Rate in the overall
 * metabolic energy rate calculation (true/false).
 */
bool MuscleMetabolicPowerProbeUmberger2003::isWorkRateOn() const
{
    return get_mechanical_work_rate_on();
}


//_____________________________________________________________________________
/**
 * Get the scaling factor S (see Umberger at al., (2002).
 */
double MuscleMetabolicPowerProbeUmberger2003::getScalingFactor() const
{
    return get_scaling_factor();
}

//_____________________________________________________________________________
/**
 * Get the basal coefficient used in the calculation of the Basal Heat Rate.
 */
double MuscleMetabolicPowerProbeUmberger2003::getBasalCoefficient() const
{
    return get_basal_coefficient();
}

//_____________________________________________________________________________
/**
 * Get the basal exponent used in the calculation of the Basal Heat Rate.
 */
double MuscleMetabolicPowerProbeUmberger2003::getBasalExponent() const
{
    return get_basal_exponent();
}

//_____________________________________________________________________________
/**
 * Get whether the mechanical work rate is to be normalized by muscle mass (true/false).
 */
bool MuscleMetabolicPowerProbeUmberger2003::isMechanicalWorkRateNormalizedToMuscleMass() const
{
    return get_normalize_mechanical_work_rate_by_muscle_mass();
}

//_____________________________________________________________________________
/**
 * Get a const of the MetabolicMuscleSet containing the set of MetabolicMuscle parameters
 * that are specific to the calculation of the total muscle metabolic energy rate.
 */
const MetabolicMuscleSet& MuscleMetabolicPowerProbeUmberger2003::getMetabolicMuscleSet() const
{
    return get_MetabolicMuscleSet();
}

//_____________________________________________________________________________
/**
 * Get an updatable reference to the MetabolicMuscleSet containing the set of 
 * MetabolicMuscle parameters that are specific to the calculation of the total
 * muscle metabolic energy rate.
 */
MetabolicMuscleSet& MuscleMetabolicPowerProbeUmberger2003::updMetabolicMuscleSet()
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
void MuscleMetabolicPowerProbeUmberger2003::setActivationMaintenanceRateOn(const bool aActMainRateOn)
{
    set_activation_maintenance_rate_on(aActMainRateOn);
}

//_____________________________________________________________________________
/**
 * Set whether to include the Shortening Heat Rate in the overall
 * metabolic energy rate calculation.
 */
void MuscleMetabolicPowerProbeUmberger2003::setShorteningRateOn(const bool aShortRateOn)
{
    set_shortening_rate_on(aShortRateOn);
}

//_____________________________________________________________________________
/**
 * Set whether to include the Basal Heat Rate in the overall
 * metabolic energy rate calculation.
 */
void MuscleMetabolicPowerProbeUmberger2003::setBasalRateOn(const bool aBasalRateOn)
{
    set_basal_rate_on(aBasalRateOn);
}

//_____________________________________________________________________________
/**
 * Set whether to include the Mechanical Work Rate in the overall
 * metabolic energy rate calculation.
 */
void MuscleMetabolicPowerProbeUmberger2003::setWorkRateOn(const bool aWorkRateOn)
{
    set_mechanical_work_rate_on(aWorkRateOn);
}


//_____________________________________________________________________________
/**
 * Set the scaling factor S (see Umberger at al., (2002).
 */
void MuscleMetabolicPowerProbeUmberger2003::setScalingFactor(const double S)
{
    set_scaling_factor(S);
}

//_____________________________________________________________________________
/**
 * Set the basal coefficient used in the calculation of the Basal Heat Rate.
 */
void MuscleMetabolicPowerProbeUmberger2003::setBasalCoefficient(const double aBasalCoeff)
{
    set_basal_coefficient(aBasalCoeff);
}

//_____________________________________________________________________________
/**
 * Set the basal exponent used in the calculation of the Basal Heat Rate.
 */
void MuscleMetabolicPowerProbeUmberger2003::setBasalExponent(const double aBasalExp)
{
    set_basal_exponent(aBasalExp);
}

//_____________________________________________________________________________
/**
 * Set whether the mechanical work rate is to be normalized by muscle mass (true/false).
 */
void MuscleMetabolicPowerProbeUmberger2003::setMechanicalWorkRateNormalizedToMuscleMass(const bool normalizeWorkRate)
{
    set_normalize_mechanical_work_rate_by_muscle_mass(normalizeWorkRate);
}

//_____________________________________________________________________________
/**
 * Set the MetabolicMuscleSet containing the set of MetabolicMuscle parameters
 * that are specific to the calculation of the total muscle metabolic energy rate.
 */
void MuscleMetabolicPowerProbeUmberger2003::setMetabolicMuscleSet(const MetabolicMuscleSet mms)
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
 * @param aModel OpenSim model containing this MuscleMetabolicPowerProbeUmberger2003.
 */
void MuscleMetabolicPowerProbeUmberger2003::connectToModel(Model& aModel)
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
SimTK::Vector MuscleMetabolicPowerProbeUmberger2003::computeProbeInputs(const State& s) const
{
    // Initialize metabolic energy rate values
    double AMdot, Sdot, Bdot, Wdot;
    AMdot = Sdot = Bdot = Wdot = 0;

    // BASAL METABOLIC RATE for whole body
    // ------------------------------------------
    if (isBasalRateOn() == true)
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

        // Get some muscle properties at the current time state
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
        double A, A_rel, B_rel;

        // Set activation dependence scaling parameter: A
        if (excitation > activation)
            A = excitation;
        else
            A = (excitation + activation) / 2;

        // Get the normalized active fiber force, F_iso, that 'would' be developed at the current activation
        // and fiber length under isometric conditions (i.e. Vm=0)
        double F_iso = (fiber_force_active/m->getForceVelocityMultiplier(s)) / max_isometric_force;

        // DEBUG
        //cout << "fiber_velocity_normalized = " << fiber_velocity_normalized << endl;
        //cout << "fiber_velocity_multiplier = " << m->getForceVelocityMultiplier(s) << endl;
        //cout << "fiber_force_active = " << fiber_force_active << endl;
        //cout << "fiber_force_total = " << fiber_force_total << endl;
        //cout << "max_isometric_force = " << max_isometric_force << endl;
        //cout << "F_iso = " << F_iso << endl;
        //system("pause");


        // Set normalized hill constants: A_rel and B_rel
        A_rel = 0.1 + 0.4*(1 - mm.getRatioSlowTwitchFibers());
        B_rel = A_rel * max_shortening_velocity;

        // Warnings
        if (fiber_length_normalized < 0)
            cout << "WARNING: (t = " << s.getTime() << "), muscle '" << m->getName() << "' has negative normalized fiber-length." << endl; 



        // ACTIVATION & MAINTENANCE HEAT RATE for muscle i
        // --> depends on the normalized fiber length of the contractile element
        // -----------------------------------------------------------------------
        if (isActivationMaintenanceRateOn())
        {
            double unscaledAMdot = 128*(1 - mm.getRatioSlowTwitchFibers()) + 25;

            if (fiber_length_normalized <= 1.0)
                AMdot = getScalingFactor() * std::pow(A, 0.6) * unscaledAMdot;
            else
                AMdot = getScalingFactor() * std::pow(A, 0.6) * ((0.4 * unscaledAMdot) + (0.6 * unscaledAMdot * F_iso));
        }



        // SHORTENING HEAT RATE for muscle i
        // --> depends on the normalized fiber length of the contractile element
        // --> note that we define Vm<0 as shortening and Vm>0 as lengthening
        // -----------------------------------------------------------------------
        if (isShorteningRateOn())
        {
			double Vmax_slowtwitch = max_shortening_velocity / (1 + 1.5*(1 - mm.getRatioSlowTwitchFibers()));
            double Vmax_fasttwitch = 2.5*Vmax_slowtwitch;
            double alpha_shortening_fasttwitch = 153 / Vmax_fasttwitch;
            double alpha_shortening_slowtwitch = 100 / Vmax_slowtwitch;
            double unscaledSdot, tmp_slowTwitch, tmp_fastTwitch;

            // Calculate UNSCALED heat rate --- muscle velocity dependent
            // -----------------------------------------------------------
            if (fiber_velocity_normalized <= 0)    // concentric contraction, Vm<0
            {
                const double maxShorteningRate = 100;

                tmp_slowTwitch = alpha_shortening_slowtwitch * fiber_velocity_normalized * mm.getRatioSlowTwitchFibers();
                if (tmp_slowTwitch > maxShorteningRate) {
                    cout << "WARNING: " << getName() << "  (t = " << s.getTime() << 
                        "Slow twitch shortening heat rate exceeds the max value of " << maxShorteningRate << 
                        " W/kg. Setting to " << maxShorteningRate << " W/kg." << endl; 
                    tmp_slowTwitch = maxShorteningRate;		// limit maximum value to 100 W.kg-1
                }

                tmp_fastTwitch = alpha_shortening_fasttwitch * fiber_velocity_normalized * (1-mm.getRatioSlowTwitchFibers());
                if (tmp_fastTwitch > maxShorteningRate) {
                    cout << "WARNING: " << getName() << "  (t = " << s.getTime() << 
                        "Fast twitch shortening heat rate exceeds the max value of " << maxShorteningRate << 
                        " W/kg. Setting to " << maxShorteningRate << " W/kg." << endl; 
                    tmp_fastTwitch = maxShorteningRate;		// limit maximum value to 100 W.kg-1
                }

                unscaledSdot = -tmp_slowTwitch - tmp_fastTwitch;                               // unscaled shortening heat rate: muscle shortening
            }

            else	// eccentric contraction, Vm>0
                unscaledSdot = -0.3 * alpha_shortening_slowtwitch * fiber_velocity_normalized;  // unscaled shortening heat rate: muscle lengthening


            // Calculate SCALED heat rate --- muscle velocity and length dependent
            // ---------------------------------------------------------------------
            if (fiber_velocity_normalized <= 0)     // concentric contraction, Vm<0
                Sdot = getScalingFactor() * std::pow(A, 2.0) * unscaledSdot;
            else                                    // eccentric contraction,  Vm>0
                Sdot = getScalingFactor() * A * unscaledSdot;

            if (fiber_length_normalized > 1.0)
                Sdot *= F_iso;  
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
        Edot(i) = AMdot + Sdot + Wdot;
		

        // DEBUG
        // ----------
        bool debug = false;
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
            cout << "slow_twitch_excitation = " << slow_twitch_excitation << endl;
            cout << "fast_twitch_excitation = " << fast_twitch_excitation << endl;
            cout << "max shortening velocity = " << max_shortening_velocity << endl;
            cout << "A_rel = " << A_rel << endl;
            cout << "B_rel = " << B_rel << endl;
            cout << "AMdot = " << AMdot << endl;
            cout << "Sdot = " << Sdot << endl;
            cout << "Bdot = " << Bdot << endl;
            cout << "Wdot = " << Wdot << endl;
            cout << "Edot = " << Edot(i) << endl;
            system("pause");
        }
    }

    SimTK::Vector EdotTotal(1, Edot.sum() + Bdot);

	if(EdotTotal(0) < 1.0 && isActivationMaintenanceRateOn() && isShorteningRateOn() && isWorkRateOn()) {
			cout << "WARNING: " << getName() << "  (t = " << s.getTime() << "), the model has a net metabolic energy rate of less than 1.0 W.kg-1." << endl; 
			EdotTotal(0) = 1.0;			// not allowed to fall below 1.0 W.kg-1
	}

    return EdotTotal;
}


//_____________________________________________________________________________
/** 
 * Returns the number of probe inputs in the vector returned by computeProbeInputs().
 */
int MuscleMetabolicPowerProbeUmberger2003::getNumProbeInputs() const
{
    return 1;
}


//_____________________________________________________________________________
/** 
 * Provide labels for the probe values being reported.
 */
Array<string> MuscleMetabolicPowerProbeUmberger2003::getProbeLabels() const 
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
Muscle* MuscleMetabolicPowerProbeUmberger2003::checkValidMetabolicMuscle(MetabolicMuscle mm) const
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