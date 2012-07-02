#ifndef OPENSIM_METABOLIC_POWER_PROBE_BHARGAVA2004_H_
#define OPENSIM_METABOLIC_POWER_PROBE_BHARGAVA2004_H_

// MuscleMetabolicPowerProbeBhargava2004.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Tim Dorn
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2012, Stanford University. All rights reserved. 
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

#include "Probe.h"
#include "Model.h"
#include "MetabolicMuscleSet.h"
#include <OpenSim/Common/PiecewiseLinearFunction.h>

namespace OpenSim { 

//=============================================================================
//             MUSCLE METABOLIC POWER PROBE (Bhargava, et al., 2004)
//=============================================================================
/**
 * MuscleMetabolicPowerProbeBhargava2004 is a ModelComponent Probe for computing the 
 * metabolic energy rate of a set of Muscles in the model during a simulation. 
 * 
 * Based on the following paper:
 *
 * <a href="http://www.ncbi.nlm.nih.gov/pubmed/14672571">
 * Bhargava, L. J., Pandy, M. G. and Anderson, F. C. (2004). 
 * A phenomenological model for estimating metabolic energy consumption
 * in muscle contraction. J Biomech 37, 81-8.</a>
 *
 * <I>Note that the equations below that describe the particular implementation of 
 * MuscleMetabolicPowerProbeBhargava2004 may slightly differ from the equations
 * described in the representative publication above.</I>
 *
 *
 * Muscle metabolic power (or rate of metabolic energy consumption) is equal to the
 * rate at which heat is liberated plus the rate at which work is done:\n
 * <B>Edot = Bdot + sumOfAllMuscles(Adot + Mdot + Sdot + Wdot).</B>
 *
 *       - Bdot is the basal heat rate.
 *       - Adot is the activation heat rate.
 *       - Mdot is the maintenance heat rate.
 *       - Sdot is the shortening heat rate.
 *       - Wdot is the mechanical work rate.
 *
 *
 * This probe also uses muscle parameters stored in the MetabolicMuscle object for each muscle.
 * The full set of all MetabolicMuscles (MetabolicMuscleSet) is a property of this probe:
 * 
 * - m = The mass of the muscle (kg).
 * - r = Ratio of slow twitch fibers in the muscle (must be between 0 and 1).
 * - Adot_slow = Activation constant for slow twitch fibers (W/kg).
 * - Adot_fast = Activation constant for fast twitch fibers (W/kg).
 * - Mdot_slow = Maintenance constant for slow twitch fibers (W/kg).
 * - Mdot_fast = Maintenance constant for slow twitch fibers (W/kg).
 *
 *
 * <H2><B> BASAL HEAT RATE </B></H2>
 * If <I>basal_rate_on</I> is set to true, then Bdot is calculated as follows:\n
 * <B>Bdot = basal_coefficient * (m_body^basal_exponent)</B>
 *     - m_body = mass of the entire model
 *     - basal_coefficient and basal_exponent are defined by their respective properties.\n
 * <I>Note that this quantity is muscle independant. Rather it is calculated on a whole body level.</I>
 *
 *
 * <H2><B> ACTIVATION HEAT RATE </B></H2>
 * If <I>activation_rate_on</I> is set to true, then Adot is calculated as follows:\n
 * <B>Adot = m * [ Adot_slow * r * sin((pi/2)*u)    +    Adot_fast * (1-r) * (1-cos((pi/2)*u)) ]</B>
 *     - u = muscle excitation at the current time.
 *
 *
 * <H2><B> MAINTENANCE HEAT RATE </B></H2>
 * If <I>maintenance_rate_on</I> is set to true, then Mdot is calculated as follows:\n
 * <B>Mdot = m * f * [ Mdot_slow * r * sin((pi/2)*u)    +    Mdot_fast * (1-r) * (1-cos((pi/2)*u)) ]</B>
 * - u = muscle excitation at the current time.
 * - f is a piecewise linear function that describes the normalized fiber length denendence
 * of the maintenance heat rate (default curve is shown below):
 * \image html fig_NormalizedFiberLengthDependenceOfMaintenanceHeatRateBhargava2004.png
 *
 *
 * <H2><B> SHORTENING HEAT RATE </B></H2>
 * If <I>shortening_rate_on</I> is set to true, then Sdot is calculated as follows:\n
 * <B>Sdot = alpha * v_CE</B>
 *
 * If use_force_dependent_shortening_prop_constant = true,
 *     - <B>alpha = (0.16 * F_CE_iso) + (0.18 * F_CE)   </B>,   <I>v_CE >= 0 (concentric / isometric contraction)</I>
 *     - <B>alpha = 0.157 * F_CE                        </B>,   <I>v_CE <  0 (eccentric contraction)</I>
 * 
 *     - v_CE = muscle fiber velocity at the current time.
 *     - F_CE = force developed by the contractile element of muscle at the current time.
 *     - F_CE_iso = force that would be developed by the contractile element of muscle under isometric conditions with the current activation and fiber length.
 *
 * If use_force_dependent_shortening_prop_constant = false,
 *     - <B>alpha = 0.25,   </B>,   <I>v_CE >= 0 (concentric / isometric contraction)</I>
 *     - <B>alpha = 0.00    </B>,   <I>v_CE <  0 (eccentric contraction)</I>
 *
 *
 * <H2><B> MECHANICAL WORK RATE </B></H2>
 * If <I>mechanical_work_rate_on</I> is set to true, then Wdot is calculated as follows:\n
 * <B>Wdot = F_CE * v_CE       </B>,   <I>v_CE >= 0 (concentric / isometric contraction)</I>\n
 * <B>Wdot = 0                 </B>,   <I>v_CE <  0 (eccentric contraction)</I>
 *     - v_CE = muscle fiber velocity at the current time.
 *     - F_CE = force developed by the contractile element of muscle at the current time.\n
 * <I> Note: if normalize_mechanical_work_rate_by_muscle_mass ia set to true, then the mechanical work rate
 *       for each muscle is normalized by its muscle mass (kg).</I>
 *
 *
 * @author Tim Dorn
 */

class OSIMSIMULATION_API MuscleMetabolicPowerProbeBhargava2004 : public Probe {
OpenSim_DECLARE_CONCRETE_OBJECT(MuscleMetabolicPowerProbeBhargava2004, Probe);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
    /** Enabled by default. **/
    OpenSim_DECLARE_PROPERTY(activation_rate_on, bool,
        "Specify whether the activation heat rate is to be calculated (true/false).");

    /** Enabled by default. **/
    OpenSim_DECLARE_PROPERTY(maintenance_rate_on, bool,
        "Specify whether the maintenance heat rate is to be calculated (true/false).");

    /** Enabled by default. **/
    OpenSim_DECLARE_PROPERTY(shortening_rate_on, bool,
        "Specify whether the shortening heat rate is to be calculated (true/false).");

    /** Enabled by default. **/
    OpenSim_DECLARE_PROPERTY(basal_rate_on, bool,
        "Specify whether the basal heat rate is to be calculated (true/false).");

    /** Enabled by default. **/
    OpenSim_DECLARE_PROPERTY(mechanical_work_rate_on, bool,
        "Specify whether the mechanical work rate is to be calculated (true/false).");

    /** Default curve shown in doxygen. **/
    OpenSim_DECLARE_PROPERTY(normalized_fiber_length_dependence_on_maintenance_rate, PiecewiseLinearFunction,
        "Contains a PiecewiseLinearFunction object that describes the normalized fiber length dependence on maintenance rate.");

    /** Disabled by default. **/
    OpenSim_DECLARE_PROPERTY(use_force_dependent_shortening_prop_constant, bool,
        "Specify whether to use a force dependent shortening proportionality constant (true/false).");

    /** Default value = 1.51. **/
    OpenSim_DECLARE_PROPERTY(basal_coefficient, double,
        "Basal metabolic coefficient.");

    /** Default value = 1.0. **/
    OpenSim_DECLARE_PROPERTY(basal_exponent, double,
        "Basal metabolic exponent.");

    /** Disabled by default. **/
    OpenSim_DECLARE_PROPERTY(normalize_mechanical_work_rate_by_muscle_mass, bool,
        "Specify whether the mechanical work rate for each muscle is to be normalized by muscle mass (true/false).");

    OpenSim_DECLARE_UNNAMED_PROPERTY(MetabolicMuscleSet,
        "A MetabolicMuscleSet containing the muscle information required to calculate metabolic energy expenditure. "
        "If multiple muscles are contained in the set, then the probe value will equal the summation of all metabolic powers.");

    /**@}**/

//=============================================================================
// PUBLIC METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // Constructor(s) and Setup
    //--------------------------------------------------------------------------
    /** Default constructor */
    MuscleMetabolicPowerProbeBhargava2004();
    /** Convenience constructor */
    MuscleMetabolicPowerProbeBhargava2004(bool activation_rate_on, bool maintenance_rate_on, 
        bool shortening_rate_on, bool basal_rate_on, bool work_rate_on);
        

    //--------------------------------------------------------------------------
    // Get and Set
    //--------------------------------------------------------------------------
    /** Returns if the activation metabolic rate is to be included in the calculation (true/false). */
    bool isActivationRateOn() const;
    /** Returns if the maintenance metabolic rate is to be included in the calculation (true/false). */
    bool isMaintenanceRateOn() const;
    /** Returns if the shortening metabolic rate is to be included in the calculation (true/false). */
    bool isShorteningRateOn() const;
    /** Returns if the basal metabolic rate is to be included in the calculation (true/false). */
    bool isBasalRateOn() const;
    /** Returns if the mechanical work rate is to be included in the calculation (true/false). */
    bool isWorkRateOn() const;
    /** Returns the PiecewiseLinearFunction used in calculating the fiber length dependence of the maintenance metabolic rate. */
    PiecewiseLinearFunction getFiberLengthDependenceMaintenanceRateFunction() const;
    /** Returns true if the shortening metabolic rate is a function of fiber force, false if it is not. */
    bool usingForceDepShorteningPropConstant() const;
    /** Returns the basal metabolic rate coefficient (W/kg). */
    double getBasalCoefficient() const;
    /** Returns the basal metabolic rate exponent. */
    double getBasalExponent() const;
    /** Returns if the mechanical work rate is to be normalized by muscle mass (true/false). */
    bool isMechanicalWorkRateNormalizedToMuscleMass() const;
    /** Returns a const MetabolicMuscleSet containing the Muscle(s) being probed. */
    const MetabolicMuscleSet& getMetabolicMuscleSet() const;
    /** Returns an updatable MetabolicMuscleSet containing the Muscle(s) being probed. */
    MetabolicMuscleSet& updMetabolicMuscleSet();

    /** Sets if the activation metabolic rate is to be included in the calculation (true/false). */
    void setActivationRateOn(const bool aActRateOn);
    /** Sets if the shortening metabolic rate is to be included in the calculation (true/false). */
    void setMaintenanceRateOn(const bool aMainRateOn);
    /** Sets if the shortening metabolic rate is to be included in the calculation (true/false). */
    void setShorteningRateOn(const bool aShortRateOn);
    /** Sets if the basal metabolic rate is to be included in the calculation (true/false). */
    void setBasalRateOn(const bool aBasalRateOn);
    /** Sets if the mechanical work rate is to be included in the calculation (true/false). */
    void setWorkRateOn(const bool aWorkRateOn);
    /** Sets the PiecewiseLinearFunction used in calculating the fiber length dependence of the maintenance metabolic rate. */
    void setFiberLengthDependenceMaintenanceRateFunction(const PiecewiseLinearFunction aFunct);
    /** Sets true if the shortening metabolic rate is a function of fiber force, false if it is not. */
    void setUsingForceDepShorteningPropConstant(const bool aUseForceDepShortPropConst);
    /** Sets the basal metabolic rate coefficient. */
    void setBasalCoefficient(const double aBasalCoeff);
    /** Sets the basal metabolic rate exponent. */
    void setBasalExponent(const double aBasalExp);
    /** Sets if the mechanical work rate is to be normalized by muscle mass (true/false). */
    void setMechanicalWorkRateNormalizedToMuscleMass(const bool normalizeWorkRate);
    /** Sets the MetabolicMuscleSet containing the Muscle(s) being probed. */
    void setMetabolicMuscleSet(const MetabolicMuscleSet mms);


    //-----------------------------------------------------------------------------
    // Computation
    //-----------------------------------------------------------------------------
    /** Compute muscle metabolic power. */
    virtual double computeProbeValue(const SimTK::State& state) const OVERRIDE_11;

    /** Check that the MetabolicMuscle represents a valid muscle in the model. */
    Muscle* checkValidMetabolicMuscle(MetabolicMuscle mm) const;


//==============================================================================
// PRIVATE
//==============================================================================
private:
    //--------------------------------------------------------------------------
    // ModelComponent Interface
    //--------------------------------------------------------------------------
    void connectToModel(Model& aModel) OVERRIDE_11;

    void setNull();
    void constructProperties();


//=============================================================================
};	// END of class MuscleMetabolicPowerProbeBhargava2004

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef OPENSIM_METABOLIC_POWER_PROBE_BHARGAVA2004_H_
