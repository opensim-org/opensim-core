#ifndef OPENSIM_METABOLIC_POWER_PROBE_UMBERGER2003_H_
#define OPENSIM_METABOLIC_POWER_PROBE_UMBERGER2003_H_
/* -------------------------------------------------------------------------- *
 *             OpenSim:  MuscleMetabolicPowerProbeUmberger2003.h              *
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

#include "Probe.h"
#include "Model.h"
#include "MetabolicMuscleParameterSet.h"


namespace OpenSim { 

//=============================================================================
//             MUSCLE METABOLIC POWER PROBE (Umberger, et al., 2003)
//=============================================================================
/**
 * MuscleMetabolicPowerProbeUmberger2003 is a ModelComponent Probe for computing the 
 * net metabolic energy rate of a set of Muscles in the model during a simulation. 
 * 
 * Based on the following papers:
 *
 * <a href="http://www.ncbi.nlm.nih.gov/pubmed/12745424">
 * Umberger, B. R., Gerritsen, K. G. and Martin, P. E. (2003). 
 * A model of human muscle energy expenditure. 
 * Comput Methods Biomech Biomed Engin 6, 99-111.</a>
 *
 * <a href="http://www.ncbi.nlm.nih.gov/pubmed/20356877">
 * Umberger, B. R. (2010). Stance and swing phase costs in human walking.
 * J R Soc Interface 7, 1329-40.</a>
 *
 * <I>Note that the equations below that describe the particular implementation of 
 * MuscleMetabolicPowerProbeUmberger2003 may slightly differ from the equations
 * described in the representative publications above. Note also that we define
 * positive muscle velocity to indicate lengthening (eccentric contraction) and
 * negative muscle velocity to indicate shortening (concentric contraction).</I>
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
 *
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
 * <H2><B> ACTIVATION & MAINTENANCE HEAT RATE </B></H2>
 * If <I>activation_maintenance_rate_on</I> is set to true, then Adot+Mdot is calculated as follows:\n
 * <B>Adot+Mdot = [128*(1-r) + 25] * A^0.6 * S                        </B>,  <I> l_CE <= l_CE_opt </I>\n 
 * <B>Adot+Mdot = 0.4*[128*(1-r) + 25] + 0.6*F_iso*[128*(1-r) + 25]   </B>,  <I> l_CE >  l_CE_opt </I>
 *     - <B>A = u          </B>,    u >  a
 *     - <B>A = (u+a)/2    </B>,    u <= a
 *
 *     - l_CE = muscle fiber length at the current time.
 *     - l_CE_opt = optimal fiber length of the muscle.
 *     - F_CE_iso = force that would be developed by the contractile element of muscle under isometric conditions with the current activation and fiber length.
 *     - u = muscle excitation at the current time.
 *     - a = muscle activation at the current time.
 *     - S = aerobic/anaerobic scaling factor, defined by the 'scaling_factor' property..
 *
 *
 * <H2><B> SHORTENING HEAT RATE </B></H2>
 * If <I>shortening_rate_on</I> is set to true, then Sdot is calculated as follows:\n
 * <B>Sdot = -[(alphaS_slow * v_CE_norm * r) + (alphaS_fast * v_CE_norm * (1-r))] * A^2 * S           </B>,   <I>l_CE <= l_CE_opt   &   v_CE >= 0 (concentric / isometric contraction)</I>\n
 * <B>Sdot = -[(alphaS_slow * v_CE_norm * r) + (alphaS_fast * v_CE_norm * (1-r))] * A^2 * S * F_iso   </B>,   <I>l_CE >  l_CE_opt   &   v_CE >= 0 (concentric / isometric contraction)</I>\n
 * <B>Sdot = -alphaL * v_CE_norm * A * S           </B>,   <I>l_CE <= l_CE_opt   &   v_CE <  0 (eccentric contraction)</I>\n
 * <B>Sdot = -alphaL * v_CE_norm * A * S * F_iso   </B>,   <I>l_CE >  l_CE_opt   &   v_CE <  0 (eccentric contraction)</I>
 * 
 *     - <B>A = u          </B>,    <I>u >  a </I>
 *     - <B>A = (u+a)/2    </B>,    <I>u <= a </I>
 *
 *     - <B>alphaS_slow = 100 /     ( v_CE_max / (1 + (1.5*r)) ) </B>
 *     - <B>alphaS_fast = 153 / 2.5*( v_CE_max / (1 + (1.5*r)) ) </B>
 *     - <B>alphaL = 0.3 * alphaS_slow </B>
 *
 *     - l_CE = muscle fiber length at the current time.
 *     - l_CE_opt = optimal fiber length of the muscle.
 *     - F_CE = force developed by the contractile element of muscle at the current time.
 *     - F_CE_iso = force that would be developed by the contractile element of muscle under isometric conditions with the current activation and fiber length.
 *     - v_CE = muscle fiber velocity at the current time.
 *     - v_CE_max = maximum shortening velocity of the muscle.
 *     - v_CE_norm = normalized muscle fiber velocity = v_CE/v_CE_max.
 *     - S = aerobic/anaerobic scaling factor, defined by the 'scaling_factor' property.
 *
 *
 * <H2><B> MECHANICAL WORK RATE </B></H2>
 * If <I>mechanical_work_rate_on</I> is set to true, then Wdot is calculated as follows:\n
 * <B>Wdot = -F_CE * v_CE       </B>,   <I>v_CE >= 0 (concentric / isometric contraction)</I>\n
 * <B>Wdot = 0                  </B>,   <I>v_CE <  0 (eccentric contraction)</I>
 *     - v_CE = muscle fiber velocity at the current time.
 *     - F_CE = force developed by the contractile element of muscle at the current time.\n
 * <I> Note: if normalize_mechanical_work_rate_by_muscle_mass ia set to true, then the mechanical work rate
 *       for each muscle is normalized by its muscle mass (kg).</I>
 *
 *
 * @author Tim Dorn
 */

class OSIMSIMULATION_API MuscleMetabolicPowerProbeUmberger2003 : public Probe {
OpenSim_DECLARE_CONCRETE_OBJECT(MuscleMetabolicPowerProbeUmberger2003, Probe);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
    /** Enabled by default. **/
    OpenSim_DECLARE_PROPERTY(activation_maintenance_rate_on, 
        bool,
        "Specify whether the activation & maintenance heat rate is to be calculated (true/false).");

    /** Enabled by default. **/
    OpenSim_DECLARE_PROPERTY(shortening_rate_on, 
        bool,
        "Specify whether the shortening heat rate is to be calculated (true/false).");

    /** Enabled by default. **/
    OpenSim_DECLARE_PROPERTY(basal_rate_on, 
        bool,
        "Specify whether the basal heat rate is to be calculated (true/false).");

    /** Enabled by default. **/
    OpenSim_DECLARE_PROPERTY(mechanical_work_rate_on, 
        bool,
        "Specify whether the mechanical work rate is to be calculated (true/false).");

    /** Default value = 1.0. **/
    OpenSim_DECLARE_PROPERTY(scaling_factor, 
        double,
        "Scaling factor (S=1.0 for primarily anaerobic conditions and S=1.5 for primarily aerobic conditions. See Umberger et al., (2002).");

    /** Default value = 1.51. **/
    OpenSim_DECLARE_PROPERTY(basal_coefficient, 
        double,
        "Basal metabolic coefficient.");

    /** Default value = 1.0. **/
    OpenSim_DECLARE_PROPERTY(basal_exponent, 
        double,
        "Basal metabolic exponent.");

    /** Disabled by default. **/
    OpenSim_DECLARE_PROPERTY(normalize_mechanical_work_rate_by_muscle_mass, 
        bool,
        "Specify whether the mechanical work rate for each muscle is to be normalized by muscle mass (true/false).");

    OpenSim_DECLARE_UNNAMED_PROPERTY(MetabolicMuscleParameterSet,
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
    MuscleMetabolicPowerProbeUmberger2003();
    /** Convenience constructor */
    MuscleMetabolicPowerProbeUmberger2003(bool activation_maintenance_rate_on, 
        bool shortening_rate_on, bool basal_rate_on, bool work_rate_on);


    //-----------------------------------------------------------------------------
    // Computation
    //-----------------------------------------------------------------------------
    /** Compute muscle metabolic power. */
    virtual SimTK::Vector computeProbeInputs(const SimTK::State& state) const OVERRIDE_11;

    /** Returns the number of probe inputs in the vector returned by computeProbeInputs(). */
    int getNumProbeInputs() const OVERRIDE_11;

    /** Returns the column labels of the probe values for reporting. 
        Currently uses the Probe name as the column label, so be sure
        to name your probe appropiately!  */
    virtual OpenSim::Array<std::string> getProbeOutputLabels() const OVERRIDE_11;

    /** Check that the MetabolicMuscleParameter represents a valid muscle in the model. */
    Muscle* checkValidMetabolicMuscle(MetabolicMuscleParameter mm) const;


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
};	// END of class MuscleMetabolicPowerProbeUmberger2003

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef OPENSIM_METABOLIC_POWER_PROBE_UMBERGER2003_H_
