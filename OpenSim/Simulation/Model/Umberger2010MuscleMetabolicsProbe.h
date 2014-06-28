#ifndef OPENSIM_UMBERGER2010_METABOLIC_POWER_PROBE_H_
#define OPENSIM_UMBERGER2010_METABOLIC_POWER_PROBE_H_
/* -------------------------------------------------------------------------- *
 *               OpenSim:  Umberger2010MuscleMetabolicsProbe.h                *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

#include "Probe.h"
#include "Model.h"



namespace OpenSim { 

// Helper classes defined below.
class Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter;
class Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameterSet;

//=============================================================================
//             MUSCLE METABOLIC POWER PROBE (Umberger, et al., 2010)
//=============================================================================

/**
 * %Umberger2010MuscleMetabolicsProbe is a Probe ModelComponent for computing 
 * the net metabolic energy rate of a set of Muscles in the model during a 
 * simulation. 
 * 
 * <h1>%Umberger2010MuscleMetabolicsProbe Theory</h1>
 *
 * The discussion here is based on the following papers:
 *
 * <a href="http://www.ncbi.nlm.nih.gov/pubmed/20356877">
 * Umberger, B. R. (2010). Stance and swing phase costs in human walking.
 * J R Soc Interface 7, 1329-40.</a>
 *
 * <a href="http://www.ncbi.nlm.nih.gov/pubmed/12745424">
 * Umberger, B. R., Gerritsen, K. G. and Martin, P. E. (2003). 
 * A model of human muscle energy expenditure. 
 * Comput Methods Biomech Biomed Engin 6, 99-111.</a>
 *
 * <I>Note that the equations below that describe the particular implementation 
 * of %Umberger2010MuscleMetabolicsProbe may slightly differ from the equations
 * described in the representative publications above. Note also that we define
 * positive muscle velocity to indicate lengthening (eccentric contraction) and
 * negative muscle velocity to indicate shortening (concentric contraction).</I>
 *
 *
 * %Muscle metabolic power (or rate of metabolic energy consumption) is equal to
 * the rate at which heat is liberated plus the rate at which work is done:\n
 * <B>Edot = Bdot + sumOfAllMuscles(Adot + Mdot + Sdot + Wdot).</B>
 *
 *       - Bdot is the basal heat rate (W).
 *       - Adot is the activation heat rate (W).
 *       - Mdot is the maintenance heat rate (W).
 *       - Sdot is the shortening heat rate (W).
 *       - Wdot is the mechanical work rate (W).
 *
 *
 * This probe also uses muscle parameters stored in the MetabolicMuscle object for each muscle.
 * The full set of all MetabolicMuscles (MetabolicMuscleSet) is a property of this probe:
 * 
 * - m = The mass of the muscle (kg).
 * - r = Ratio of slow-twitch fibers in the muscle (between 0 and 1).
 *
 * The recruitment model described by Bhargava et al. (2004) is used to set the
 * slow-twitch fiber ratio used in the calculations below. The ratio specified
 * by the user indicates the composition of the muscle; this value is used only
 * at full excitation (i.e., when all fibers are recruited). As excitation
 * decreases from 1 to 0, the proportion of recruited fibers that are
 * slow-twitch fibers increases from r to 1. See
 * <a href="http://www.ncbi.nlm.nih.gov/pubmed/14672571">Bhargava, L.J., Pandy,
 * M.G., Anderson, F.C. (2004) A phenomenological model for estimating metabolic
 * energy consumption in muscle contraction. J Biomech 37:81-88</a>. To assume a
 * constant ratio of slow- and fast-twitch fiber recruitment, set the
 * 'use_Bhargava_recruitment_model' property to false.
 *
 *
 *
 * <H2><B> BASAL HEAT RATE (W) </B></H2>
 * If <I>basal_rate_on</I> is set to true, then Bdot is calculated as follows:\n
 * <B>Bdot = basal_coefficient * (m_body^basal_exponent) </B>
 *     - m_body = mass of the entire model
 *     - basal_coefficient and basal_exponent are defined by their respective properties.\n
 * <I>Note that this quantity is muscle independant. Rather it is calculated on a whole body level.</I>
 *
 *
 * <H2><B> ACTIVATION & MAINTENANCE HEAT RATE (W) </B></H2>
 * If <I>activation_maintenance_rate_on</I> is set to true, then Adot+Mdot is calculated as follows:\n
 * <B>Adot+Mdot = [128*(1-r) + 25] * A^0.6 * S                                         </B>,  <I> l_CE <= l_CE_opt </I>\n 
 * <B>Adot+Mdot = (0.4*[128*(1-r) + 25] + 0.6*[128*(1-r) + 25]*F_CE_iso) * A^0.6 * S   </B>,  <I> l_CE >  l_CE_opt </I>
 *     - <B>A = u          </B>,    u >  a
 *     - <B>A = (u+a)/2    </B>,    u <= a
 *
 *     - m = The mass of the muscle (kg).
 *     - l_CE = muscle fiber length at the current time.
 *     - l_CE_opt = optimal fiber length of the muscle.
 *     - F_CE_iso = normalized contractile element force-length curve.
 *     - u = muscle excitation at the current time.
 *     - a = muscle activation at the current time.
 *     - S = aerobic/anaerobic scaling factor, defined by the 'aerobic_factor' property (i.e. usually 1.0 for primarily anaerobic activities, 1.5 for primarily aerobic activities).
 *
 *
 * <H2><B> SHORTENING HEAT RATE (W) </B></H2>
 * If <I>shortening_rate_on</I> is set to true, then Sdot is calculated as follows:\n
 * <B>Sdot = m * (-[(alphaS_slow * v_CE_norm * r) + (alphaS_fast * v_CE_norm * (1-r))] * A^2 * S)           </B>,   <I>l_CE <= l_CE_opt   &   v_CE >= 0 (concentric / isometric contraction)</I>\n
 * <B>Sdot = m * (-[(alphaS_slow * v_CE_norm * r) + (alphaS_fast * v_CE_norm * (1-r))] * A^2 * S * F_iso)   </B>,   <I>l_CE >  l_CE_opt   &   v_CE >= 0 (concentric / isometric contraction)</I>\n
 * <B>Sdot = m * (alphaL * v_CE_norm * A * S)              </B>,   <I>l_CE <= l_CE_opt   &   v_CE <  0 (eccentric contraction)</I>\n
 * <B>Sdot = m * (alphaL * v_CE_norm * A * S * F_CE_iso)   </B>,   <I>l_CE >  l_CE_opt   &   v_CE <  0 (eccentric contraction)</I>
 * 
 *     - <B>A = u          </B>,    <I>u >  a </I>
 *     - <B>A = (u+a)/2    </B>,    <I>u <= a </I>
 *
 *     - <B>alphaS_fast = 153 / v_CE_max          </B>
 *     - <B>alphaS_slow = 100 / (v_CE_max / 2.5)  </B>
 *     - <B>alphaL = 4.0 * alphaS_slow </B>
 *
 *     - m = The mass of the muscle (kg).
 *     - l_CE = muscle fiber length at the current time.
 *     - l_CE_opt = optimal fiber length of the muscle.
 *     - F_CE_iso = force that would be developed by the contractile element of muscle under isometric conditions with the current activation and fiber length.
 *     - v_CE = muscle fiber velocity at the current time.
 *     - v_CE_max = maximum shortening velocity of the muscle.
 *     - v_CE_norm = normalized muscle fiber velocity (defined for this model as v_CE/l_CE_opt).
 *               Note that this is a different metric to the typical normalized_muscle_fiber_velocity of v_CE/v_CE_max.
 *     - S = aerobic/anaerobic scaling factor, defined by the 'aerobic_factor' property (i.e. usually 1.0 for primarily anaerobic activities, 1.5 for primarily aerobic activities).
 *
 *
 * <H2><B> MECHANICAL WORK RATE (W) </B></H2>
 * If <I>mechanical_work_rate_on</I> is set to true, then Wdot is calculated as follows:\n
 * <B>Wdot = -(F_CE * v_CE)           </B>
 *     - v_CE = muscle fiber velocity at the current time.
 *     - F_CE = force developed by the contractile element of muscle at the current time.\n
 *
 * If we draw a control volume around the fiber, the first law of thermodynamics
 * suggests that negative mechanical work should be included in Wdot. As such,
 * we revert back to the model described in Umberger et al. (2003) by default.
 * To exclude negative mechanical work from Wdot and use a coefficient of 0.3
 * (rather than 4.0) to calculate alpha_L, set the
 * 'include_negative_mechanical_work' property to false.
 *
 * During eccentric contraction, the magnitude of the (negative) mechanical work
 * rate can exceed that of the total (positive) heat rate, resulting in a flow
 * of energy into the fiber. Experiments indicate that the chemical processes
 * involved in fiber contraction cannot be reversed, and most of the energy that
 * is absorbed during eccentric contraction (in increased cross-bridge
 * potentials, for example) is eventually converted into heat. Thus, we increase
 * Sdot (if necessary) to ensure Edot > 0 for each muscle. See
 * <a href="http://www.ncbi.nlm.nih.gov/pubmed/9409483">Constable, J.K.,
 * Barclay, C.J., Gibbs, C.L. (1997) Energetics of lengthening in mouse and toad
 * skeletal muscles. J Physiol 505:205-215</a>. To allow muscles to have
 * negative total power, set the 'forbid_negative_total_power' property to false.
 *
 *
 * Note that if enforce_minimum_heat_rate_per_muscle == true AND 
 * activation_maintenance_rate_on == shortening_rate_on == true, then the total heat
 * rate (AMdot + Sdot) will be capped to a minimum value of 1.0 W/kg (Umberger(2003), page 104).
 *
 *
 *
 *
 * <H1>Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter</H1>
 *
 * Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter is an Object class that 
 * holds the metabolic parameters required to calculate metabolic power for a single muscle.
 *
 * <H2><B> Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter Properties </B></H2>
 *
 * REQUIRED PROPERTIES
 * - <B>specific_tension</B> = The specific tension of the muscle (Pascals (N/m^2)).
 * - <B>density</B> = The density of the muscle (kg/m^3).
 * - <B>ratio_slow_twitch_fibers</B> = Ratio of slow twitch fibers in the muscle (must be between 0 and 1).
 *
 * OPTIONAL PROPERTIES
 * - <B>use_provided_muscle_mass</B> = An optional flag that allows the user to
 *      explicitly specify a muscle mass. If set to true, the 'provided_muscle_mass'
 *      property must be specified. The default setting is false, in which case, the
 *      muscle mass is calculated from the following formula:
 *          m = (Fmax/specific_tension)*density*Lm_opt, where 
 *              specific_tension and density are properties defined above
 *                  (note that their default values are set based on mammalian muscle,
 *                  0.25e6 N/m^2 and 1059.7 kg/m^3, respectively);
 *              Fmax and Lm_opt are the maximum isometric force and optimal 
 *                  fiber length, respectively, of the muscle.
 *
 * - <B>provided_muscle_mass</B> = The user specified muscle mass (kg).
 *
 *
 * @author Tim Dorn
 */

class OSIMSIMULATION_API Umberger2010MuscleMetabolicsProbe : public Probe {
OpenSim_DECLARE_CONCRETE_OBJECT(Umberger2010MuscleMetabolicsProbe, Probe);
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
        "Specify whether activation & maintenance heat rate is to be calculated (true/false).");

    /** Enabled by default. **/
    OpenSim_DECLARE_PROPERTY(shortening_rate_on, 
        bool,
        "Specify whether shortening heat rate is to be calculated (true/false).");

    /** Enabled by default. **/
    OpenSim_DECLARE_PROPERTY(basal_rate_on, 
        bool,
        "Specify whether basal heat rate is to be calculated (true/false).");

    /** Enabled by default. **/
    OpenSim_DECLARE_PROPERTY(mechanical_work_rate_on, 
        bool,
        "Specify whether mechanical work rate is to be calculated (true/false).");

    /** Enabled by default. **/
    OpenSim_DECLARE_PROPERTY(enforce_minimum_heat_rate_per_muscle, 
        bool,
        "Specify whether the total heat rate for a muscle will be clamped to a "
        "minimum value of 1.0 W/kg (true/false).");

    /** Default value = 1.5. **/
    OpenSim_DECLARE_PROPERTY(aerobic_factor, 
        double,
        "Aerobic scale factor (S=1.0 for primarily anaerobic conditions and S=1.5 "
        "for primarily aerobic conditions. See Umberger et al., (2003).");

    /** Default value = 1.2. **/
    OpenSim_DECLARE_PROPERTY(basal_coefficient, 
        double,
        "Basal metabolic coefficient.");

    /** Default value = 1.0. **/
    OpenSim_DECLARE_PROPERTY(basal_exponent, 
        double,
        "Basal metabolic exponent.");

    /** Default value = 1.0. **/
    OpenSim_DECLARE_PROPERTY(muscle_effort_scaling_factor,
        double,
        "Scale the excitation and activation values used by the probe to "
        "compensate for solutions with excessive coactivation (e.g., when a "
        "suboptimal tracking strategy is used).");

    /** Enabled by default. **/
    OpenSim_DECLARE_PROPERTY(use_Bhargava_recruitment_model,
        bool,
        "Specify whether the recruitment model described by Bhargava et al. "
        "(2004) will used to determine the slow-twitch fiber ratio "
        "(true/false). Disable to use the model as published in Umberger "
        "(2010).");

    /** Enabled by default. **/
    OpenSim_DECLARE_PROPERTY(include_negative_mechanical_work,
        bool,
        "Specify whether negative mechanical work will be included in Wdot and "
        "a coefficient of 4.0 will be used to calculate alpha_L (true/false). "
        "Disable to use the model as published in Umberger (2010).");

    /** Enabled by default. **/
    OpenSim_DECLARE_PROPERTY(forbid_negative_total_power,
        bool,
        "Specify whether the total power for each muscle must remain positive "
        "(true/false). Disable to use the model as published in Umberger "
        "(2010).");

    /** Default value = true **/
    OpenSim_DECLARE_PROPERTY(report_total_metabolics_only, 
        bool,
        "If set to false, the individual muscle metabolics, basal rate, and "
        "total summation will be reported. If set to true, only the total "
        "summation will be reported.");

    OpenSim_DECLARE_UNNAMED_PROPERTY(
        Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameterSet,
        "A set containing, for each muscle, the parameters "
        "required to calculate muscle metabolic power.");

    /**@}**/

//=============================================================================
// PUBLIC METHODS
//=============================================================================
    /** MuscleMap typedef */
    typedef std::map
       <std::string, 
        Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter*> 
        MuscleMap;

    //--------------------------------------------------------------------------
    // Constructor(s) and Setup
    //--------------------------------------------------------------------------
    /** Default constructor */
    Umberger2010MuscleMetabolicsProbe();

    /** Convenience constructor */
    Umberger2010MuscleMetabolicsProbe(
        const bool activation_maintenance_rate_on, 
        const bool shortening_rate_on, 
        const bool basal_rate_on, 
        const bool work_rate_on);


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


    //-----------------------------------------------------------------------------
    /** @name     Umberger2010MuscleMetabolicsProbe Interface
    These accessor methods are to be used when setting up a new muscle 
    metabolic analysis from the API. The basic operation is as follows:
    @code
    Umberger2010MuscleMetabolicsProbe* myProbe new Umberger2010MuscleMetabolicsProbe(...); 
    model.addProbe(myProbe);
    myProbe->addMuscle("muscleName1", ... );
    myProbe->addMuscle("muscleName2", ... );
    myProbe->addMuscle("muscleName3", ... );
    myProbe->useProvidedMass("muscleName1", 1.2);         // muscle1 mass = 1.2 kg
    myProbe->useCalculatedMass("muscleName2");            // muscle2 mass is based on muscle properties (below)
    myProbe->setDensity("muscleName2", 1100.0);           // muscle2 density is 1100 kg/m^3
    myProbe->setSpecificTension("muscleName2", 0.26e6);   // muscle2 specific tension is 0.26e6 Pa
    myProbe->removeMuscle("muscleName3");
    myProbe->setOperation("integrate")           // See OpenSim::Probe for other operations
    @endcode
    @note It is important to first add the metabolic probe to the model before
    calling any other methods that may modify its properties. This is because 
    some methods (e.g. addMuscle() or useCalculatedMass) may require information
    about the muscles to sucsessfully execute, and this information can only be
    obtained if the metabolic probe is already 'connected' to the model.
    */
    /** Get the number of muscles being analysed in the metabolic analysis. */
    const int getNumMetabolicMuscles() const;  

    /** Add a muscle and its parameters so that it can be included in the metabolic analysis. */
    void addMuscle(const std::string& muscleName, 
        double ratio_slow_twitch_fibers);

    /** Add a muscle and its parameters so that it can be included in the metabolic analysis. */
    void addMuscle(const std::string& muscleName, 
        double ratio_slow_twitch_fibers, 
        double muscle_mass);

    /** Remove a muscle from the metabolic analysis. */
    void removeMuscle(const std::string& muscleName);

    /** Set an existing muscle to use a provided muscle mass. */
    void useProvidedMass(const std::string& muscleName, double providedMass);
    
    /** Set an existing muscle to calculate its own mass. */
    void useCalculatedMass(const std::string& muscleName);

    /** Get whether the muscle mass is being explicitly provided.
        True means that it is using the property 'provided_muscle_mass'
        False means that the muscle mass is being calculated from muscle properties. */
    bool isUsingProvidedMass(const std::string& muscleName);

    /** Get the muscle mass used in the metabolic analysis. The value
        returned will depend on if the muscle mass is explicitly provided
        (i.e. isUsingProvidedMass = true), or if it is being automatically
        calculated from muscle data already present in the model
        (i.e. isUsingProvidedMass = true). */
    const double getMuscleMass(const std::string& muscleName) const;

    /** Get the ratio of slow twitch fibers for an existing muscle. */
    const double getRatioSlowTwitchFibers(const std::string& muscleName) const;

    /** Set the ratio of slow twitch fibers for an existing muscle. */
    void setRatioSlowTwitchFibers(const std::string& muscleName, const double& ratio);

    /** Get the density for an existing muscle (kg/m^3). */
    const double getDensity(const std::string& muscleName) const;

    /** Set the density for an existing muscle (kg/m^3). */
    void setDensity(const std::string& muscleName, const double& density);

    /** Get the specific tension for an existing muscle (Pascals (N/m^2)). */
    const double getSpecificTension(const std::string& muscleName) const;

    /** Set the specific tension for an existing muscle (Pascals (N/m^2)). */
    void setSpecificTension(const std::string& muscleName, const double& specificTension);



//==============================================================================
// PRIVATE
//==============================================================================
private:
    //--------------------------------------------------------------------------
    // Data
    //--------------------------------------------------------------------------
    MuscleMap _muscleMap;

    //--------------------------------------------------------------------------
    // ModelComponent Interface
    //--------------------------------------------------------------------------
    void connectToModel(Model& aModel) OVERRIDE_11;
    void connectIndividualMetabolicMuscle
       (Model& aModel, 
        Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter& mm);

    void setNull();
    void constructProperties();


    //--------------------------------------------------------------------------
    // MetabolicMuscleParameter Private Interface
    //--------------------------------------------------------------------------
    // Get const MetabolicMuscleParameter from the MuscleMap using a string accessor.
    const Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter* 
        getMetabolicParameters(const std::string& muscleName) const;

    // Get writable MetabolicMuscleParameter from the MuscleMap using a string accessor.
    Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter* 
        updMetabolicParameters(const std::string& muscleName);

public:


//=============================================================================
};	// END of class Umberger2010MuscleMetabolicsProbe
//=============================================================================

//==============================================================================
//          Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter
//==============================================================================
class OSIMSIMULATION_API 
    Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter 
    : public Object  
{
    OpenSim_DECLARE_CONCRETE_OBJECT(
        Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter, Object);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
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
    /**@}**/

    //=============================================================================
    // METHODS
    //=============================================================================
    //--------------------------------------------------------------------------
    // Constructor(s)
    //--------------------------------------------------------------------------
    Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter(); 

    Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter(
        const std::string& muscleName,
        double ratio_slow_twitch_fibers, 
        double muscle_mass = SimTK::NaN);

    //--------------------------------------------------------------------------
    // Muscle mass
    //--------------------------------------------------------------------------
    const double& getMuscleMass() const      { return _muscMass; }
    void setMuscleMass();

    //--------------------------------------------------------------------------
    // Internal muscle pointer
    //--------------------------------------------------------------------------
    const Muscle* getMuscle() const         { return _musc; }
    void setMuscle(Muscle* m)               { _musc = m; }


    //--------------------------------------------------------------------------
    // Object interface
    //--------------------------------------------------------------------------
private:
    void setNull();
    void constructProperties();

    //=============================================================================
    // DATA
    //=============================================================================
    // These private member variables are set by the probe that owns this
    // MetabolicMuscleParameter
    SimTK::ReferencePtr<Muscle> _musc;  // Internal pointer to the muscle that corresponds
                            // to these parameters.
    double _muscMass;       // The mass of the muscle (depends on if
                            // <use_provided_muscle_mass> is true or false.

//=============================================================================
};	// END of class MetabolicMuscleParameter
//=============================================================================



//==============================================================================
//                          MetabolicMuscleParameterSet
//==============================================================================
/**
 * MetabolicMuscleParameterSet is an internal container class containing the set 
 * of MetabolicMuscleParameters for each muscle that is probed.
 */
class OSIMSIMULATION_API 
    Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameterSet 
    : public Set<Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter>
{
    OpenSim_DECLARE_CONCRETE_OBJECT(
        Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameterSet, 
        Set<Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter>);

public:
    Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameterSet()  
    {  }   

//=============================================================================
};	// END of class MetabolicMuscleParameterSet
//=============================================================================


}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef OPENSIM_UMBERGER2010_METABOLIC_POWER_PROBE_H_
