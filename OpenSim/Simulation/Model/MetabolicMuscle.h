#ifndef OPENSIM_METABOLIC_MUSCLE_H_
#define OPENSIM_METABOLIC_MUSCLE_H_

// MetabolicMuscle.h
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

// INCLUDE
#include "Model.h"
#include "Muscle.h"

namespace OpenSim {

//=============================================================================
//                              METABOLIC MUSCLE
//=============================================================================
/**
 * A class for holding the meatbolic properties of a muscle. 
 * The list of current metabolic properties are:
 *
 * - <B>muscle_mass</B> = The mass of the muscle (kg).
 * - <B>ratio_slow_twitch_fibers</B> = Ratio of slow twitch fibers in the muscle (must be between 0 and 1).
 * - <B>activation_constant_slow_twitch</B> = Activation constant for slow twitch fibers (W/kg).
 * - <B>activation_constant_fast_twitch</B> = Activation constant for fast twitch fibers (W/kg).
 * - <B>maintenance_constant_slow_twitch</B> = Maintenance constant for slow twitch fibers (W/kg).
 * - <B>maintenance_constant_fast_twitch</B> = Maintenance constant for slow twitch fibers (W/kg).
 *
 * @author Tim Dorn
 */

//class Model;
//class Muscle;

class OSIMSIMULATION_API MetabolicMuscle : public Object  
{
    OpenSim_DECLARE_CONCRETE_OBJECT(MetabolicMuscle, Object);

//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
    OpenSim_DECLARE_PROPERTY(muscle_mass, double,
        "The mass of the muscle (kg).");

    OpenSim_DECLARE_PROPERTY(ratio_slow_twitch_fibers, double,
        "Ratio of slow twitch fibers in the muscle (must be between 0 and 1).");

    OpenSim_DECLARE_PROPERTY(activation_constant_slow_twitch, double,
        "Activation constant for slow twitch fibers (W/kg).");

    OpenSim_DECLARE_PROPERTY(activation_constant_fast_twitch, double,
        "Activation constant for fast twitch fibers (W/kg).");

    OpenSim_DECLARE_PROPERTY(maintenance_constant_slow_twitch, double,
        "Maintenance constant for slow twitch fibers (W/kg).");

    OpenSim_DECLARE_PROPERTY(maintenance_constant_fast_twitch, double,
        "Maintenance constant for fast twitch fibers (W/kg).");
    /**@}**/

//=============================================================================
// DATA
//=============================================================================
protected:

//=============================================================================
// METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // Constructor(s) and Setup
    //--------------------------------------------------------------------------
    MetabolicMuscle();

    MetabolicMuscle(double muscle_mass, double ratio_slow_twitch_fibers, 
        double activation_constant_slow_twitch, double activation_constant_fast_twitch, 
        double maintenance_constant_slow_twitch, double maintenance_constant_fast_twitch); 

    // Uses default (compiler-generated) destructor, copy constructor, copy 
    // assignment operator.

    //--------------------------------------------------------------------------
    // Get and Set
    //--------------------------------------------------------------------------
    double getMuscleMass() const;
    double getRatioSlowTwitchFibers() const;
    double getActivationConstantSlowTwitch() const;
    double getActivationConstantFastTwitch() const;
    double getMaintenanceConstantSlowTwitch() const;
    double getMaintenanceConstantFastTwitch() const;

    void setMuscleMass(const double aMuscleMass);
    void setRatioSlowTwitchFibers(const double aRatioSlowTwitchFibers);
    void setActivationConstantSlowTwitch(const double aActivationConstantSlowTwitch);
    void setActivationConstantFastTwitch(const double aActivationConstantFastTwitch);
    void setMaintenanceConstantSlowTwitch(const double aMaintenanceSlowTwitch);
    void setMaintenanceConstantFastTwitch(const double aMaintenanceFastTwitch);


private:
    void setNull();
    void constructProperties();

//=============================================================================
};	// END of class MetabolicMuscle
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_METABOLIC_MUSCLE_H_
