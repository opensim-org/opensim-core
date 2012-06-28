// MetabolicMuscle.cpp
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
// INCLUDES & STATICS
//=============================================================================
#include "MetabolicMuscle.h"

using namespace std;
using namespace SimTK;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR
//=============================================================================
// Uses default (compiler-generated) destructor, copy constructor, copy 
// assignment operator.

//_____________________________________________________________________________
/**
 * Default constructor
 */
MetabolicMuscle::MetabolicMuscle() : Object()
{
    setNull();
    constructProperties();
}

//_____________________________________________________________________________
/** 
 * Convenience constructor
 */
MetabolicMuscle::MetabolicMuscle(double muscle_mass, double ratio_slow_twitch_fibers, 
    double activation_constant_slow_twitch, double activation_constant_fast_twitch, 
    double maintenance_constant_slow_twitch, double maintenance_constant_fast_twitch): Object()
{
    setNull();
    constructProperties();

    set_muscle_mass(muscle_mass);
    set_ratio_slow_twitch_fibers(ratio_slow_twitch_fibers);
    set_activation_constant_slow_twitch(activation_constant_slow_twitch);
    set_activation_constant_fast_twitch(activation_constant_fast_twitch);
    set_maintenance_constant_slow_twitch(maintenance_constant_slow_twitch);
    set_maintenance_constant_fast_twitch(maintenance_constant_fast_twitch);
}



//_____________________________________________________________________________
/**
 * Set the data members of this MetabolicMuscle to their null values.
 */
void MetabolicMuscle::setNull()
{
    // no data members
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void MetabolicMuscle::constructProperties(void)
{
    constructProperty_muscle_mass(1.0);
    constructProperty_ratio_slow_twitch_fibers(0.5);
    constructProperty_activation_constant_slow_twitch(1.0);
    constructProperty_activation_constant_fast_twitch(1.0);
    constructProperty_maintenance_constant_slow_twitch(1.0);
    constructProperty_maintenance_constant_fast_twitch(1.0);
}



//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the parameters.
 */
double MetabolicMuscle::getMuscleMass() const
{
    return get_muscle_mass();
}

double MetabolicMuscle::getRatioSlowTwitchFibers() const
{
    return get_ratio_slow_twitch_fibers();
}

double MetabolicMuscle::getActivationConstantSlowTwitch() const
{
    return get_activation_constant_slow_twitch();
}

double MetabolicMuscle::getActivationConstantFastTwitch() const
{
    return get_activation_constant_fast_twitch();
}

double MetabolicMuscle::getMaintenanceConstantSlowTwitch() const
{
    return get_maintenance_constant_slow_twitch();
}

double MetabolicMuscle::getMaintenanceConstantFastTwitch() const
{
    return get_maintenance_constant_fast_twitch();
}

//_____________________________________________________________________________
/**
 * Set the parameters.
 */

void MetabolicMuscle::setMuscleMass(const double aMuscleMass)
{
    set_muscle_mass(aMuscleMass);
}

void MetabolicMuscle::setRatioSlowTwitchFibers(const double aRatioSlowTwitchFibers)
{
    set_ratio_slow_twitch_fibers(aRatioSlowTwitchFibers);
}

void MetabolicMuscle::setActivationConstantSlowTwitch(const double aActivationConstantSlowTwitch)
{
    set_activation_constant_slow_twitch(aActivationConstantSlowTwitch);
}

void MetabolicMuscle::setActivationConstantFastTwitch(const double aActivationConstantFastTwitch)
{
    set_activation_constant_fast_twitch(aActivationConstantFastTwitch);
}

void MetabolicMuscle::setMaintenanceConstantSlowTwitch(const double aMaintenanceSlowTwitch)
{
    set_maintenance_constant_slow_twitch(aMaintenanceSlowTwitch);
}

void MetabolicMuscle::setMaintenanceConstantFastTwitch(const double aMaintenanceFastTwitch)
{
    set_maintenance_constant_fast_twitch(aMaintenanceFastTwitch);
}

