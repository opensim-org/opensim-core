#ifndef OPENSIM_THELEN_2003_MUSCLE_DEPRECATED_H_
#define OPENSIM_THELEN_2003_MUSCLE_DEPRECATED_H_

// Thelen2003Muscle_Deprecated.h
/*
 * Copyright (c)  2006-12, Stanford University. All rights reserved. 
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
#include "osimActuatorsDLL.h"
#include <OpenSim/Simulation/Model/ActivationFiberLengthMuscle_Deprecated.h>

#ifdef SWIG
	#ifdef OSIMACTUATORS_API
		#undef OSIMACTUATORS_API
		#define OSIMACTUATORS_API
	#endif
#endif

namespace OpenSim {

//==============================================================================
//                   THELEN 2003 MUSCLE (DEPRECATED)
//==============================================================================
/**
 * A class implementing a SIMM muscle.
 *
 * @author Peter Loan
 */
class OSIMACTUATORS_API Thelen2003Muscle_Deprecated 
:   public ActivationFiberLengthMuscle_Deprecated {
OpenSim_DECLARE_CONCRETE_OBJECT(Thelen2003Muscle_Deprecated, 
                                ActivationFiberLengthMuscle_Deprecated);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations 
    These are the serializable properties associated with this class. **/
    /**@{**/
    OpenSim_DECLARE_PROPERTY(activation_time_constant, double,
		"time constant for ramping up of muscle activation");
	OpenSim_DECLARE_PROPERTY(deactivation_time_constant, double,
		"time constant for ramping down of muscle activation");
	OpenSim_DECLARE_PROPERTY(Vmax, double,
		"maximum contraction velocity at full activation in fiber lengths/second");
	OpenSim_DECLARE_PROPERTY(Vmax0, double,
		"maximum contraction velocity at low activation in fiber lengths/second");
	OpenSim_DECLARE_PROPERTY(FmaxTendonStrain, double,
		"tendon strain due to maximum isometric muscle force");
	OpenSim_DECLARE_PROPERTY(FmaxMuscleStrain, double,
		"passive muscle strain due to maximum isometric muscle force");
	OpenSim_DECLARE_PROPERTY(KshapeActive, double,
		"shape factor for Gaussian active muscle force-length relationship");
	OpenSim_DECLARE_PROPERTY(KshapePassive, double,
		"exponential shape factor for passive force-length relationship");
	OpenSim_DECLARE_PROPERTY(damping, double,
		"passive damping in the force-velocity relationship");
	OpenSim_DECLARE_PROPERTY(Af, double,
		"force-velocity shape factor");
	OpenSim_DECLARE_PROPERTY(Flen, double,
		"maximum normalized lengthening force");
	/**@}**/

//==============================================================================
// PUBLIC METHODS
//==============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	Thelen2003Muscle_Deprecated();
	Thelen2003Muscle_Deprecated(const std::string&  name,
                                double              maxIsometricForce,
                                double              optimalFiberLength,
                                double              tendonSlackLength,
                                double              pennationAngle);
	
	// Properties
	double getActivationTimeConstant() const 
    {   return get_activation_time_constant(); }
	double getDeactivationTimeConstant() const 
    {   return get_deactivation_time_constant(); }
	double getVmax() const {return get_Vmax();}
	double getVmax0() const {return get_Vmax0();}
	double getFmaxTendonStrain() const {return get_FmaxTendonStrain();}
	double getFmaxMuscleStrain() const {return get_FmaxMuscleStrain();}
	double getKshapeActive() const {return get_KshapeActive();}
	double getKshapePassive() const {return get_KshapePassive();}
	double getDamping() const {return get_damping();}
	double getAf() const {return get_Af();}
	double getFlen() const {return get_Flen();}

	void setActivationTimeConstant(double aActivationTimeConstant)
    {	set_activation_time_constant(aActivationTimeConstant); }
	void setDeactivationTimeConstant(double aDeactivationTimeConstant)
    {	set_deactivation_time_constant(aDeactivationTimeConstant); }
	void setVmax(double aVmax)
    {	set_Vmax(aVmax); }
	void setVmax0(double aVmax0)
    {	set_Vmax0(aVmax0); }
	void setFmaxTendonStrain(double aFmaxTendonStrain)
    {	set_FmaxTendonStrain(aFmaxTendonStrain); }
	void setFmaxMuscleStrain(double aFmaxMuscleStrain)
    {	set_FmaxMuscleStrain(aFmaxMuscleStrain); }
	void setKshapeActive(double aKShapeActive)
    {	set_KshapeActive(aKShapeActive); }
	void setKshapePassive(double aKshapePassive)
    {	set_KshapePassive(aKshapePassive); }
	void setDamping(double aDamping)
    {	set_damping(aDamping); }
	void setAf(double aAf)
    {	set_Af(aAf); }
	void setFlen(double aFlen)
    {	set_Flen(aFlen); }

	// Computed quantities
	//--------------------------------------------------------------------------
	// FORCE-LENGTH-VELOCITY PROPERTIES
	//--------------------------------------------------------------------------

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual double computeActuation(const SimTK::State& s) const;
	double calcTendonForce(const SimTK::State& s, double aNormTendonLength) const;
	double calcPassiveForce(const SimTK::State& s, double aNormFiberLength) const;
	double calcActiveForce(const SimTK::State& s, double aNormFiberLength) const;
	double calcFiberVelocity(const SimTK::State& s, double aActivation, double aActiveForce, double aVelocityDependentForce) const;
	virtual double computeIsometricForce(SimTK::State& s, double activation) const;

private:
	void constructProperties();
//==============================================================================
};	// END of class Thelen2003Muscle_Deprecated
//==============================================================================
//==============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_THELEN_2003_MUSCLE_DEPRECATED_H_