#ifndef __Thelen2003Muscle_Deprecated_h__
#define __Thelen2003Muscle_Deprecated_h__

// Thelen2003Muscle_Deprecated.h
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM muscle.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMACTUATORS_API Thelen2003Muscle_Deprecated : public ActivationFiberLengthMuscle_Deprecated  
{

//=============================================================================
// DATA
//=============================================================================
protected:

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	Thelen2003Muscle_Deprecated();
	Thelen2003Muscle_Deprecated(const std::string &aName,double aMaxIsometricForce,double aOptimalFiberLength,double aTendonSlackLength,double aPennationAngle);
	Thelen2003Muscle_Deprecated(const Thelen2003Muscle_Deprecated &aMuscle);
	virtual ~Thelen2003Muscle_Deprecated();
	virtual Object* copy() const;

#ifndef SWIG
	Thelen2003Muscle_Deprecated& operator=(const Thelen2003Muscle_Deprecated &aMuscle);
#endif
	void copyData(const Thelen2003Muscle_Deprecated &aMuscle);

#ifndef SWIG

#endif

	//--------------------------------------------------------------------------
	// GET
	//--------------------------------------------------------------------------
	// Properties
	double getActivationTimeConstant() const { return getPropertyValue<double>("activation_time_constant"); }
	double getDeactivationTimeConstant() const { return getPropertyValue<double>("deactivation_time_constant"); }
	double getVmax() const { return getPropertyValue<double>("Vmax"); }
	double getVmax0() const { return getPropertyValue<double>("Vmax0"); }
	double getFmaxTendonStrain() const { return getPropertyValue<double>("FmaxTendonStrain"); }
	double getFmaxMuscleStrain() const { return getPropertyValue<double>("FmaxMuscleStrain"); }
	double getKshapeActive() const { return getPropertyValue<double>("KshapeActive"); }
	double getKshapePassive() const { return getPropertyValue<double>("KshapePassive"); }
	double getDamping() const { return getPropertyValue<double>("damping"); }
	double getAf() const { return getPropertyValue<double>("Af"); }
	double getFlen() const { return getPropertyValue<double>("Flen"); }

	void setActivationTimeConstant(double aActivationTimeConstant)
    {	setPropertyValue("activation_time_constant", aActivationTimeConstant); }
	void setDeactivationTimeConstant(double aDeactivationTimeConstant)
    {	setPropertyValue("deactivation_time_constant", aDeactivationTimeConstant); }
	void setVmax(double aVmax)
    {	setPropertyValue("Vmax", aVmax); }
	void setVmax0(double aVmax0)
    {	setPropertyValue("Vmax0", aVmax0); }
	void setFmaxTendonStrain(double aFmaxTendonStrain)
    {	setPropertyValue("FmaxTendonStrain", aFmaxTendonStrain); }
	void setFmaxMuscleStrain(double aFmaxMuscleStrain)
    {	setPropertyValue("FmaxMuscleStrain", aFmaxMuscleStrain); }
	void setKshapeActive(double aKShapeActive)
    {	setPropertyValue("KshapeActive", aKShapeActive); }
	void setKshapePassive(double aKshapePassive)
    {	setPropertyValue("KshapePassive", aKshapePassive); }
	void setDamping(double aDamping)
    {	setPropertyValue("damping", aDamping); }
	void setAf(double aAf)
    {	setPropertyValue("Af", aAf); }
	void setFlen(double aFlen)
    {	setPropertyValue("Flen", aFlen); }

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

	OPENSIM_DECLARE_DERIVED(Thelen2003Muscle_Deprecated, ActivationFiberLengthMuscle_Deprecated);

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class Thelen2003Muscle_Deprecated
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Thelen2003Muscle_Deprecated_h__