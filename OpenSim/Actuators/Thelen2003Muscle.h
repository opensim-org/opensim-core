#ifndef __Thelen2003Muscle_h__
#define __Thelen2003Muscle_h__

// Thelen2003Muscle.h
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


// INCLUDE
#include "osimActuatorsDLL.h"
#include <OpenSim/Simulation/Model/ActivationFiberLengthMuscle.h>

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
 * Implementation of a two state (activation and fiber-length) Muscle model by:
 * DG Thelen, Adjustment of muscle mechanics model parameters to simulate dynamic 
 * contractions in older adults. Journal of biomechanical engineering, 2003.
 * This a complete rewrite of a previous implementation by Peter Loan.
 *
 * @author Matt Millard
 * @author Ajay Seth
 * @version 3.0
 *
 * @contributor Peter Loan
 */
class OSIMACTUATORS_API Thelen2003Muscle : public ActivationFiberLengthMuscle  
{

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	Thelen2003Muscle();
	Thelen2003Muscle(const std::string &aName,double aMaxIsometricForce,double aOptimalFiberLength,double aTendonSlackLength,double aPennationAngle);
	Thelen2003Muscle(const Thelen2003Muscle &aMuscle);
	virtual ~Thelen2003Muscle();
	virtual Object* copy() const;

#ifndef SWIG
	Thelen2003Muscle& operator=(const Thelen2003Muscle &aMuscle);
#endif
	void copyData(const Thelen2003Muscle &aMuscle);

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
	void setActivationTimeConstant(double aActivationTimeConstant);
	void setDeactivationTimeConstant(double aDeactivationTimeConstant);
	void setVmax(double aVmax);
	void setVmax0(double aVmax0);
	void setFmaxTendonStrain(double aFmaxTendonStrain);
	void setFmaxMuscleStrain(double aFmaxMuscleStrain);
	void setKshapeActive(double aKShapeActive);
	void setKshapePassive(double aKshapePassive);
	void setDamping(double aDamping);
	void setAf(double aAf);
	void setFlen(double aFlen);

	// Computed quantities
	//--------------------------------------------------------------------------
	// FORCE-LENGTH-VELOCITY PROPERTIES
	//--------------------------------------------------------------------------

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	double computeActuation(const SimTK::State& s) const;
	double calcTendonForce(const SimTK::State& s, double aNormTendonLength) const;
	double calcPassiveForce(const SimTK::State& s, double aNormFiberLength) const;
	double calcActiveForce(const SimTK::State& s, double aNormFiberLength) const;
	double calcFiberVelocity(const SimTK::State& s, double aActivation, double aActiveForce, double aVelocityDependentForce) const;
	virtual double computeIsometricForce(SimTK::State& s, double activation) const;

	OPENSIM_DECLARE_DERIVED(Thelen2003Muscle, ActivationFiberLengthMuscle);

protected:
	/** calculate muscle's position related values such fiber and tendon lengths,
	normalized lengths, pennation angle, etc... */
	virtual void calcMuscleLengthInfo(const SimTK::State& s, MuscleLengthInfo& mli) const;

	/** calculate muscle's velocity related values such fiber and tendon velocities,
		normalized velocities, pennation angular velocity, etc... */
	virtual void  calcFiberVelocityInfo(const SimTK::State& s, FiberVelocityInfo& fvi) const;

	/** calculate muscle's active and passive force-length, force-velocity, 
	    tendon force, relationships and their related values */
	virtual void  calcMuscleDynamicsInfo(const SimTK::State& s, MuscleDynamicsInfo& mdi) const;

	/** Calculate activation rate */
	virtual double calcActivationRate(const SimTK::State& s) const;

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class Thelen2003Muscle
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Thelen2003Muscle_h__