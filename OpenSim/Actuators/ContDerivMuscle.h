#ifndef __ContDerivMuscle_h__
#define __ContDerivMuscle_h__

// ContDerivMuscle.h
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
 * A class implementing a SIMM muscle.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMACTUATORS_API ContDerivMuscle : public ActivationFiberLengthMuscle  
{

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	ContDerivMuscle();
	ContDerivMuscle(const ContDerivMuscle &aMuscle);
	virtual ~ContDerivMuscle();
	virtual Object* copy() const;

#ifndef SWIG
	ContDerivMuscle& operator=(const ContDerivMuscle &aMuscle);
#endif
	void copyData(const ContDerivMuscle &aMuscle);

	//--------------------------------------------------------------------------
	// GET
	//--------------------------------------------------------------------------
	// Properties
	virtual double getActivationTimeConstant() const { return getPropertyValue<double>("activation_time_constant"); }
	virtual double getDeactivationTimeConstant() const { return getPropertyValue<double>("deactivation_time_constant"); }
	virtual double getVmax() const { return getPropertyValue<double>("Vmax"); }
	virtual double getVmax0() const { return getPropertyValue<double>("Vmax0"); }
	virtual double getFmaxTendonStrain() const { return getPropertyValue<double>("FmaxTendonStrain"); }
	virtual double getFmaxMuscleStrain() const { return getPropertyValue<double>("FmaxMuscleStrain"); }
	virtual double getKshapeActive() const { return getPropertyValue<double>("KshapeActive"); }
	virtual double getKshapePassive() const { return getPropertyValue<double>("KshapePassive"); }
	virtual double getDamping() const { return getPropertyValue<double>("damping"); }
	virtual double getAf() const { return getPropertyValue<double>("Af"); }
	virtual double getFlen() const { return getPropertyValue<double>("Flen"); }
	// Computed quantities
	virtual double getNormalizedFiberLength(const SimTK::State& s) const;
	virtual double getPassiveFiberForce(const SimTK::State& s) const;
	virtual double getStress(const SimTK::State& s) const;
	virtual double getActivation(const SimTK::State& s) const { return getStateVariable(s, STATE_ACTIVATION_NAME); }
    virtual void setActivation(SimTK::State& s, double activation) const { setStateVariable(s, STATE_ACTIVATION_NAME, activation); }
    virtual double getActivationDeriv(const SimTK::State& s) const { return getStateVariableDeriv(s, STATE_ACTIVATION_NAME); }
    virtual void setActivationDeriv(const SimTK::State& s, double activationDeriv) const { setStateVariableDeriv(s, STATE_ACTIVATION_NAME, activationDeriv); }
    virtual double getFiberLength(const SimTK::State& s) const { return getStateVariable(s, STATE_FIBER_LENGTH_NAME); }
    virtual void setFiberLength(SimTK::State& s, double fiberLength) const { setStateVariable(s, STATE_FIBER_LENGTH_NAME, fiberLength); }
    virtual double getFiberLengthDeriv(const SimTK::State& s) const { return getStateVariableDeriv(s, STATE_FIBER_LENGTH_NAME); }
    virtual void setFiberLengthDeriv(const SimTK::State& s, double fiberLengthDeriv) const { setStateVariableDeriv(s, STATE_FIBER_LENGTH_NAME, fiberLengthDeriv); }
    virtual void setTendonForce(const SimTK::State& s, double aForce) const;
    virtual double getTendonForce( const SimTK::State& s) const;
    virtual void setActiveForce(const SimTK::State& s, double aForce) const;
    virtual double getActiveForce( const SimTK::State& s) const;
    virtual void setPassiveForce(const SimTK::State& s, double aForce) const;
    virtual double getPassiveForce( const SimTK::State& s) const;



	//--------------------------------------------------------------------------
	// FORCE-LENGTH-VELOCITY PROPERTIES
	//--------------------------------------------------------------------------

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual void computeEquilibrium(SimTK::State& s ) const;
	virtual double computeActuation(const SimTK::State& s) const;
	double calcTendonForce(const SimTK::State& s, double aNormTendonLength) const;
	double calcPassiveForce(const SimTK::State& s, double aNormFiberLength) const;
	double calcActiveForce(const SimTK::State& s, double aNormFiberLength) const;
	double calcFiberVelocity(const SimTK::State& s, double aActivation, double aActiveForce, double aVelocityDependentForce) const;
	virtual double computeIsometricForce(SimTK::State& s, double activation) const;

	OPENSIM_DECLARE_DERIVED(ContDerivMuscle, ActivationFiberLengthMuscle);

protected:
	// Model Component Interface
	virtual void setup(Model& aModel);
	virtual void createSystem(SimTK::MultibodySystem& system) const;
	virtual SimTK::Vector computeStateVariableDerivatives(const SimTK::State &s) const;

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class ContDerivMuscle
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __ContDerivMuscle_h__


