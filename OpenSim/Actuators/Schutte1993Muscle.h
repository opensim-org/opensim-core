#ifndef __Schutte1993Muscle_h__
#define __Schutte1993Muscle_h__

// Schutte1993Muscle.h
// Author: Peter Loan
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
#include <iostream>
#include <math.h>
#include "osimActuatorsDLL.h"
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyObjPtr.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/ArrayPtrs.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Simulation/Model/Muscle.h>

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
class OSIMACTUATORS_API Schutte1993Muscle : public Muscle  
{

//=============================================================================
// DATA
//=============================================================================
protected:
	/** Scale factor for normalizing time */
	PropertyDbl _timeScaleProp;
	double &_timeScale;

	/** Parameter used in time constant of ramping up of muscle force */
	PropertyDbl _activation1Prop;
	double &_activation1;

	/** Parameter used in time constant of ramping up and ramping down of muscle force */
	PropertyDbl _activation2Prop;
	double &_activation2;

	/** Maximum isometric force that the fibers can generate */
	PropertyDbl _maxIsometricForceProp;
	double &_maxIsometricForce;

	/** Optimal length of the muscle fibers */
	PropertyDbl _optimalFiberLengthProp;
	double &_optimalFiberLength;

	/** Resting length of the tendon */
	PropertyDbl _tendonSlackLengthProp;
	double &_tendonSlackLength;

	/** Angle between tendon and fibers at optimal fiber length */
	PropertyDbl _pennationAngleProp;
	double &_pennationAngle;

	/** Maximum contraction velocity of the fibers, in optimal fiberlengths per second */
	PropertyDbl _maxContractionVelocityProp;
	double &_maxContractionVelocity;

	/** Damping factor related to maximum contraction velocity */
	PropertyDbl _dampingProp;
	double &_damping;

	/* Function representing force-length behavior of tendon */
	PropertyObjPtr<Function> _tendonForceLengthCurveProp;
	Function *&_tendonForceLengthCurve;

	/* Function representing active force-length behavior of muscle fibers */
	PropertyObjPtr<Function> _activeForceLengthCurveProp;
	Function *&_activeForceLengthCurve;

	/* Function representing passive force-length behavior of muscle fibers */
	PropertyObjPtr<Function> _passiveForceLengthCurveProp;
	Function *&_passiveForceLengthCurve;

    // index for Forces in various components
    SimTK::CacheEntryIndex _passiveForceIndex;
	 SimTK::CacheEntryIndex _tendonForceIndex;

protected:
	static const int STATE_ACTIVATION;
	static const int STATE_FIBER_LENGTH;
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	Schutte1993Muscle();
	Schutte1993Muscle(const std::string &aName,double aMaxIsometricForce,double aOptimalFiberLength,double aTendonSlackLength,double aPennationAngle);
	Schutte1993Muscle(const Schutte1993Muscle &aMuscle);
	virtual ~Schutte1993Muscle();
	virtual Object* copy() const;

#ifndef SWIG
	Schutte1993Muscle& operator=(const Schutte1993Muscle &aMuscle);
    virtual void initStateCache(SimTK::State& s, SimTK::SubsystemIndex subsystemIndex, Model& model);
    virtual void equilibrate(SimTK::State& state) const;
#endif
    void copyData(const Schutte1993Muscle &aMuscle);
    virtual void copyPropertyValues(Actuator& aActuator);

	//--------------------------------------------------------------------------
	// GET
	//--------------------------------------------------------------------------
	// Properties
	virtual double getMaxIsometricForce() const { return _maxIsometricForce; }
	virtual double getOptimalFiberLength() const { return _optimalFiberLength; }
	virtual double getTendonSlackLength() const { return _tendonSlackLength; }
	virtual double getPennationAngleAtOptimalFiberLength() const { return _pennationAngle; }
	virtual double getMaxContractionVelocity() const { return _maxContractionVelocity; }
	virtual double getTimeScale() const { return _timeScale; }
	virtual double getDamping() const { return _damping; }
	virtual bool setTimeScale(double aTimeScale);
	virtual bool setActivation1(double aActivation1);
	virtual bool setActivation2(double aActivation2);
	virtual bool setMaxIsometricForce(double aMaxIsometricForce);
	virtual bool setOptimalFiberLength(double aOptimalFiberLength);
	virtual bool setTendonSlackLength(double aTendonSlackLength);
	virtual bool setPennationAngle(double aPennationAngle);
	virtual bool setMaxContractionVelocity(double aMaxContractionVelocity);
	virtual bool setDamping(double aDamping);
	virtual double getDamping() { return _damping; }
	// Computed quantities
#ifndef SWIG
	virtual double getPennationAngle(const SimTK::State& s) const;
	virtual double getNormalizedFiberLength(const SimTK::State& s) const;
	virtual double getPassiveFiberForce(const SimTK::State& s) const;
	double getStress(const SimTK::State& s) const;
    virtual double getActivation(const SimTK::State& s) const { return getStateVariable(s, STATE_ACTIVATION); }
    virtual void setActivation(SimTK::State& s, double activation) const { setStateVariable(s, STATE_ACTIVATION, activation); }
    virtual double getActivationDeriv(const SimTK::State& s) const { return getStateVariableDeriv(s, STATE_ACTIVATION); }
    virtual void setActivationDeriv(const SimTK::State& s, double activationDeriv) const { setStateVariableDeriv(s, STATE_ACTIVATION, activationDeriv); }
    virtual double getFiberLength(const SimTK::State& s) const { return getStateVariable(s, STATE_FIBER_LENGTH); }
    virtual void setFiberLength(SimTK::State& s, double fiberLength) const { setStateVariable(s, STATE_FIBER_LENGTH, fiberLength); }
    virtual double getFiberLengthDeriv(const SimTK::State& s) const { return getStateVariableDeriv(s, STATE_FIBER_LENGTH); }
    virtual void setFiberLengthDeriv(const SimTK::State& s, double fiberLengthDeriv) const { setStateVariableDeriv(s, STATE_FIBER_LENGTH, fiberLengthDeriv); }
    virtual void setPassiveForce(const SimTK::State& s, double aForce) const;
    virtual double getPassiveForce( const SimTK::State& s) const;
	 virtual void setTendonForce(const SimTK::State& s, double aForce) const;
	 virtual double getTendonForce(const SimTK::State& s) const;

	//--------------------------------------------------------------------------
	// COMPUTATION
	//--------------------------------------------------------------------------
	virtual void computeStateDerivatives(const SimTK::State& s );
	virtual void computeEquilibrium(SimTK::State& s ) const;
	virtual double computeActuation( const SimTK::State& s ) const;
	virtual double computeIsometricForce(SimTK::State& s, double activation) const;
	virtual double computeIsokineticForceAssumingInfinitelyStiffTendon(SimTK::State& s, double aActivation);

	virtual void postScale(const SimTK::State& s, const ScaleSet& aScaleSet);
	virtual void scale(const SimTK::State& s, const ScaleSet& aScaleSet);
#endif
	virtual void setup(Model& aModel);

	virtual Function* getActiveForceLengthCurve() const;
	virtual bool setActiveForceLengthCurve(Function* aActiveForceLengthCurve);
	virtual Function* getPassiveForceLengthCurve() const;
	virtual bool setPassiveForceLengthCurve(Function* aPassiveForceLengthCurve);
	virtual Function* getTendonForceLengthCurve() const;
	virtual bool setTendonForceLengthCurve(Function* aTendonForceLengthCurve);

	OPENSIM_DECLARE_DERIVED(Schutte1993Muscle, Actuator);

private:
	double calcNonzeroPassiveForce(const SimTK::State& s, double aNormFiberLength, double aNormFiberVelocity) const;
	double calcFiberVelocity(const SimTK::State& s, double aActivation, double aActiveForce, double aVelocityDependentForce) const;
	double calcTendonForce(const SimTK::State& s, double aNormTendonLength) const;

	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class Schutte1993Muscle
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Schutte1993Muscle_h__


