#ifndef __Delp1990Muscle_h__
#define __Delp1990Muscle_h__

// Delp1990Muscle.h
// Author: Peter Loan
/*
 * Copyright (c)  2008, Stanford University. All rights reserved. 
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
#include <OpenSim/Common/PropertyObjPtr.h>
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
 * A class implementing a SIMM muscle. This implementation is based on muscle
 * model 2 from the Dynamics Pipeline, but is modified slightly so that when
 * fiber_velocity = 0.0, this model calculates lengths and forces that match
 * SIMM's isometric calculations.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMACTUATORS_API Delp1990Muscle : public ActivationFiberLengthMuscle  
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

	/** Mass between the tendon and muscle fibers */
	PropertyDbl _massProp;
	double &_mass;

	/* Function representing force-length behavior of tendon */
	PropertyObjPtr<Function> _tendonForceLengthCurveProp;
	Function *&_tendonForceLengthCurve;

	/* Function representing active force-length behavior of muscle fibers */
	PropertyObjPtr<Function> _activeForceLengthCurveProp;
	Function *&_activeForceLengthCurve;

	/* Function representing passive force-length behavior of muscle fibers */
	PropertyObjPtr<Function> _passiveForceLengthCurveProp;
	Function *&_passiveForceLengthCurve;

	/* Function representing force-velocity behavior of muscle fibers */
	PropertyObjPtr<Function> _forceVelocityCurveProp;
	Function *&_forceVelocityCurve;

private:
	static const int STATE_ACTIVATION;
	static const int STATE_FIBER_LENGTH;
	static const int STATE_FIBER_VELOCITY;
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	Delp1990Muscle();
	Delp1990Muscle(const std::string &aName,double aMaxIsometricForce,double aOptimalFiberLength,double aTendonSlackLength,double aPennationAngle);
	Delp1990Muscle(const Delp1990Muscle &aMuscle);
	virtual ~Delp1990Muscle();
	virtual Object* copy() const;

#ifndef SWIG
	Delp1990Muscle& operator=(const Delp1990Muscle &aMuscle);

#endif
	void copyData(const Delp1990Muscle &aMuscle);

	//--------------------------------------------------------------------------
	// GET
	//--------------------------------------------------------------------------
	// Properties
	virtual double getTimeScale() const { return _timeScale; }
	virtual double getMass() const { return _mass; }
	virtual bool setTimeScale(double aTimeScale);
	virtual bool setActivation1(double aActivation1);
	virtual bool setActivation2(double aActivation2);
	virtual bool setMass(double aMass);
	// Computed quantities
#ifndef SWIG
	virtual double getFiberVelocity(const SimTK::State& s) const { return getStateVariable(s, STATE_FIBER_VELOCITY); }
	virtual void setFiberVelocity(SimTK::State& s, double fiberVelocity) const { setStateVariable(s, STATE_FIBER_VELOCITY, fiberVelocity); }
	virtual double getFiberVelocityDeriv(const SimTK::State& s) const { return getStateVariableDeriv(s, STATE_FIBER_VELOCITY); }
	virtual void setFiberVelocityDeriv(const SimTK::State& s, double fiberVelocityDeriv) const { setStateVariableDeriv(s, STATE_FIBER_VELOCITY, fiberVelocityDeriv); }
	virtual void setActiveForce(const SimTK::State& s, double aForce) const;
	virtual double getActiveForce(const SimTK::State& s) const;

	//--------------------------------------------------------------------------
	// COMPUTATION
	//--------------------------------------------------------------------------
	virtual double computeActuation(const SimTK::State& s) const;
	virtual double computeIsometricForce(SimTK::State& s, double activation) const;

#endif

	virtual Function* getActiveForceLengthCurve() const;
	virtual bool setActiveForceLengthCurve(Function* aActiveForceLengthCurve);
	virtual Function* getPassiveForceLengthCurve() const;
	virtual bool setPassiveForceLengthCurve(Function* aPassiveForceLengthCurve);
	virtual Function* getTendonForceLengthCurve() const;
	virtual bool setTendonForceLengthCurve(Function* aTendonForceLengthCurve);
	virtual Function* getForceVelocityCurve() const;
	virtual bool setForceVelocityCurve(Function* aForceVelocityCurve);

	virtual int getStateVariableYIndex(int index) const;
	OPENSIM_DECLARE_DERIVED(Delp1990Muscle, ActivationFiberLengthMuscle);

protected:
	// Model Component Interface
	virtual void setup(Model& aModel);
	virtual void createSystem(SimTK::MultibodySystem& system) const;
	virtual std::string Delp1990Muscle::getStateVariableName(int aIndex) const;
	virtual SimTK::Vector computeStateVariableDerivatives(const SimTK::State &s) const;

private:
	void setNull();
	void setupProperties();
	double calcTendonForce(const SimTK::State& s, double aNormTendonLength) const;
	double calcFiberForce(double aActivation, double aNormFiberLength, double aNormFiberVelocity) const;

//=============================================================================
};	// END of class Delp1990Muscle
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Delp1990Muscle_h__


