#ifndef __Delp1990Muscle_Deprecated_h__
#define __Delp1990Muscle_Deprecated_h__
/* -------------------------------------------------------------------------- *
 *                   OpenSim:  Delp1990Muscle_Deprecated.h                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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


// INCLUDE
#include "osimActuatorsDLL.h"
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyObjPtr.h>
#include <OpenSim/Simulation/Model/ActivationFiberLengthMuscle_Deprecated.h>

#ifdef SWIG
	#ifdef OSIMACTUATORS_API
		#undef OSIMACTUATORS_API
		#define OSIMACTUATORS_API
	#endif
#endif

namespace OpenSim {

class Function;

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
class OSIMACTUATORS_API Delp1990Muscle_Deprecated 
:   public ActivationFiberLengthMuscle_Deprecated {
OpenSim_DECLARE_CONCRETE_OBJECT(Delp1990Muscle_Deprecated, 
                                ActivationFiberLengthMuscle_Deprecated);

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
	static const int STATE_FIBER_VELOCITY;
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	Delp1990Muscle_Deprecated();
	Delp1990Muscle_Deprecated(const std::string &aName,double aMaxIsometricForce,double aOptimalFiberLength,double aTendonSlackLength,double aPennationAngle);
	Delp1990Muscle_Deprecated(const Delp1990Muscle_Deprecated &aMuscle);
	virtual ~Delp1990Muscle_Deprecated();

#ifndef SWIG
	Delp1990Muscle_Deprecated& operator=(const Delp1990Muscle_Deprecated &aMuscle);

#endif
	void copyData(const Delp1990Muscle_Deprecated &aMuscle);

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
	virtual double getFiberVelocity(const SimTK::State& s) const { return getStateVariable(s, "fiber_velocity"); }
	virtual void setFiberVelocity(SimTK::State& s, double fiberVelocity) const { setStateVariable(s, "fiber_velocity", fiberVelocity); }
	virtual double getFiberVelocityDeriv(const SimTK::State& s) const { return getStateVariableDeriv(s, "fiber_velocity"); }
	virtual void setFiberVelocityDeriv(const SimTK::State& s, double fiberVelocityDeriv) const { setStateVariableDeriv(s, "fiber_velocity", fiberVelocityDeriv); }
	virtual void setActiveForce(const SimTK::State& s, double aForce) const;
	virtual double getActiveForce(const SimTK::State& s) const;

	//--------------------------------------------------------------------------
	// COMPUTATION
	//--------------------------------------------------------------------------
	virtual double computeActuation(const SimTK::State& s) const;
	virtual double computeIsometricForce(SimTK::State& s, double activation) const;

	virtual Function* getActiveForceLengthCurve() const;
	virtual bool setActiveForceLengthCurve(Function* aActiveForceLengthCurve);
	virtual Function* getPassiveForceLengthCurve() const;
	virtual bool setPassiveForceLengthCurve(Function* aPassiveForceLengthCurve);
	virtual Function* getTendonForceLengthCurve() const;
	virtual bool setTendonForceLengthCurve(Function* aTendonForceLengthCurve);
	virtual Function* getForceVelocityCurve() const;
	virtual bool setForceVelocityCurve(Function* aForceVelocityCurve);

protected:
	// Model Component Interface
	void connectToModel(Model& aModel) OVERRIDE_11;
	void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;
	SimTK::Vector computeStateVariableDerivatives(const SimTK::State &s) const 
                                                                OVERRIDE_11;

private:
	void setNull();
	void setupProperties();
	double calcTendonForce(const SimTK::State& s, double aNormTendonLength) const;
	double calcFiberForce(double aActivation, double aNormFiberLength, double aNormFiberVelocity) const;

//=============================================================================
};	// END of class Delp1990Muscle_Deprecated
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Delp1990Muscle_Deprecated_h__


