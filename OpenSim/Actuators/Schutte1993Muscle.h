#ifndef __Schutte1993Muscle_h__
#define __Schutte1993Muscle_h__

// Schutte1993Muscle.h
// Author: Peter Loan
/*
 * Copyright (c) 2006, Stanford University. All rights reserved. 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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
#include <OpenSim/Simulation/Model/AbstractMuscle.h>

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
class OSIMACTUATORS_API Schutte1993Muscle : public AbstractMuscle  
{

//=============================================================================
// DATA
//=============================================================================
protected:
	PropertyDbl _timeScaleProp;
	double &_timeScale;

	PropertyDbl _activation1Prop;
	double &_activation1;

	PropertyDbl _activation2Prop;
	double &_activation2;

	PropertyDbl _maxIsometricForceProp;
	double &_maxIsometricForce;

	PropertyDbl _optimalFiberLengthProp;
	double &_optimalFiberLength;

	PropertyDbl _tendonSlackLengthProp;
	double &_tendonSlackLength;

	PropertyDbl _pennationAngleProp;
	double &_pennationAngle;

	PropertyDbl _maxContractionVelocityProp;
	double &_maxContractionVelocity;

	PropertyDbl _dampingProp;
	double &_damping;

	PropertyObjPtr<Function> _tendonForceLengthCurveProp;
	Function *&_tendonForceLengthCurve;

	PropertyObjPtr<Function> _activeForceLengthCurveProp;
	Function *&_activeForceLengthCurve;

	PropertyObjPtr<Function> _passiveForceLengthCurveProp;
	Function *&_passiveForceLengthCurve;

	PropertyObjPtr<Function> _forceVelocityCurveProp;
	Function *&_forceVelocityCurve;

	// Muscle controls
	double _excitation;

	// Muscle states and derivatives
	double _activation;
	double _activationDeriv;
	double _fiberLength;
	double _fiberLengthDeriv;

	// Forces in various components
	double _tendonForce;
	double _activeForce;
	double _passiveForce;

private:
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
	Schutte1993Muscle(const Schutte1993Muscle &aMuscle);
	virtual ~Schutte1993Muscle();
	virtual Object* copy() const;

#ifndef SWIG
	Schutte1993Muscle& operator=(const Schutte1993Muscle &aMuscle);
#endif
   void copyData(const Schutte1993Muscle &aMuscle);
	virtual void copyPropertyValues(AbstractActuator& aActuator);

	//--------------------------------------------------------------------------
	// GET
	//--------------------------------------------------------------------------
	// Properties
	virtual double getMaxIsometricForce() { return _maxIsometricForce; }
	virtual double getOptimalFiberLength() { return _optimalFiberLength; }
	virtual double getTendonSlackLength() { return _tendonSlackLength; }
	virtual double getPennationAngleAtOptimalFiberLength() { return _pennationAngle; }
	virtual double getMaxContractionVelocity() { return _maxContractionVelocity; }
	virtual double getTimeScale() { return _timeScale; }
	virtual double getDamping() { return _damping; }
	// Computed quantities
	virtual double getPennationAngle();
	virtual double getFiberLength();
	virtual double getNormalizedFiberLength();
	virtual double getPassiveFiberForce();

	//--------------------------------------------------------------------------
	// COMPUTATION
	//--------------------------------------------------------------------------
	virtual void computeStateDerivatives(double rDYDT[]);
	virtual void computeEquilibrium();
	virtual void computeActuation();

	virtual void postScale(const ScaleSet& aScaleSet);
	virtual void scale(const ScaleSet& aScaleSet);
	virtual void setup(Model* aModel);

	virtual Function* getActiveForceLengthCurve() const;
	virtual Function* getPassiveForceLengthCurve() const;
	virtual Function* getTendonForceLengthCurve() const;
	virtual Function* getForceVelocityCurve() const;

	double calcNonzeroPassiveForce(double aNormFiberLength, double aNormFiberVelocity) const;
	double calcFiberVelocity(double aActivation, double aActiveForce, double aVelocityDependentForce) const;
	double calcTendonForce(double aNormTendonLength) const;

	double getStress() const;
	double computeIsometricForce(double activation);

	virtual double getActivation() const { return getState(STATE_ACTIVATION); }

	OPENSIM_DECLARE_DERIVED(Schutte1993Muscle, AbstractActuator);

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class Schutte1993Muscle
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Schutte1993Muscle_h__


