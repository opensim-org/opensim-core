#ifndef __SimmZajacHill_h__
#define __SimmZajacHill_h__

// SimmZajacHill.h
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
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/PropertyDbl.h>
#include <OpenSim/Tools/PropertyObjPtr.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/ArrayPtrs.h>
#include <OpenSim/Tools/ScaleSet.h>
#include <OpenSim/Tools/Function.h>
#include "AbstractSimmMuscle.h"

#ifdef SWIG
	#ifdef RDSIMULATION_API
		#undef RDSIMULATION_API
		#define RDSIMULATION_API
	#endif
#endif

namespace OpenSim {

class RDSIMULATION_API SimmMuscleGroup;

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM muscle.
 *
 * @author Peter Loan
 * @version 1.0
 */
class RDSIMULATION_API SimmZajacHill : public AbstractSimmMuscle  
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
	SimmZajacHill();
	SimmZajacHill(const SimmZajacHill &aMuscle);
	virtual ~SimmZajacHill();
	virtual Object* copy() const;

#ifndef SWIG
	SimmZajacHill& operator=(const SimmZajacHill &aMuscle);
#endif
   void copyData(const SimmZajacHill &aMuscle);

	virtual void computeStateDerivatives(double rDYDT[]);
	virtual void computeActuation();

	/* Register types to be used when reading a SimmZajacHill object from xml file. */
	static void registerTypes();

	virtual void postScale(const ScaleSet& aScaleSet);
	virtual void scale(const ScaleSet& aScaleSet);
	virtual void setup(AbstractModel* aModel);

	virtual double getMaxIsometricForce() { return _maxIsometricForce; }
	virtual double getOptimalFiberLength() { return _optimalFiberLength; }
	virtual double getTendonSlackLength() { return _tendonSlackLength; }
	virtual double getPennationAngle() { return _pennationAngle; }
	virtual double getMaxContractionVelocity() { return _maxContractionVelocity; }
	virtual double getTimeScale() { return _timeScale; }
	virtual double getDamping() { return _damping; }

	virtual Function* getActiveForceLengthCurve() const;
	virtual Function* getPassiveForceLengthCurve() const;
	virtual Function* getTendonForceLengthCurve() const;
	virtual Function* getForceVelocityCurve() const;

	double calcNonzeroPassiveForce(double aNormFiberLength, double aNormFiberVelocity) const;
	double calcFiberVelocity(double aActivation, double aActiveForce, double aVelocityDependentForce) const;
	double calcTendonForce(double aNormTendonLength) const;

	double getStress() const;
	double computeIsometricForce(double activation);

	virtual void peteTest() const;

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class SimmZajacHill
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimmZajacHill_h__


