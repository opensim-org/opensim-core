#ifndef __SimmDarrylMuscle_h__
#define __SimmDarrylMuscle_h__

// SimmDarrylMuscle.h
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
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/PropertyDbl.h>
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
class RDSIMULATION_API SimmDarrylMuscle : public AbstractSimmMuscle  
{

//=============================================================================
// DATA
//=============================================================================
protected:
	PropertyDbl _maxIsometricForceProp;
	double &_maxIsometricForce;

	PropertyDbl _optimalFiberLengthProp;
	double &_optimalFiberLength;

	PropertyDbl _tendonSlackLengthProp;
	double &_tendonSlackLength;

	PropertyDbl _pennationAngleProp;
	double &_pennationAngle;

	/* Activation time constant */  
	PropertyDbl _activationTimeConstantProp;
	double &_activationTimeConstant;

	/* Deactivation time constant */
	PropertyDbl _deactivationTimeConstantProp;
	double &_deactivationTimeConstant;

	/* max contraction velocity full activation in fiber lengths per second */
	PropertyDbl _vmaxProp;
	double &_vmax;

	/* max contraction velocity at low activation */
	PropertyDbl _vmax0Prop;
	double &_vmax0;

	/* Tendon strain due to maximum isometric muscle force */
	PropertyDbl _fmaxTendonStrainProp;
	double &_fmaxTendonStrain;

	/* Passive muscle strain due to maximum isometric muscle force */
	PropertyDbl _fmaxMuscleStrainProp;
	double &_fmaxMuscleStrain;

	/* Shape factor for Gaussian active muscle force-length relationship */
	PropertyDbl _kShapeActiveProp;
	double &_kShapeActive;

	/* Exponential shape factor for passive force-length relationship */
	PropertyDbl _kShapePassiveProp;
	double &_kShapePassive;

	/* Passive damping included in the force-velocity relationship */
	PropertyDbl _dampingProp;
	double &_damping;

	/* Force-velocity shape factor */
	PropertyDbl _afProp;
	double &_af;

	/* Maximum normalized lengthening force */
	PropertyDbl _flenProp;
	double &_flen;

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
	SimmDarrylMuscle();
	SimmDarrylMuscle(const SimmDarrylMuscle &aMuscle);
	virtual ~SimmDarrylMuscle();
	virtual Object* copy() const;

#ifndef SWIG
	SimmDarrylMuscle& operator=(const SimmDarrylMuscle &aMuscle);
#endif
   void copyData(const SimmDarrylMuscle &aMuscle);

	virtual void computeStateDerivatives(double rDYDT[]);
	virtual void computeActuation();

	/* Register types to be used when reading a SimmDarrylMuscle object from xml file. */
	static void registerTypes();

	virtual void postScale(const ScaleSet& aScaleSet);
	virtual void scale(const ScaleSet& aScaleSet);
	virtual void setup(AbstractModel* aModel);

	double calcTendonForce(double aNormTendonLength) const;
	double calcPassiveForce(double aNormFiberLength) const;
	double calcActiveForce(double aNormFiberLength) const;
	double calcFiberVelocity(double aActivation, double aActiveForce, double aVelocityDependentForce) const;

	virtual double getMaxIsometricForce() { return _maxIsometricForce; }
	virtual double getOptimalFiberLength() { return _optimalFiberLength; }
	virtual double getTendonSlackLength() { return _tendonSlackLength; }
	virtual double getPennationAngle() { return _pennationAngle; }
	virtual double getActivationTimeConstant() { return _activationTimeConstant; }
	virtual double getDeactivationTimeConstant() { return _deactivationTimeConstant; }
	virtual double getVmax() { return _vmax; }
	virtual double getVmax0() { return _vmax0; }
	virtual double getFmaxTendonStrain() { return _fmaxTendonStrain; }
	virtual double getFmaxMuscleStrain() { return _fmaxMuscleStrain; }
	virtual double getKshapeActive() { return _kShapeActive; }
	virtual double getKshapePassive() { return _kShapePassive; }
	virtual double getDamping() { return _damping; }
	virtual double getAf() { return _af; }
	virtual double getFlen() { return _flen; }

	double getStress() const;
	double computeIsometricForce(double activation);

	virtual void peteTest() const;

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class SimmDarrylMuscle
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimmDarrylMuscle_h__


