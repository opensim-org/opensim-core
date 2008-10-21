#ifndef __MuscleTemplate_h__
#define __MuscleTemplate_h__

// MuscleTemplate.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHORS: Darryl Thelen, Peter Loan
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
 * Copyright (c)  2006-8, Stanford University. All rights reserved. 
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


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/ArrayPtrs.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Simulation/Model/AbstractMuscle.h>
// Header to define plugin (DLL) interface
#include "osimPluginDLL.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a muscle force model. The path of the muscle is
 * stored in and calculated by the base class AbstractMuscle. The class
 * implemented here should handle all actuation and force calculations.
 *
 * To make your own muscle model you should replace the properties and
 * other member variables below with ones appropriate for your model.
 * All of the public methods related to construction, computation, and
 * scaling need to be implemented as they are needed by various tools
 * that operate on the set of muscles in a model. GET and SET methods
 * for properties and computed quantities should not be implemented
 * unless needed. Six GET methods (listed below) are pure virtual methods
 * in AbstractMuscle so they must be implemented.
 *
 * @version 1.0
 */
class OSIMPLUGIN_API MuscleTemplate : public AbstractMuscle  
{

//=============================================================================
// DATA
//=============================================================================
protected:
	// Properties are the user-specified quantities that are read in from
	// file and are used to configure your class.
	// 
	// A property consists of a type, a name, and a value.  You can
	// access each of these by calling methods on the property.  For example,
	//
	// string type = property.getType();
	// string name = property.getName();
	// double value = property.getValueDbl();
	// double x = 1.0;
	// property.setValue(x);
	// 
	// To make writing your code more streamlined, you can initialize
	// a reference to point to the value of a property.  For example,
	//
	// double &_param1 = _param1Prop.getValueDbl();
	//
	// In this way you can write your code using _param1 as a normal
	// variable, instead of using get and set methods on
	// _param1Prop all the time.  The references to the properties
	// (e.g., _param1) are initialized in the initialization list
	// of the class constructors.
	//
	// See AnalysisTemplate.h for a list of the different property
	// types that are available.
	//
	// As a convention, all member variables of a class are
	// preceded with an underscore (e.g., _param1).  In this way,
	// you can tell which variables are member variables and which
	// are not as you look at and modify code.

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

	/** Activation time constant */  
	PropertyDbl _activationTimeConstantProp;
	double &_activationTimeConstant;

	/** Deactivation time constant */
	PropertyDbl _deactivationTimeConstantProp;
	double &_deactivationTimeConstant;

	/** Max contraction velocity full activation in fiber lengths per second */
	PropertyDbl _vmaxProp;
	double &_vmax;

	/** Max contraction velocity at low activation */
	PropertyDbl _vmax0Prop;
	double &_vmax0;

	/** Tendon strain due to maximum isometric muscle force */
	PropertyDbl _fmaxTendonStrainProp;
	double &_fmaxTendonStrain;

	/** Passive muscle strain due to maximum isometric muscle force */
	PropertyDbl _fmaxMuscleStrainProp;
	double &_fmaxMuscleStrain;

	/** Shape factor for Gaussian active muscle force-length relationship */
	PropertyDbl _kShapeActiveProp;
	double &_kShapeActive;

	/** Exponential shape factor for passive force-length relationship */
	PropertyDbl _kShapePassiveProp;
	double &_kShapePassive;

	/** Passive damping included in the force-velocity relationship */
	PropertyDbl _dampingProp;
	double &_damping;

	/** Force-velocity shape factor */
	PropertyDbl _afProp;
	double &_af;

	/** Maximum normalized lengthening force */
	PropertyDbl _flenProp;
	double &_flen;

	// In addition to properties, add any additional member variables
	// you need for your muscle model.  These variables are not read from
	// or written to files.  They are just variables you use to calculate
	// and store values needed to implement the model.  For example,
	// _tendonForce holds the current force in the tendon.

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
	MuscleTemplate();
	MuscleTemplate(const MuscleTemplate &aMuscle);
	virtual ~MuscleTemplate();
	virtual Object* copy() const;

	MuscleTemplate& operator=(const MuscleTemplate &aMuscle);
    void copyData(const MuscleTemplate &aMuscle);
	virtual void copyPropertyValues(AbstractActuator& aActuator);

	//--------------------------------------------------------------------------
	// GET
	//--------------------------------------------------------------------------
	// Properties
	virtual double getMaxIsometricForce() { return _maxIsometricForce; }
	virtual double getOptimalFiberLength() { return _optimalFiberLength; }
	virtual double getTendonSlackLength() { return _tendonSlackLength; }
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
	// Computed quantities
	virtual double getStress() const;

	// The following GET methods are pure virtual methods in AbstractMuscle,
	// so they must be defined for each muscle class.
	virtual double getPennationAngle();
	virtual double getPennationAngleAtOptimalFiberLength() { return _pennationAngle; }
	virtual double getFiberLength();
	virtual double getNormalizedFiberLength();
	virtual double getPassiveFiberForce();
	virtual double getActivation() const { return getState(STATE_ACTIVATION); }

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual void setup(Model* aModel);
	virtual void computeStateDerivatives(double rDYDT[]);
	virtual void computeEquilibrium();
	virtual void computeActuation();
	// These two methods are pure virtual methods in AbstractMuscle,
	// so they must be defined for each muscle class.
	virtual double computeIsometricForce(double activation);
	virtual double computeIsokineticForceAssumingInfinitelyStiffTendon(double aActivation);

	//--------------------------------------------------------------------------
	// SCALE
	//--------------------------------------------------------------------------
	virtual void preScale(const ScaleSet& aScaleSet);
	virtual void scale(const ScaleSet& aScaleSet);
	virtual void postScale(const ScaleSet& aScaleSet);

	// This macro allows the OpenSim GUI to check AbstractActuator
	// objects to see if they are instances of this muscle class.
	OPENSIM_DECLARE_DERIVED(MuscleTemplate, AbstractActuator);

private:
	void setNull();
	void setupProperties();

	// These methods are called only from within this class,
	// so they are declared private.
	double calcTendonForce(double aNormTendonLength) const;
	double calcPassiveForce(double aNormFiberLength) const;
	double calcActiveForce(double aNormFiberLength) const;
	double calcFiberVelocity(double aActivation, double aActiveForce, double aVelocityDependentForce) const;
//=============================================================================
};	// END of class MuscleTemplate
//=============================================================================
//=============================================================================
} // end of namespace OpenSim

#endif // __MuscleTemplate_h__
