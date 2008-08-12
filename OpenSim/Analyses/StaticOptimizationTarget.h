#ifndef _StaticOptimizationTarget_h_
#define _StaticOptimizationTarget_h_
// StaticOptimizationTarget.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Jeff Reinbolt
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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
#include "osimAnalysesDLL.h"
#include "OpenSim/Common/Array.h"
#include <simmath/Optimizer.h>

//=============================================================================
//=============================================================================
/**
 * This class provides an interface specification for static optimization.
 */
namespace OpenSim { 

class OSIMANALYSES_API StaticOptimizationTarget : public SimTK::OptimizerSystem
{
//=============================================================================
// DATA
//=============================================================================
public:
	/** Smallest allowable perturbation size for computing derivatives. */
	static const double SMALLDX;
	/** . */
private:

	/** Work model. */
	Model *_model;
	/** Work variable of the time. */
	double _t;
	/** Work array of the controls. */
	double  *_x;
	/** Work array of the states. */
	double  *_y;
	/** Work array of the state derivatives. */
	double  *_dydt;
	/** Reciprocal of actuator area squared. */
	Array<double> _recipAreaSquared;
	/** Reciprocal of optimal force squared accounting for force-length curve if actuator is a muscle. */
	Array<double> _recipOptForceSquared;
	/** Optimal force accounting for force-length curve if desired and if actuator is a muscle. */
	Array<double> _optimalForce;
	
	SimTK::Matrix _constraintMatrix;
	SimTK::Vector _constraintVector;

protected:
	double _activationExponent;
	bool   _useMusclePhysiology;
	/** Perturbation size for computing numerical derivatives. */
	Array<double> _dx;
	Array<int> _accelerationIndices;

//=============================================================================
// METHODS
//=============================================================================
public:
	StaticOptimizationTarget(Model *aModel,int aNX,int aNC,double aT,double *aX,double *aY,double *aDYDT, const bool useMusclePhysiology=true);

	// SET AND GET
	void setModel(Model *aModel);
	void setNumParams(const int aNP);
	void setNumConstraints(const int aNC);
	void setTime(double aT);
	void setControls(double *aX);
	void setStates(double *aY);
	void setStateDerivatives(double *aDYDT);
	void setDX(double aVal);
	void setDX(int aIndex,double aVal);
	double getDX(int aIndex);
	double* getDXArray();
	void getActuation(const SimTK::Vector &parameters, SimTK::Vector &forces);
	void setActivationExponent(double aActivationExponent) { _activationExponent=aActivationExponent; }
	double getActivationExponent() const { return _activationExponent; }
	// UTILITY
	void validatePerturbationSize(double &aSize);

	virtual void printPerformance(double *x);

	void computeActuatorAreas();

	static int
		CentralDifferencesConstraint(const StaticOptimizationTarget *aTarget,
		double *dx,const SimTK::Vector &x,SimTK::Matrix &jacobian);
	static int
		CentralDifferences(const StaticOptimizationTarget *aTarget,
		double *dx,const SimTK::Vector &x,SimTK::Vector &dpdx);

	bool prepareToOptimize(double *x);

	//--------------------------------------------------------------------------
	// REQUIRED OPTIMIZATION TARGET METHODS
	//--------------------------------------------------------------------------
	int objectiveFunc(const SimTK::Vector &x, const bool new_coefficients, SimTK::Real& rP) const;
	int gradientFunc(const SimTK::Vector &x, const bool new_coefficients, SimTK::Vector &gradient) const;
	int constraintFunc(const SimTK::Vector &x, const bool new_coefficients, SimTK::Vector &constraints) const;
	int constraintJacobian(const SimTK::Vector &x, const bool new_coefficients, SimTK::Matrix &jac) const;

private:
	void computeConstraintVector(const SimTK::Vector &x, SimTK::Vector &c) const;
	void computeAcceleration(double aT,double *aX,double *aY,const SimTK::Vector &aF,SimTK::Vector &rAccel) const;
	void cumulativeTime(double &aTime, double aIncrement);
};

}; //namespace

#endif // _StaticOptimizationTarget_h_
