// ActuatorForceTarget.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Contributors: Frank C. Anderson
//
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

//
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef ActuatorForceTarget_h__
#define ActuatorForceTarget_h__

#include "osimToolsDLL.h"
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/OptimizationTarget.h>
#include <SimTKmath.h>

namespace OpenSim {

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/**
 * A Computed Muscle Control (CMC) optimization target for controlling 
 * dynamic systems whose actuators may be themselves governed by differential
 * equations, meaning there may be non-linear behavior and delays in force
 * production.
 *
 * The performance criterion is a sum of two terms.  The first term
 * is the sum of actutor stresses squared.  The second term is a weighted
 * sum of terms designed to achieve a set of desired accelerations that
 * will drive the dynamic model toward a set of target kinematic trajectories.
 * The desired accelerations are according to the Proportional Derivative (PD)
 * control law.
 *
 * Because the performance criterion is simply a long sum of things,
 * achieving the desired accelerations can be compromised in order to
 * reduce the forces (or moments) applied by the actutors.  This feature
 * is what is exploited by the Residual Reduction Algorithm.
 *
 * Although this target is fairly robust (meaning the optimizer should not
 * fail to find a solution), it is a bit slower and less accurate than
 * the "fast" target @see ActuatorForceTargetFast.
 * 
 * @version 1.0
 * @author Frank C. Anderson
 */
class OSIMTOOLS_API ActuatorForceTarget : public OptimizationTarget
{
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//==============================================================================
// DATA
//==============================================================================
private:
	/** Parent controller. */
	CMC *_controller;
	/** Work array of the states. */
	Array<double> _y;
	/** Work array of the derivatives of the states. */
	Array<double> _dydt;
	/** Array of the derivatives of the generalized coordinates. */
	Array<double> _dqdt;
	/** Array of the derivatives of the generalized speeds. */
	Array<double> _dudt;
	/** Weight used to scale the stress term of objective function. **/
	double _stressTermWeight;

	SimTK::Matrix _performanceGradientMatrix;
	SimTK::Vector _performanceGradientVector;
	SimTK::Matrix _accelPerformanceMatrix, _forcePerformanceMatrix;
	SimTK::Vector _accelPerformanceVector, _forcePerformanceVector;

	double *_lapackA;
	double *_lapackB;
	double *_lapackSingularValues;
	int _lapackLWork;
	double *_lapackWork;

	// Save a (copy) of the state for state tracking purposes
	SimTK::State	_saveState;

//==============================================================================
// METHODS
//==============================================================================
public:
	//---------------------------------------------------------------------------
	// CONSTRUCTION
	//---------------------------------------------------------------------------
	virtual ~ActuatorForceTarget();
	ActuatorForceTarget(int aNX,CMC *aController);

public:
	void setStressTermWeight(double aWeight);
	bool prepareToOptimize(SimTK::State& s, double *x);

	//--------------------------------------------------------------------------
	// REQUIRED OPTIMIZATION TARGET METHODS
	//--------------------------------------------------------------------------
   int objectiveFunc(const SimTK::Vector &aF, bool new_coefficients, SimTK::Real& rP) const;
   int gradientFunc(const SimTK::Vector &x, bool new_coefficients, SimTK::Vector &gradient ) const;

private:
	void computePerformanceVectors(SimTK::State& s, const SimTK::Vector &aF, SimTK::Vector &rAccelPerformanceVector, SimTK::Vector &rForcePerformanceVector);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
};	// END class ActuatorForceTarget
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

}; // end namespace

#endif // ActuatorForceTarget_h__
