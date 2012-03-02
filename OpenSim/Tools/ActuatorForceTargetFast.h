// ActuatorForceTargetFast.h
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
#ifndef ActuatorForceTargetFast_h__
#define ActuatorForceTargetFast_h__


//==============================================================================
// INCLUDES
//==============================================================================
#include "osimToolsDLL.h"
#include <OpenSim/Common/OptimizationTarget.h>
#include <SimTKcommon.h>
#include <OpenSim/Common/Storage.h>
#include "SimTKsimbody.h"

namespace OpenSim {

class CMC;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/**
 * A Computed Muscle Control (CMC) optimization target for controlling 
 * dynamic systems whose actuators may be themselves governed by differential
 * equations, meaning there may be non-linear behavior and delays in force
 * production.
 *
 * The performance criterion is the sum of actutor stresses squared. The
 * desired accelerations are achieved by imposing a set of constraints.
 * The desired accelerations are computed according to the Proportional
 * Derivative (PD) control in order to drive the dynamic model toward a set
 * of target kinematic trajectories.
 *
 * Because the desired accelerations are achieved by imposing a set of
 * linear hard constraints, this optimiztion target can fail if the desired
 * accelreations cannot be achieved.  To insure that the constraints can
 * be achieved, a number of sufficiently strong actuators can be added
 * to the model.  Althernatively, one can use a different optimization
 * target @wee ActuatorForceTarget.  The benefits of using the fast
 * target are both speed and tracking accuracy.
 * 
 * @version 1.0
 * @author Frank C. Anderson
 */
class OSIMTOOLS_API ActuatorForceTargetFast : public OptimizationTarget
{
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//==============================================================================
// DATA
//==============================================================================
private:
	/** Model. */
	CMC *_controller;
	/** Work array of the states. */
	Array<double> _y;
	/** Work array of the derivatives of the derivatives of the states. */
	Array<double> _dydt;
	/** Array of the derivatives of the generalized coordinates. */
	Array<double> _dqdt;
	/** Array of the derivatives of the generalized speeds. */
	Array<double> _dudt;
	/** Reciprocal of actuator area squared. */
	Array<double> _recipAreaSquared;
	/** Reciprocal of optimal force squared accounting for force-length curve if actuator is a muscle. */
	Array<double> _recipOptForceSquared;
	
	Array<double> _recipAvgActForceRangeSquared;

	SimTK::Matrix _constraintMatrix;
	SimTK::Vector _constraintVector;
	
	// Save a (copy) of the state for state tracking purposes
	SimTK::State	_saveState;
//==============================================================================
// METHODS
//==============================================================================
public:
	//---------------------------------------------------------------------------
	// CONSTRUCTION
	//---------------------------------------------------------------------------
	virtual ~ActuatorForceTargetFast();
	ActuatorForceTargetFast(SimTK::State& s, int aNX,CMC *aController);

private:
	void setNull();

public:

	bool prepareToOptimize(SimTK::State& s, double *x);

	//--------------------------------------------------------------------------
	// REQUIRED OPTIMIZATION TARGET METHODS
	//--------------------------------------------------------------------------
	int objectiveFunc(const SimTK::Vector &aF, bool new_coefficients, SimTK::Real& rP) const;
	int gradientFunc(const SimTK::Vector &x, bool new_coefficients, SimTK::Vector &gradient) const;
	int constraintFunc( const SimTK::Vector &x, bool new_coefficients, SimTK::Vector &constraints) const;
	int constraintJacobian(const SimTK::Vector &x, bool new_coefficients, SimTK::Matrix &jac) const;
	CMC* getController() {return (_controller); }
private:
	void computeConstraintVector(SimTK::State& s, const SimTK::Vector &x, SimTK::Vector &c) const;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
};	// END class ActuatorForceTargetFast
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

}; // end namespace

#endif // ActuatorForceTargetFast_h__
