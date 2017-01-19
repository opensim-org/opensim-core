/* -------------------------------------------------------------------------- *
 *                    OpenSim:  ActuatorForceTargetFast.h                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
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
 * The performance criterion is the sum of actuator stresses squared. The
 * desired accelerations are achieved by imposing a set of constraints.
 * The desired accelerations are computed according to the Proportional
 * Derivative (PD) control in order to drive the dynamic model toward a set
 * of target kinematic trajectories.
 *
 * Because the desired accelerations are achieved by imposing a set of
 * linear hard constraints, this optimization target can fail if the desired
 * accelerations cannot be achieved.  To insure that the constraints can
 * be achieved, a number of sufficiently strong actuators can be added
 * to the model.  Alternatively, one can use a different optimization
 * target ActuatorForceTarget.  The benefits of using the fast
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
    SimTK::State    _saveState;
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

    bool prepareToOptimize(SimTK::State& s, double *x) override;

    //--------------------------------------------------------------------------
    // REQUIRED OPTIMIZATION TARGET METHODS
    //--------------------------------------------------------------------------
    int objectiveFunc(const SimTK::Vector &aF, bool new_coefficients, SimTK::Real& rP) const override;
    int gradientFunc(const SimTK::Vector &x, bool new_coefficients, SimTK::Vector &gradient) const override;
    int constraintFunc( const SimTK::Vector &x, bool new_coefficients, SimTK::Vector &constraints) const override;
    int constraintJacobian(const SimTK::Vector &x, bool new_coefficients, SimTK::Matrix &jac) const override;
    CMC* getController() {return (_controller); }
private:
    void computeConstraintVector(SimTK::State& s, const SimTK::Vector &x, SimTK::Vector &c) const;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
};  // END class ActuatorForceTargetFast
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

}; // end namespace

#endif // ActuatorForceTargetFast_h__
