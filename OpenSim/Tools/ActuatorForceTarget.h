/* -------------------------------------------------------------------------- *
 *                      OpenSim:  ActuatorForceTarget.h                       *
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
#ifndef ActuatorForceTarget_h__
#define ActuatorForceTarget_h__

#include "osimToolsDLL.h"
#include <OpenSim/Common/Array.h>
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
 * The performance criterion is a sum of two terms.  The first term
 * is the sum of actuator stresses squared.  The second term is a weighted
 * sum of terms designed to achieve a set of desired accelerations that
 * will drive the dynamic model toward a set of target kinematic trajectories.
 * The desired accelerations are according to the Proportional Derivative (PD)
 * control law.
 *
 * Because the performance criterion is simply a long sum of things,
 * achieving the desired accelerations can be compromised in order to
 * reduce the forces (or moments) applied by the actuators.  This feature
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
    SimTK::State    _saveState;

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
    bool prepareToOptimize(SimTK::State& s, double *x) override;

    //--------------------------------------------------------------------------
    // REQUIRED OPTIMIZATION TARGET METHODS
    //--------------------------------------------------------------------------
    int objectiveFunc(const SimTK::Vector &aF, bool new_coefficients, SimTK::Real& rP) const override;
    int gradientFunc(const SimTK::Vector &x, bool new_coefficients, SimTK::Vector &gradient ) const override;

private:
    void computePerformanceVectors(SimTK::State& s, const SimTK::Vector &aF, SimTK::Vector &rAccelPerformanceVector, SimTK::Vector &rForcePerformanceVector);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
};  // END class ActuatorForceTarget
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

}; // end namespace

#endif // ActuatorForceTarget_h__
