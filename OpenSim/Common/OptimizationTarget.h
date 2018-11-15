#ifndef _OptimizationTarget_h_
#define _OptimizationTarget_h_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  OptimizationTarget.h                       *
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include "osimCommonDLL.h"
#include "Array.h"
#include <simmath/Optimizer.h>


namespace OpenSim { 

//=============================================================================
//=============================================================================
/**
 * This class provides an interface specification for optimizing redundant
 * systems.  If a class represents a redundant system for which one would
 * like to find a set of optimal controls, the class should inherit from
 * this class and implement the virtual functions defined here.
 * @author Frank C. Anderson
 */
class OSIMCOMMON_API OptimizationTarget : public SimTK::OptimizerSystem
{
//=============================================================================
// DATA
//=============================================================================
public:
    /** Smallest allowable perturbation size for computing derivatives. */
    static const double SMALLDX;
protected:
    /** Perturbation size for computing numerical derivatives. */
    Array<double> _dx;

//=============================================================================
// METHODS
//=============================================================================
public:
    OptimizationTarget(int aNX=0);

    // SET AND GET
    void setNumParameters(const int aNX); // OptimizerSystem function
    void setDX(double aVal);
    void setDX(int aIndex,double aVal);
    double getDX(int aIndex);
    double* getDXArray();

    // UTILITY
    void validatePerturbationSize(double &aSize);

    virtual bool prepareToOptimize(SimTK::State& s, double *x) { return false; }
    virtual void printPerformance(double *x);

    static int
        CentralDifferencesConstraint(const OptimizationTarget *aTarget,
        double *dx,const SimTK::Vector &x,SimTK::Matrix &jacobian);
    static int
        CentralDifferences(const OptimizationTarget *aTarget,
        double *dx,const SimTK::Vector &x,SimTK::Vector &dpdx);
    static int
        ForwardDifferences(const OptimizationTarget *aTarget,
        double *dx,const SimTK::Vector &x,SimTK::Vector &dpdx);

};

}; //namespace

#endif // _OptimizationTarget_h_
