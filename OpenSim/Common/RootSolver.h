#ifndef _RootSolver_h_
#define _RootSolver_h_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  RootSolver.h                           *
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

/*  
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include "osimCommonDLL.h"
#include "Array.h"
#include "VectorFunctionUncoupledNxN.h"


//template class OSIMCOMMON_API Array<double>;

namespace SimTK { class State; }

namespace OpenSim { 

//=============================================================================
//=============================================================================
/**
 * A class for finding the N roots of N one-dimensional non-linear equations.
 * 
 * The rational for making this class solve for the roots of N equations
 * simultaneously is that, for some problems, it is more computationally
 * efficient to evaluate the N equations at the same time, as opposed to one
 * after the other.  That is, the N equations, though decoupled, do share
 * some common terms.
 *
 * This class can always be used for a system where N=1, although there
 * will be some small amount of overhead for this class to function in
 * this way when compared to a class that is dedicated to an N=1.
 *
 * To construct an instance of this class, the user must provide an
 * instance of a VectorFunctionUncoupledNxN.
 *
 * @version 1.0
 * @author Frank C. Anderson
 */
class OSIMCOMMON_API RootSolver
{
//=============================================================================
// MEMBER VARIABLES
//=============================================================================
private:
    
    VectorFunctionUncoupledNxN *_function;


//=============================================================================
// METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    RootSolver(VectorFunctionUncoupledNxN *aFunc);
    virtual ~RootSolver();
private:
    void setNull();

    //--------------------------------------------------------------------------
    // SOLVE
    //--------------------------------------------------------------------------
public:
    Array<double> solve(const SimTK::State& s, const Array<double> &ax,const Array<double> &bx,
        const Array<double> &tol);

//=============================================================================
};  // END class RootSolver

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __RootSolver_h__
