#ifndef OPENSIM_VECTOR_FUNCTION_UNCOUPLED_NXN_H_
#define OPENSIM_VECTOR_FUNCTION_UNCOUPLED_NXN_H_
/* -------------------------------------------------------------------------- *
 *                   OpenSim:  VectorFunctionUncoupledNxN.h                   *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Saryn R. Goldberg                            *
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
#include "VectorFunction.h"
//#include "Array.h"

namespace SimTK { class State; }

//=============================================================================
//=============================================================================
namespace OpenSim { 

template <class T> class Array;

/**
 * An abstract class for representing a vector function.
 *
 * A vector function is a relation between some number of independent variables 
 * and some number of dependent values such that for any particular set of
 * independent variables the correct number of dependent variables is returned.
 * Values of the function and its derivatives
 * are obtained by calling the evaluate() method.  The curve may or may not
 * be finite or differentiable; the evaluate method returns values between
 * -`SimTK::Infinity` and `SimTK::Infinity`, or it returns `SimTK::NaN`
 * (not a number) if the curve is not defined.
 * Currently, functions of up to 3 variables (x,y,z) are supported.
 *
 * @author Frank C. Anderson and Saryn R. Goldberg
 */
class OSIMCOMMON_API VectorFunctionUncoupledNxN : public VectorFunction {
OpenSim_DECLARE_ABSTRACT_OBJECT(VectorFunctionUncoupledNxN, VectorFunction);

//=============================================================================
// DATA
//=============================================================================
protected:



//=============================================================================
// METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    VectorFunctionUncoupledNxN();
    VectorFunctionUncoupledNxN(int aN);
    VectorFunctionUncoupledNxN(const VectorFunctionUncoupledNxN &aFunction);
    virtual ~VectorFunctionUncoupledNxN();

private:
    void setNull();
    void setEqual(const VectorFunctionUncoupledNxN &aVectorFunction);

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:
    VectorFunctionUncoupledNxN&
        operator=(const VectorFunctionUncoupledNxN &aFunction);

    //--------------------------------------------------------------------------
    // SET AND GET
    //--------------------------------------------------------------------------
    
    //--------------------------------------------------------------------------
    // EVALUATE
    //--------------------------------------------------------------------------
    virtual void evaluate( const SimTK::State& s, const double *aX, double *rF) {
        log_error("VectorFunctionUncoupledNxN UNIMPLEMENTED: evaluate( "
                  "const SimTK::State&, const double*, double*)");
    }
    virtual void evaluate( const SimTK::State& s, const Array<double> &aX, Array<double> &rF){
        log_error("VectorFunctionUncoupledNxN UNIMPLEMENTED: evaluate( "
                  "const SimTK::State&, const Array<double>, Array<double>)");
    }
    virtual void evaluate( const SimTK::State& s, const Array<double> &aX, Array<double> &rF, const Array<int> &aDerivWRT){
        log_error("VectorFunctionUncoupledNxN UNIMPLEMENTED: evaluate( "
                     "const SimTK::State&, const Array<double>&a, "
                     "Array<double>&, const Array<int>&)");
    }

//=============================================================================
};  // END class VectorFunctionUncoupledNxN

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_VECTOR_FUNCTION_UNCOUPLED_NXN_H_
