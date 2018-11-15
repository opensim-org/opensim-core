#ifndef OPENSIM_EXAMPLE_VECTOR_FUNCTION_UNCOUPLED_NXN_H_
#define OPENSIM_EXAMPLE_VECTOR_FUNCTION_UNCOUPLED_NXN_H_
/* -------------------------------------------------------------------------- *
 *               OpenSim:  ExampleVectorFunctionUncoupledNxN.h                *
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

#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/VectorFunctionUncoupledNxN.h>

//=============================================================================
//=============================================================================
namespace OpenSim { 

/**
* An abstract class for representing a vector function.
*
* A vector function is a relation between some number of independent variables 
* and some number of dependent values such that for any particular set of
* independent variables the correct number of dependent variables is returned.
* Values of the function and its derivatives
* are obtained by calling the evaluate() method.  The curve may or may not
* be finite or diferentiable; the evaluate method returns values between
* Math::MINUS_INFINITY and Math::PLUS_INFINITY, or it returns Math::NAN
* (not a number) if the curve is not defined.
* Currently, functions of up to 3 variables (x,y,z) are supported.
*
* @author Frank C. Anderson
*/
class ExampleVectorFunctionUncoupledNxN : public VectorFunctionUncoupledNxN {
    OpenSim_DECLARE_CONCRETE_OBJECT(ExampleVectorFunctionUncoupledNxN, 
                                    VectorFunctionUncoupledNxN);
    //==========================================================================
    // DATA
    //==========================================================================
protected:



    //==========================================================================
    // METHODS
    //==========================================================================
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    ExampleVectorFunctionUncoupledNxN():
    VectorFunctionUncoupledNxN(1)
    {
        setNull();
    }
    ExampleVectorFunctionUncoupledNxN(int aN):
    VectorFunctionUncoupledNxN(aN)
    {
        setNull();
    }
    ExampleVectorFunctionUncoupledNxN(const ExampleVectorFunctionUncoupledNxN &aVectorFunction) :
    VectorFunctionUncoupledNxN(aVectorFunction)
    {
        setNull();

        // ASSIGN
        setEqual(aVectorFunction);
    }
    virtual ~ExampleVectorFunctionUncoupledNxN() {}

private:
    void setNull(){}
    void setEqual(const ExampleVectorFunctionUncoupledNxN &aVectorFunction){}

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:
    ExampleVectorFunctionUncoupledNxN&
    operator=(const ExampleVectorFunctionUncoupledNxN& source) {
        if (&source != this) {
            // BASE CLASS
            Super::operator=(source);

            // DATA
            setEqual(source);
        }
        return *this;
    };

    //--------------------------------------------------------------------------
    // SET AND GET
    //--------------------------------------------------------------------------
public:

    //--------------------------------------------------------------------------
    // EVALUATE
    //--------------------------------------------------------------------------
    void calcValue(const double *aX,double *rY, int aSize) override {
        int N = getNX();

        // COMMON PART
        int i;
        double sum;
        double scale = 0.01;
        for(sum=0.0,i=0;i<N;i++) {
            sum += (double)i;
        }
        sum *= scale;

        // UNIQUE PART
        // Uncoupled-- each aY depends only on its corresponding aX.
        double root;
        for(i=0;i<N;i++) {
            root = scale * (double)i;
            // sin test function
            rY[i] = sum * sin(aX[i] - root);
            // parabolic test function
            //rY[i] = sum *aX[i]*aX[i]*aX[i] - sum*root*root*root; 
        }
    }
    void calcValue(const Array<double> &aX,Array<double> &rY) override {
        calcValue(&aX[0],&rY[0], aX.getSize());
    }
    void calcDerivative(const Array<double> &aX,Array<double> &rY,
        const Array<int> &aDerivWRT) override {
            std::cout<<"\nExampleVectorFunctionUncoupledNxN.evalute(x,y,derivWRT): not implemented.\n";
    }

    //=============================================================================
};

} //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_EXAMPLE_VECTOR_FUNCTION_UNCOUPLED_NXN_H_
