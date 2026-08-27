/* -------------------------------------------------------------------------- *
 *                        OpenSim:  testRootSolver.cpp                        *
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
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Common/Signal.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyIntArray.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Common/PropertySet.h>
#include <OpenSim/Common/RootSolver.h>
#include <OpenSim/Common/VectorFunctionUncoupledNxN.h>

#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <catch2/catch_all.hpp>

#include <cmath>
#include <iostream>
#include <string>

using namespace OpenSim;
using namespace std;


namespace {

    /**
    * An abstract class for representing a vector function.
    *
    * A vector function is a relation between some number of independent
    * variables and some number of dependent values such that for any particular
    * set of independent variables the correct number of dependent variables is
    * returned. Values of the function and its derivatives are obtained by
    * calling the evaluate() method. The curve may or may not be finite or
    * differentiable; the evaluate method returns values between
    * Math::MINUS_INFINITY and Math::PLUS_INFINITY, or it returns Math::NAN
    * (not a number) if the curve is not defined.
    * Currently, functions of up to 3 variables (x,y,z) are supported.
    *
    * @author Frank C. Anderson
    */
    class ExampleVectorFunctionUncoupledNxN : public VectorFunctionUncoupledNxN {
        OpenSim_DECLARE_CONCRETE_OBJECT(ExampleVectorFunctionUncoupledNxN,
                                        VectorFunctionUncoupledNxN);

    public:
        // CONSTRUCTION
        ExampleVectorFunctionUncoupledNxN() : VectorFunctionUncoupledNxN(1) {}
        ExampleVectorFunctionUncoupledNxN(int aN) :
            VectorFunctionUncoupledNxN(aN) {}
        ExampleVectorFunctionUncoupledNxN(
            const ExampleVectorFunctionUncoupledNxN &aVectorFunction) :
                VectorFunctionUncoupledNxN(aVectorFunction) {}
        virtual ~ExampleVectorFunctionUncoupledNxN() {}

        // OPERATORS
        ExampleVectorFunctionUncoupledNxN&
        operator=(const ExampleVectorFunctionUncoupledNxN& source) {
            if (&source != this) {
                Super::operator=(source);
            }
            return *this;
        };

        // EVALUATE
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

        void calcDerivative(const Array<double>&, Array<double>&,
                const Array<int>&) override {
            log_cout("ExampleVectorFunctionUncoupledNxN.calcDerivative(): "
                     "not implemented.");
        }

    };
}


TEST_CASE("APDM Data Reader")
{
    // CONSTRUCT THE UNCOUPLED VECTOR FUNCTION
    int N = 101;
    ExampleVectorFunctionUncoupledNxN function(N);

    // EVALUATE THE FUNCTION
    cout<<"\n\nEvaluate the function:\n";
    Array<double> x(0.0, N), y(0.0, N);
    function.calcValue(&x[0], &y[0], N);
    cout << "x:\n" << x << endl;
    cout << "y:\n" << y << endl;

    // ROOT SOLVE
    Array<double> a(-1.0,N), b(1.0,N), tol(1.0e-6,N);
    Array<double> roots(0.0,N);
    RootSolver solver(&function);
    //roots = solver.solve(a,b,tol);  NOTE: JACKM need to pass in state or change CMC
    cout<<endl<<endl<<"-------------"<<endl;
    cout<<"roots:\n";
    cout<<roots<<endl<<endl;
    for (int i=0; i <= 100; i++){
        //ASSERT_EQUAL(i*0.01, roots[i], 1e-6);
    }
}
