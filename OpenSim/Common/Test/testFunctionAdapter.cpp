/* -------------------------------------------------------------------------- *
 *                     OpenSim:  testFunctionAdapter.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

#include <OpenSim/Common/FunctionAdapter.h>
#include <OpenSim/Common/PiecewiseLinearFunction.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

int main() {
    try {
        double x[] = {0.0, 1.0, 2.0, 2.5, 5.0, 10.0};
        double y[] = {0.5, 0.7, 2.0, -1.0, 0.5, 0.1};
        PiecewiseLinearFunction f1(6, x, y);
        FunctionAdapter adapter(f1);
        std::unique_ptr<SimTK::Function> func_ptr{f1.createSimTKFunction()};
        const SimTK::Function& f2 = *func_ptr;
        SimTK::Vector xvec(1);
        vector<int> deriv(1, 0);
        for (int i = 0; i < 100; ++i) {
            xvec[0] = i*0.01;
            ASSERT_EQUAL(f1.calcValue(xvec), adapter.calcValue(xvec), 1e-10, __FILE__, __LINE__);
            ASSERT_EQUAL(f1.calcDerivative(deriv,xvec), adapter.calcDerivative(deriv,xvec), 1e-10, __FILE__, __LINE__);
            ASSERT_EQUAL(f1.calcValue(xvec), f2.calcValue(xvec),  1e-10, __FILE__, __LINE__);
            ASSERT_EQUAL(f1.calcDerivative(deriv,xvec), f2.calcDerivative(deriv,xvec), 1e-10, __FILE__, __LINE__);
        }
        ASSERT(adapter.getArgumentSize() == 1, __FILE__, __LINE__);
    }
    catch (const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}
