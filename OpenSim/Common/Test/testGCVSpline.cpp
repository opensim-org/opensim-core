/* -------------------------------------------------------------------------- *
 *                        OpenSim:  testGCVSpline.cpp                         *
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

#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

int main() {
    try {
        const int size = 101;
        const double T = 1.0;
        const double omega = 2 * SimTK::Pi;
        const double dt = T / (size - 1);
        double x[size], y[size];
        for (int i = 0; i < size; ++i) {
            x[i] = dt*i;
            y[i] = sin(omega*x[i]);
        }
        GCVSpline spline(5, size, x, y);
        SimTK::Vector t(1, 0.0);
        // Should obtain the input samples exactly
        for (int i = 0; i < size; ++i) {
            t[0] = x[i];
            //cout << t[0] << " error = " << y[i] - spline.calcValue(t) << endl;
            ASSERT_EQUAL(y[i], spline.calcValue(t),
                SimTK::SignificantReal, __FILE__, __LINE__,
                "GCVSpline failed to reproduce input data points.");
        }
        cout << "GCVSpline successfully reproduced input data points." << endl;

        for (int i = 0; i < (2*size-1); ++i) {
            t[0] = dt / 2 * i;
            //cout << t[0] << " error = " << sin(omega*t[0]) - spline.calcValue(t) << endl;
            ASSERT_EQUAL(sin(omega*t[0]), spline.calcValue(t),
                dt*dt, __FILE__, __LINE__,
                "GCVSpline failed to interpolate within accuracy requirement.");
        }
        cout << "GCVSpline successfully interpolated within accuracy." << endl;

        std::vector<int> derivComponents(1, 0); //take first derivative
        for (int i = 5; i < size-5; ++i) {
            t[0] = x[i];
            double dy = omega*cos(omega*t[0]);
            double dS = spline.calcDerivative(derivComponents, t);

            //cout << t[0] << " error = " << dy - dS<< endl;
            ASSERT_EQUAL(dy, dS,
                omega*dt*dt, __FILE__, __LINE__,
                "GCVSpline failed to reproduce accurate first derivative.");
        }
        cout << "GCVSpline successfully produced first derivatives." << endl;

        GCVSpline spline2(5, size, x, y);
        for (int i = 0; i < size; ++i) {
            t[0] = x[i];
            double dS = spline.calcDerivative(derivComponents, t);
            double dS2 = spline2.calcDerivative(derivComponents, t);
            //cout << t[0] << " error = " << dS - dS2 << endl;
            ASSERT_EQUAL(dS, dS,
                SimTK::Eps, __FILE__, __LINE__,
                "Duplicate GCVSpline failed to reproduce identical first derivative.");
        }
    }
    catch(const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}
