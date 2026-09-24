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

#include <tests/Testing.h>

#include <catch2/catch_all.hpp>

using namespace OpenSim;
using namespace std;

TEST_CASE("GCVSpline Behaves as Expected")
{
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
        OpenSim_CHECK_EQUAL(y[i], spline.calcValue(t), SimTK::SignificantReal);
    }
    cout << "GCVSpline successfully reproduced input data points." << endl;

    for (int i = 0; i < (2*size-1); ++i) {
        t[0] = dt / 2 * i;
        OpenSim_CHECK_EQUAL(sin(omega*t[0]), spline.calcValue(t), dt*dt);
    }
    cout << "GCVSpline successfully interpolated within accuracy." << endl;

    std::vector<int> derivComponents(1, 0); //take first derivative
    for (int i = 5; i < size-5; ++i) {
        t[0] = x[i];
        double dy = omega*cos(omega*t[0]);
        double dS = spline.calcDerivative(derivComponents, t);
        OpenSim_CHECK_EQUAL(dy, dS, omega*dt*dt);
    }
    cout << "GCVSpline successfully produced first derivatives." << endl;

    GCVSpline spline2(5, size, x, y);
    for (int i = 0; i < size; ++i) {
        t[0] = x[i];
        double dS = spline.calcDerivative(derivComponents, t);
        double dS2 = spline2.calcDerivative(derivComponents, t);
        OpenSim_CHECK_EQUAL(dS, dS, SimTK::Eps);
    }
}
