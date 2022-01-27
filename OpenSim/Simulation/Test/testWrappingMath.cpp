/* -------------------------------------------------------------------------- *
 *                         OpenSim:  testWrappingMath.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2020 Stanford University and the Authors                *
 * Author(s): Nicholas Bianco                                                 *
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

#define CATCH_CONFIG_MAIN
#include <OpenSim/Auxiliary/catch.hpp>
#include <OpenSim/Simulation/Wrap/WrapMath.h>
#include <SimTKcommon/Testing.h>

#include <random>

using namespace OpenSim;

static std::minstd_rand rng{std::random_device{}()};
static std::uniform_real_distribution<double> dist;
static auto generate_v = []{ return dist(rng); };

template<typename C, typename D, int N = 3>
static void fillMat(C mat1, D mat2) {
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            double v = generate_v();
            mat1[i][j] = v;
            mat2[i][j] = v;
        }
    }
}

template<typename C, typename D, int N = 3>
static void fillVec(C vec1, D vec2) {
    for (int i = 0; i < N; ++i) {
        double v = generate_v();
        vec1[i] = v;
        vec2[i] = v;
    }
}

template<typename C, typename D, int N = 3>
static void compareMat(C mat1, D mat2,
                       double tol=SimTK::Test::defTol2<double, SimTK::Real>()) {
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            SimTK_TEST_EQ_TOL(mat1[i][j], mat2[i][j], tol);
        }
    }
}

/// These tests compare different methods for performing matrix operations used
/// in muscle wrapping. Specifically, we compare the original operations using
/// the WrapMath and lower-level Mtx classes to Simbody operations which are now
/// used as replacements.

static const double ANGLE = 0.1;
static const SimTK::Vec3 AXIS(1.0, 2.0, 3.0);
