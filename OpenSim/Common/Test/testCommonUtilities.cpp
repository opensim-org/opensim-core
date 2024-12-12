/* --------------------------------------------------------------------------*
*                         OpenSim:  testCommonUtilities.cpp                  *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2024 Stanford University and the Authors                *
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

#include <OpenSim/Common/CommonUtilities.h>

#include <algorithm> // for std::equal
#include <limits>
#include <vector>
#include <numeric>
#include <algorithm>
#include <cmath> // for std::fma

#include <catch2/catch_all.hpp>

using namespace OpenSim;

constexpr double tol = std::numeric_limits<double>::epsilon() * 10;

TEST_CASE("createVectorLinspaceInterval produces correct output for integers", "[createVectorLinspaceInterval]") {
    auto result = createVectorLinspaceInterval<int>(5, 0, 2);
    std::vector<int> expected = {0, 2, 4, 6, 8};
    REQUIRE_THAT(result, Catch::Matchers::Equals(expected));
}

TEST_CASE("createVectorLinspaceInterval produces correct output for floats", "[createVectorLinspaceInterval]") {
    auto result = createVectorLinspaceInterval<float>(5, 1.0f, 0.5f);
    std::vector<float> expected = {1.0f, 1.5f, 2.0f, 2.5f, 3.0f};
    REQUIRE_THAT(result, Catch::Matchers::Equals(expected));
}

TEST_CASE("createVectorLinspaceInterval handles negative step size", "[createVectorLinspaceInterval]") {
    auto result = createVectorLinspaceInterval<double>(5, 10.0, -2.0);
    std::vector<double> expected = {10.0, 8.0, 6.0, 4.0, 2.0};
    REQUIRE_THAT(result, Catch::Matchers::Equals(expected));
}

TEST_CASE("createVectorLinspaceInterval handles zero length", "[createVectorLinspaceInterval]") {
    auto result = createVectorLinspaceInterval<double>(0, 0.0, 1.0);
    std::vector<double> expected = {};
    REQUIRE_THAT(result, Catch::Matchers::Equals(expected));
}

TEST_CASE("createVectorLinspaceInterval handles large step size", "[createVectorLinspaceInterval]") {
    auto result = createVectorLinspaceInterval<int>(5, 0, 100);
    std::vector<int> expected = {0, 100, 200, 300, 400};
    REQUIRE_THAT(result, Catch::Matchers::Equals(expected));
}

TEST_CASE("createVectorLinspaceInterval handles large length", "[createVectorLinspaceInterval]") {
    auto result = createVectorLinspaceInterval<double>(100, 0.0, 0.1);
    std::vector<double> expected(100);
    std::iota(expected.begin(), expected.end(), 0.0);
    std::transform(expected.begin(), expected.end(), expected.begin(),
                   [](double value) { return value * 0.1; });
    REQUIRE_THAT(result, Catch::Matchers::Equals(expected));
}

TEST_CASE("createVectorLinspaceInterval produces correct output for small spacing with 10 elements", "[createVectorLinspaceInterval]") {
    auto result = createVectorLinspaceInterval<double>(10, 0.0, 0.001);
    std::vector<double> expected = {0.0, 0.001, 0.002, 0.003, 0.004, 0.005, 0.006, 0.007, 0.008, 0.009};
    // Check that the sizes of the vectors are the same
    REQUIRE(result.size() == expected.size());

    // Check that each result is within the specified tolerance of the expected value
    for (size_t i = 0; i < result.size(); ++i) {
        REQUIRE_THAT(result[i], Catch::Matchers::WithinAbs(expected[i], tol));
    }
}

TEST_CASE("createVectorLinspaceInterval produces correct output for small spacing", "[createVectorLinspaceInterval]") {
    std::vector<double> expected(100);
    std::iota(expected.begin(), expected.end(), 0.0);
    std::transform(expected.begin(), expected.end(), expected.begin(),
                   [](double value) { return value * 0.001; });
    auto result = createVectorLinspaceInterval<double>(100, 0.0, 0.001);
    // Check that the sizes of the vectors are the same
    REQUIRE(result.size() == expected.size());

    // Check that each result is within the specified tolerance of the expected value
    for (size_t i = 0; i < result.size(); ++i) {
        REQUIRE_THAT(result[i], Catch::Matchers::WithinAbs(expected[i], tol));
    }
}
