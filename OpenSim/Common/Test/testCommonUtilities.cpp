/* -------------------------------------------------------------------------- *
 *                    OpenSim:  testCommonUtilities.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2024 Stanford University and the Authors                *
 * Author(s): Alexander Beattie                                               *
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

#include <algorithm> // std::transform
#include <limits> // std::numeric_limits
#include <vector>
#include <numeric> // std::iota
#include <cmath> // std::fma

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
TEST_CASE("isUniform tests") {
    SECTION("Basic uniform spacing") {
        std::vector<double> vec1 = {1.0, 2.0, 3.0, 4.0, 5.0};
        auto result = isUniform(vec1);
        REQUIRE(result.first == true);
        REQUIRE_THAT(result.second, Catch::Matchers::WithinAbs(1.0, tol));

        std::vector<double> vec2 = {0.0, 1.0, 2.0, 3.0};
        result = isUniform(vec2);
        REQUIRE(result.first == true);
        REQUIRE_THAT(result.second, Catch::Matchers::WithinAbs(1.0, tol));
    }

    SECTION("Non-uniform spacing") {
        std::vector<double> vec2 = {0.0, 0.5, 0.11, 0.16, 0.19, 0.24};
        auto result = isUniform(vec2);
        
        REQUIRE(result.first == false); // Should not be uniformly spaced
        // Verify that the second value returned is the minimum step size found
        REQUIRE_THAT(result.second, Catch::Matchers::WithinAbs(0.03, tol));

        std::vector<double> vec3 = {1.0, 2.0, 3.5, 4.0};
        result = isUniform(vec3);
        REQUIRE(result.first == false);
        REQUIRE_THAT(result.second, Catch::Matchers::WithinAbs(0.5, tol));

        std::vector<double> vec4 = {1.0, 2.0, 3.0, 4.0, 5.1};
        result = isUniform(vec4);
        REQUIRE(result.first == false);
        REQUIRE_THAT(result.second, Catch::Matchers::WithinAbs(1.0, tol));
    }

    SECTION("Non-uniform spacing with increase decreasing and negative values") {
        // Smallest element is 42.7 - 22.9 = 19.8
        std::vector<double> vec = {100.1, -27.2, 357.2, 0.16, 22.9, 42.7};
        auto result = isUniform(vec);
        
        REQUIRE(result.first == false); // Should not be uniformly spaced
        // Verify that the second value returned is the minimum step size found
        REQUIRE_THAT(result.second, Catch::Matchers::WithinAbs(19.8, tol));
    }

    SECTION("Uniform spacing with machine epsilon spacing") {
        double epsilon = std::numeric_limits<double>::epsilon();
        std::vector<double> vec5 = {1.0, 1.0 + epsilon, 1.0 + 2 * epsilon, 1.0 + 3 * epsilon};
        auto result = isUniform(vec5);
        REQUIRE(result.first == true);
        REQUIRE_THAT(result.second, Catch::Matchers::WithinAbs(epsilon, tol));
    }
    
    SECTION("Uniform spacing with floating-point inaccuracies") {
        {
            std::vector<double> vec = {678.0599999999999, 678.0700000000001, 678.08};
            auto result = isUniform(vec);
            const double maxElement = std::max(std::abs(vec.front()), std::abs(vec.back()));
            REQUIRE(result.first == true);
            INFO("Spacing Value: " << result.second );
            REQUIRE_THAT(result.second, Catch::Matchers::WithinAbs(0.01, maxElement*tol));
        }

        {
            std::vector<double> vec = {
                23.02499999999976, 23.04999999999976, 23.07499999999976, 23.09999999999976,
                23.12499999999975, 23.14999999999975, 23.17499999999975, 23.19999999999975,
                23.22499999999975, 23.24999999999975, 23.27499999999975, 23.29999999999974,
                23.32499999999974, 23.34999999999974, 23.37499999999974, 23.39999999999974,
                23.42499999999974, 23.44999999999974, 23.47499999999973, 23.49999999999973,
                23.52499999999973, 23.54999999999973, 23.57499999999973, 23.59999999999973,
                23.62499999999973, 23.64999999999973, 23.67499999999972, 23.69999999999972,
                23.72499999999972, 23.74999999999972, 23.77499999999972, 23.79999999999972,
                23.82499999999972, 23.84999999999971, 23.87499999999971, 23.89999999999971,
                23.92499999999971, 23.94999999999971, 23.97499999999971, 23.99999999999971,
                24.0249999999997, 24.0499999999997, 24.0749999999997, 24.0999999999997,
                24.1249999999997, 24.1499999999997, 24.1749999999997, 24.19999999999969,
                24.22499999999969, 24.24999999999969, 24.27499999999969
            };
            auto result = isUniform(vec);
            REQUIRE(result.first == true);
            // Should be sampling rate 1/40 = 0.025 
            REQUIRE_THAT(result.second, Catch::Matchers::WithinAbs(0.025, tol));
        }
    }

    SECTION("Edge cases") {
        std::vector<double> vec7 = {1.0, 1.0};
        auto result = isUniform(vec7);
        REQUIRE(result.first == true);
        REQUIRE_THAT(result.second, Catch::Matchers::WithinAbs(0.0, tol));

        std::vector<double> vec8 = {1.0};
        result = isUniform(vec8);
        REQUIRE(result.first == true);
        // There isn't any spacing for a 1 element vector
        REQUIRE(std::isnan(result.second));
    }
}

TEST_CASE("isUniform tests with createVectorLinspace(SimTK::Vector) and createVectorLinspaceInterval (std::vector) Combo") {
    {
        const int numEl = 100000;
        const double rate = 1.0 / 40.0; // Expected step size
        const double last_num = 2510.65; // Manually computed
        const double start = 10.675;

        const double step_size = (last_num - start) / (numEl - 1);
        REQUIRE_THAT(rate, Catch::Matchers::WithinAbs(step_size, tol));

        std::vector<double> values_std = createVectorLinspaceInterval(numEl, start, rate);
        SimTK::Vector values_simtk = createVectorLinspace(numEl, start, last_num);
        // Convert to std::vector
        std::vector<double> std_vector_from_simtk(values_simtk.size());
        std::copy_n(values_simtk.getContiguousScalarData(), numEl, std_vector_from_simtk.data());

        // Check that the last two elements are the same
        REQUIRE_THAT(values_std.back(), Catch::Matchers::WithinAbs(last_num, tol));
        REQUIRE_THAT(std_vector_from_simtk.back(), Catch::Matchers::WithinAbs(last_num, 1000*tol));
        
        // Check that the rest of the vector matches ==> they should
        for (size_t i = 0; i < values_std.size(); ++i) {
            REQUIRE_THAT(values_simtk[i], Catch::Matchers::WithinAbs(values_std[i], 1000*tol));
        }

        auto result_std = isUniform(values_std);
        REQUIRE(result_std.first == true);
        REQUIRE_THAT(result_std.second, Catch::Matchers::WithinAbs(rate, tol));
    
        auto result_simtk = isUniform(std_vector_from_simtk);
        REQUIRE(result_simtk.first == true);
        REQUIRE_THAT(result_simtk.second, Catch::Matchers::WithinAbs(rate, tol));           
    }

    {
        const int numEl = 5281;
        const double rate = 1.0 / 60.0; // Expected step size
        const double last_num = 80.362; // Manually computed
        const double start = -7.638;

        std::vector<double> values_std = createVectorLinspaceInterval(numEl, start, rate);
        SimTK::Vector values_simtk = createVectorLinspace(numEl, start, last_num);
        // Convert to std::vector
        std::vector<double> std_vector_from_simtk(values_simtk.size());
        std::copy_n(values_simtk.getContiguousScalarData(), numEl, std_vector_from_simtk.data());

        // Check that the last two elements are the same
        REQUIRE_THAT(values_std.back(), Catch::Matchers::WithinAbs(last_num, tol));
        REQUIRE_THAT(std_vector_from_simtk.back(), Catch::Matchers::WithinAbs(last_num, tol));
        
        // Check that the rest of the vector matches ==> they should
        for (size_t i = 0; i < values_std.size(); ++i) {
            REQUIRE_THAT(values_simtk[i], Catch::Matchers::WithinAbs(values_std[i], tol));
        }

        auto result_std = isUniform(values_std);
        REQUIRE(result_std.first == true);
        REQUIRE_THAT(result_std.second, Catch::Matchers::WithinAbs(rate, tol));
    
        auto result_simtk = isUniform(std_vector_from_simtk);
        REQUIRE(result_simtk.first == true);
        REQUIRE_THAT(result_simtk.second, Catch::Matchers::WithinAbs(rate, tol));
    }
}

TEST_CASE("detectDelimiter") {
    const std::vector<std::string> delimiters = {",", ";", "|", "\t", ":", " "};
    SECTION("Comma") {
        std::string input = "a,b,c,d";
        auto delim = detectDelimiter(input, delimiters);
        REQUIRE(delim == ",");
    }
    
    SECTION("Pipe") {
        std::string input = "a|b|c|d";
        auto delim = detectDelimiter(input, delimiters);
        REQUIRE(delim == "|");
    }
    
    SECTION("Tab") {
        std::string input = "a\tb\tc\td";
        auto delim = detectDelimiter(input, delimiters);
        REQUIRE(delim == "\t");
    }
    
    SECTION("Semicolon") {
        std::string input = "a;b;c;d";
        auto delim = detectDelimiter(input, delimiters);
        REQUIRE(delim == ";");
    }
    
    SECTION("Space") {
        std::string input = "a b c d";
        auto delim = detectDelimiter(input, delimiters);
        REQUIRE(delim == " ");
    }
    
    SECTION("No Valid Delimiter") {
        std::string input = "abcd";
        auto delim = detectDelimiter(input, delimiters);
        REQUIRE(delim == "");
    }
    
    SECTION("Delimiter Exclusion") {
        std::vector<std::string> small_delimiters = {",", ";"};
        std::string input = "a|b|c|d";
        auto delim = detectDelimiter(input, small_delimiters);
        REQUIRE(delim == "");
    }    
}

TEST_CASE("rotateMarkerTable tests") {
    SECTION("No rotation produces the same value") {
        const std::vector<double> time = {1.0, 2.0, 3.0};
        const std::vector<std::string> labels = {"marker1"};

        SimTK::Matrix_<SimTK::Vec3> markerData(3, 1); // 3 rows, 1 column

        markerData(0, 0) = SimTK::Vec3(2, 3, 4);
        markerData(1, 0) = SimTK::Vec3(4, 5, 6);
        markerData(2, 0) = SimTK::Vec3(7, 8, 9);

        TimeSeriesTableVec3 table(time, markerData, labels);

        const SimTK::Vec3 marker_rotations(0, 0, 0);
        const SimTK::Rotation R =
                SimTK::Rotation(SimTK::BodyOrSpaceType::SpaceRotationSequence,
                        marker_rotations[0], SimTK::XAxis, marker_rotations[1],
                        SimTK::YAxis, marker_rotations[2], SimTK::ZAxis);

        table.rotate(R);

        SimTK::Matrix_<SimTK::Vec3> resultData(3, 1); // 3 rows, 1 column

        resultData(0, 0) = SimTK::Vec3(2, 3, 4);
        resultData(1, 0) = SimTK::Vec3(4, 5, 6);
        resultData(2, 0) = SimTK::Vec3(7, 8, 9);
        const auto A = table.getMatrix();
        const auto B = resultData.getAsMatrixView();

        REQUIRE(A.nrow() == B.nrow());
        REQUIRE(A.ncol() == B.ncol());
        for (int i = 0; i < A.nrow(); ++i) {
            for (int j = 0; j < A.ncol(); ++j) {
                REQUIRE(A(i, j).isNumericallyEqual(B(i, j)));
            }
        }
    }
    SECTION("Valid rotation rotates the markers correctly") {
        const std::vector<double> time = {1.0, 2.0, 3.0};
        const std::vector<std::string> labels = {"marker1"};

        SimTK::Matrix_<SimTK::Vec3> markerData(3, 1); // 3 rows, 1 column

        markerData(0, 0) = SimTK::Vec3(2, 3, 4);
        markerData(1, 0) = SimTK::Vec3(-4, 5, -6);
        markerData(2, 0) = SimTK::Vec3(7, 8, 9);

        TimeSeriesTableVec3 table(time, markerData, labels);

        const SimTK::Vec3 marker_rotations(-SimTK::Pi / 2, SimTK::Pi / 2, 0);
        const SimTK::Rotation R =
                SimTK::Rotation(SimTK::BodyOrSpaceType::SpaceRotationSequence,
                        marker_rotations[0], SimTK::XAxis, marker_rotations[1],
                        SimTK::YAxis, marker_rotations[2], SimTK::ZAxis);

        table.rotate(R);

        SimTK::Matrix_<SimTK::Vec3> resultData(3, 1); // 3 rows, 1 column

        resultData(0, 0) = SimTK::Vec3(-3, 4, -2);
        resultData(1, 0) = SimTK::Vec3(-5, -6, 4);
        resultData(2, 0) = SimTK::Vec3(-8, 9, -7);
        const auto A = table.getMatrix();
        const auto B = resultData.getAsMatrixView();

        REQUIRE(A.nrow() == B.nrow());
        REQUIRE(A.ncol() == B.ncol());
        for (int i = 0; i < A.nrow(); ++i) {
            for (int j = 0; j < A.ncol(); ++j) {
                REQUIRE(A(i, j).isNumericallyEqual(B(i, j)));
            }
        }
    }
    SECTION("Rotate throws for unsupported table type") {
        const std::vector<double> time = {1.0, 2.0, 3.0};
        const std::vector<std::string> labels = {"col1"};

        SimTK::Matrix_<double> data(3, 1);
        data(0, 0) = 1.0;
        data(1, 0) = 2.0;
        data(2, 0) = 3.0;

        TimeSeriesTable table(time, data, labels); // TimeSeriesTable_<double>

        const SimTK::Rotation R(SimTK::BodyOrSpaceType::SpaceRotationSequence,
                0.1, SimTK::XAxis, 0.2, SimTK::YAxis, 0.3, SimTK::ZAxis);

        REQUIRE_THROWS_AS(table.rotate(R), OpenSim::Exception);
    }
}
