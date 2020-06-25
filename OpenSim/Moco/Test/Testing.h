#ifndef OPENSIM_MOCO_TESTING_H
#define OPENSIM_MOCO_TESTING_H
/* -------------------------------------------------------------------------- *
 * OpenSim: testing.h                                                         *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Common/osimCommon.h>

// TODO: The more recent version of catch has issues with Approx() being
// ambiguous.
// TODO #include <OpenSim/Auxiliary/catch.hpp>
#include <Vendors/tropter/external/catch/catch.hpp>


// Helper functions for comparing vectors.
// ---------------------------------------
SimTK::Vector interp(const OpenSim::TimeSeriesTable& actualTable,
                     const OpenSim::TimeSeriesTable& expectedTable,
                     const std::string& expectedColumnLabel) {
    const auto& actualTime = actualTable.getIndependentColumn();
    // Interpolate the expected values based on `actual`'s time.
    const auto& expectedTime = expectedTable.getIndependentColumn();
    const auto& expectedCol =
            expectedTable.getDependentColumn(expectedColumnLabel);
    // Create a linear function for interpolation.
    OpenSim::PiecewiseLinearFunction expectedFunc(
        (int)expectedTable.getNumRows(), expectedTime.data(), &expectedCol[0]);
    SimTK::Vector expected((int)actualTable.getNumRows());
    for (int i = 0; i < (int)actualTable.getNumRows(); ++i) {
        const auto& time = actualTime[i];
        expected[i] = expectedFunc.calcValue(SimTK::Vector(1, time));
    }
    return expected;
};
// Compare each element.
void compare(const OpenSim::TimeSeriesTable& actualTable,
             const std::string& actualColumnLabel,
             const OpenSim::TimeSeriesTable& expectedTable,
             const std::string& expectedColumnLabel,
             double tol, bool verbose = false) {
    const auto& actual = actualTable.getDependentColumn(actualColumnLabel);
    SimTK::Vector expected = interp(actualTable, expectedTable,
                                    expectedColumnLabel);
    if (verbose) {
        std::cout << "Comparing " << expectedColumnLabel << std::endl;
        for (int i = 0; i < (int)actualTable.getNumRows(); ++i) {
            std::cout << actual[i] << " " << expected[i] << " "
                    << SimTK::isNumericallyEqual(actual[i], expected[i], tol)
                    << std::endl;
        }
    }
    SimTK_TEST_EQ_TOL(actual, expected, tol);
};
// A weaker check. Compute the root mean square of the error between the
// trajectory optimization and the inverse solver and ensure it is below a
// tolerance.
void rootMeanSquare(
        const OpenSim::TimeSeriesTable& actualTable,
        const std::string& actualColumnLabel,
        const OpenSim::TimeSeriesTable& expectedTable,
        const std::string& expectedColumnLabel,
        double tol, bool verbose = false) {
    const auto& actual = actualTable.getDependentColumn(actualColumnLabel);
    SimTK::Vector expected = interp(actualTable, expectedTable,
                                    expectedColumnLabel);
    const auto rmsError = (actual - expected).normRMS();
    if (verbose) {
        std::cout << "Comparing " << expectedColumnLabel << std::endl;
        for (int i = 0; i < actual.size(); ++i) {
            std::cout << actual[i] << " " << expected[i] << std::endl;
        }
        std::cout << "RMS error: " << rmsError << std::endl;
    }
    SimTK_TEST(rmsError < tol);
};

#define OpenSim_CATCH_MATRIX_INTERNAL(testtype, actual, expected, tol, toltype)\
do {                                                                         \
    const auto& a = actual;                                                  \
    const auto& b = expected;                                                \
    REQUIRE((a.nrow() == b.nrow()));                                         \
    REQUIRE((a.ncol() == b.ncol()));                                         \
    for (int ir = 0; ir < a.nrow(); ++ir) {                                  \
        for (int ic = 0; ic < a.ncol(); ++ic) {                              \
            INFO("(" << ir << "," << ic << "): " <<                          \
                    a.getElt(ir, ic) << " vs " << b.getElt(ir, ic));         \
            testtype((Approx(a.getElt(ir, ic)).toltype(tol)                  \
                    == b.getElt(ir, ic)));                                   \
        }                                                                    \
    }                                                                        \
} while (0)

#define OpenSim_REQUIRE_MATRIX(actual, expected)                             \
do {                                                                         \
    const auto& a = actual;                                                  \
    const auto& b = expected;                                                \
    using TypeA = std::remove_reference<decltype(a)>::type::E;               \
    using TypeB = std::remove_reference<decltype(b)>::type::E;               \
    const auto tol = SimTK::Test::defTol2<TypeA, TypeB>();                   \
    OpenSim_CATCH_MATRIX_INTERNAL(REQUIRE, actual, expected, tol, epsilon);  \
} while (0)

#define OpenSim_REQUIRE_MATRIX_TOL(actual, expected, tol)                    \
do {                                                                         \
    OpenSim_CATCH_MATRIX_INTERNAL(REQUIRE, actual, expected, tol, epsilon);  \
} while (0)

#define OpenSim_REQUIRE_MATRIX_ABSTOL(actual, expected, tol)                 \
do {                                                                         \
    OpenSim_CATCH_MATRIX_INTERNAL(REQUIRE, actual, expected, tol, margin);   \
} while (0)

#define OpenSim_CHECK_MATRIX(actual, expected)                               \
do {                                                                         \
    const auto& aa = actual;                                                 \
    const auto& bb = expected;                                               \
    using TypeA = std::remove_reference<decltype(aa)>::type::E;              \
    using TypeB = std::remove_reference<decltype(bb)>::type::E;              \
    const auto tol = SimTK::Test::defTol2<TypeA, TypeB>();                   \
    OpenSim_CATCH_MATRIX_INTERNAL(CHECK, actual, expected, tol, epsilon);    \
} while (0)

#define OpenSim_CHECK_MATRIX_TOL(actual, expected, tol)                      \
do {                                                                         \
    OpenSim_CATCH_MATRIX_INTERNAL(CHECK, actual, expected, tol, epsilon);    \
} while (0)

#define OpenSim_CHECK_MATRIX_ABSTOL(actual, expected, tol)                   \
do {                                                                         \
    OpenSim_CATCH_MATRIX_INTERNAL(CHECK, actual, expected, tol, margin);     \
} while (0)

#endif // OPENSIM_MOCO_TESTING_H
