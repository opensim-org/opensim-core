#ifndef OPENSIM_TESTING_H_
#define OPENSIM_TESTING_H_
/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Testing.h                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2026 Stanford University and the Authors                *
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

#include <OpenSim/Common/Assertion.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Common/PiecewiseLinearFunction.h>
#include <OpenSim/Common/PropertyObjArray.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/ActivationFiberLengthMuscle.h>

#include <fstream>
#include <iostream>
#include <regex>
#include <string>
#include <type_traits>

namespace OpenSim::Testing {

/** Floating-point values are equal if within a tolerance of the same type. */
template <typename T,
    typename std::enable_if<std::is_floating_point<T>::value>::type* = nullptr >
bool isEqual(T expected, T found, T tolerance) {

    if (SimTK::isNaN(found) && SimTK::isNaN(expected)) {
        return true;
    }

    return !(found < expected - tolerance || found > expected + tolerance);
}

/** Integral values are equal if exactly equal. */
template <typename T,
    typename std::enable_if<std::is_integral<T>::value>::type* = nullptr >
bool isEqual(T expected, T found) {

    return found == expected;
}

/** Non-arithmetic values are equal if within a tolerance of the same type. */
template <typename T,
    typename std::enable_if<!std::is_arithmetic<T>::value>::type* = nullptr >
bool isEqual(T expected, T found, T tolerance) {

    if (found.isNaN() && expected.isNaN()) {
        return true;
    }

    return !(found < expected - tolerance || found > expected + tolerance);
}

/** Two SimTK::Vecs are equal if equal elementwise at the default tolerance. */
template<int M, typename ELT, int STRIDE>
bool isEqual(const SimTK::Vec<M, ELT, STRIDE>& vecA,
             const SimTK::Vec<M, ELT, STRIDE>& vecB) {

    if (vecA.isNaN() && vecB.isNaN()) {
        return true;
    }

    return SimTK::Test::numericallyEqual(vecA, vecB, 1);
}

/** Two SimTK::Vecs are equal if equal elementwise within a tolerance. */
template<int M, typename ELT, int STRIDE>
bool isEqual(const SimTK::Vec<M, ELT, STRIDE>& vecA,
             const SimTK::Vec<M, ELT, STRIDE>& vecB,
             double tolerance) {

    if (vecA.isNaN() && vecB.isNaN()) {
        return true;
    }

    return SimTK::Test::numericallyEqual(vecA, vecB, 1, tolerance);
}

/**
 * Two containers are equal if the same size and equal elementwise within a
 * tolerance.
 */
template<typename Container, typename T>
bool isEqual(const Container& vecA, const Container& vecB, T tolerance) {

    if (vecA.size() != vecB.size()) {
        return false;
    }

    for (int i = 0; i < (int)vecA.size(); ++i) {
        if (SimTK::isNaN(vecA[i]) && SimTK::isNaN(vecB[i])) {
            continue;
        }

        if (vecA[i] < vecB[i] - tolerance || vecA[i] > vecB[i] + tolerance) {
            return false;
        }
    }

    return true;
}

/**
 * Check a Storage against a standard Storage using the specified per-column
 * tolerances. Fails if there are no common columns, or if the RMS error for
 * any column is outside its tolerance.
 */
void checkStorageAgainstStandard(const OpenSim::Storage& result,
        const OpenSim::Storage& standard,
        const std::vector<double>& tolerances);

/**
 * Randomize the property values of an `OpenSim::Object`.
 */
OpenSim::Object* randomize(OpenSim::Object* obj);

/**
 * Change version number of the file to 1 so that Storage can read it.
 * Storage can only read files with version <= 1. Returns 'true' if
 * version number was changed. Returns 'false' if no change.
 * This function can be removed when Storage class is removed.
 */
bool revertToVersionNumber1(const std::string& filenameOld,
        const std::string& filenameNew);

/** Helper function for comparing vectors. */
SimTK::Vector interp(const OpenSim::TimeSeriesTable& actualTable,
        const OpenSim::TimeSeriesTable& expectedTable,
        const std::string& expectedColumnLabel);

/** Compare each element. */
void compare(const OpenSim::TimeSeriesTable& actualTable,
        const std::string& actualColumnLabel,
        const OpenSim::TimeSeriesTable& expectedTable,
        const std::string& expectedColumnLabel,
        double tol, bool verbose = false);

/**
 * A weaker check. Compute the root mean square of the error between the
 * trajectory optimization and the inverse solver and ensure it is below a
 * tolerance.
 */
void rootMeanSquare(const OpenSim::TimeSeriesTable& actualTable,
        const std::string& actualColumnLabel,
        const OpenSim::TimeSeriesTable& expectedTable,
        const std::string& expectedColumnLabel,
        double tol, bool verbose = false);

/**
 * A debugging utility for investigating muscle equilibrium failures. For a
 * given muscle at a given state report how the muscle fiber and tendon force
 * varies with fiber-length. Also, report the difference, which represents the
 * function that the muscle equilibrium solver is trying to find a root (zero)
 * for. The intended use is to invoke this method when the muscle fails to
 * compute the equilibrium fiber-length, so that one can plot the equilibrium
 * force error vs. fiber-length to help diagnose the cause of the failure. The
 * force-velocity multiplier is assumed to be 1.0 (e.g. static fiber) unless
 * otherwise specified.
 */
template <typename T = OpenSim::ActivationFiberLengthMuscle>
void reportTendonAndFiberForcesAcrossFiberLengths(const T& muscle,
    const SimTK::State& state, const double fiberVelocityMultiplier = 1.0) {

    // should only be using this utility for equilibrium muscles
    // with a compliant tendon
    OPENSIM_ASSERT(!muscle.get_ignore_tendon_compliance());

    SimTK::State s = state;

    OpenSim::DataTable_<double, double> forcesVsFiberLengthTable;
    std::vector<std::string> labels{ "fiber_length", "pathLength",
        "tendon_force", "fiber_force", "activation", "activeFiberForce",
        "passiveFiberForce", "equilibriumError" };
    forcesVsFiberLengthTable.setColumnLabels(labels);

    // Constants
    const int N = 100;
    const int nc = int(labels.size());

    const double maxFiberLength = 2.0*muscle.getOptimalFiberLength();
    const double minFiberLength = muscle.getMinimumFiberLength();
    const double dl = (maxFiberLength - minFiberLength) / N;
    const double fiso = muscle.getMaxIsometricForce();

    // Variables
    double fiberLength = SimTK::NaN;
    // double vmt = SimTK::NaN;
    double tendonForce = SimTK::NaN;
    double activeFiberForce = SimTK::NaN;
    double passiveFiberForce = SimTK::NaN;
    double cosphi = SimTK::NaN;
    double flm = SimTK::NaN;
    double a = SimTK::NaN;

    SimTK::RowVector row(nc, SimTK::NaN);
    for (int i = 0; i <= N; ++i) {
        fiberLength = minFiberLength + i*dl;
        s.setTime(fiberLength);
        muscle.setFiberLength(s, fiberLength);
        muscle.getModel().realizeDynamics(s);

        // vmt = muscle.getSpeed(s);

        tendonForce = muscle.getTendonForce(s);

        a = muscle.getActivation(s);

        flm = muscle.getActiveForceLengthMultiplier(s);
        cosphi = muscle.getCosPennationAngle(s);

        activeFiberForce = a*fiso*flm*fiberVelocityMultiplier;

        passiveFiberForce = muscle.getPassiveFiberForce(s);

        row[0] = fiberLength; // muscle.getFiberLength(s);
        row[1] = muscle.getLength(s);
        row[2] = tendonForce;
        row[3] = (activeFiberForce + passiveFiberForce)*cosphi;
        row[4] = muscle.getActivation(s);
        row[5] = activeFiberForce;
        row[6] = passiveFiberForce;
        row[7] = row[3] - row[2];

        forcesVsFiberLengthTable.appendRow(s.getTime(), row);
    }

    std::string fileName = "forcesVsFiberLength_"
        + std::to_string(a) + ".sto";

    OpenSim::STOFileAdapter::write(forcesVsFiberLengthTable, fileName);
}

} // namespace OpenSim::Testing

// Assertion macros
// ----------------

#define ASSERT_EQUAL(...) \
    OPENSIM_ASSERT_ALWAYS(OpenSim::Testing::isEqual(__VA_ARGS__))

inline void ASSERT(bool cond, 
                   std::string file="", 
                   int line=-1, 
                   std::string message="Exception") {
    if (!cond) throw OpenSim::Exception(message, file, line);
}
#define ASSERT_THROW(EXPECTED_EXCEPTION, STATEMENT) \
do { \
    bool caughtExpectedException = false; \
    try { \
        STATEMENT; \
    } \
    catch (EXPECTED_EXCEPTION const&) { \
        caughtExpectedException = true; \
    } \
    catch (...) { \
        throw OpenSim::Exception("TESTING: Expected exception " \
            #EXPECTED_EXCEPTION " but caught different exception."); \
    } \
    if (!caughtExpectedException) { \
        throw OpenSim::Exception("TESTING: Expected exception " \
            #EXPECTED_EXCEPTION " but no exception thrown."); \
    } \
} while(false)

/**
 * MESSAGE is a std::string; the assert passes if the expected exception is
 * thrown and the exception's message contains MESSAGE.
 */
#define ASSERT_THROW_MSG(EXPECTED_EXCEPTION, MESSAGE, STATEMENT) \
do { \
    bool caughtExpectedException = false; \
    try { \
        STATEMENT; \
    } \
    catch (EXPECTED_EXCEPTION const& exc) { \
        caughtExpectedException = true; \
        std::string actualMessage = std::string(exc.what()); \
        if (actualMessage.find(MESSAGE) == std::string::npos) { \
            throw OpenSim::Exception("TESTING: Caught expected exception " \
                    "type but message did not contain desired string.\n"  \
                    "Actual message:\n" + actualMessage + "\n" \
                    "Desired substring:\n" + MESSAGE); \
        } \
    } \
    catch (...) { \
        throw OpenSim::Exception("TESTING: Expected exception " \
            #EXPECTED_EXCEPTION " but caught different exception."); \
    } \
    if (!caughtExpectedException) { \
        throw OpenSim::Exception("TESTING: Expected exception " \
            #EXPECTED_EXCEPTION " but no exception thrown."); \
    } \
} while(false)

// Catch2 helper macros
// --------------------

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
            testtype((Catch::Approx(a.getElt(ir, ic)).toltype(tol)           \
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

#endif // OPENSIM_TESTING_H_
