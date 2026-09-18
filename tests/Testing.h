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

// #include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Common/PiecewiseLinearFunction.h>
#include <OpenSim/Common/PropertyObjArray.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/ActivationFiberLengthMuscle.h>

#include <fstream>
#include <regex>
#include <string>
#include <type_traits>

 /**
 * ASSERT_EQUAL is a general utility for comparing two values and throwing
 * an Exception with a caller defined message when values are not equivalent.
 * Note, ASSERT_EQUAL is typically used to verify that some found value matches
 * a given expected or standard value. If the expected value is NaN, 
 * ASSERT_EQUAL will NOT throw if the found value is also NaN. This is
 * particularly helpful for comparing motion capture data where missing data
 * are denoted by NaN values. If NaNs are not acceptable for your test, then
 * the expected value should not be NaN. In the case of floating point values
 * (or containers of floating points) a tolerance of the same value type is
 * required.
 */
template <typename T,
   typename std::enable_if<std::is_floating_point<T>::value>::type* = nullptr >
void ASSERT_EQUAL(T expected,
                  T found, 
                  T tolerance, 
                  std::string file = "", 
                  int line = -1, 
                  std::string message = "") {
    // if both values are NaN treat them as being equivalent for the
    // sake of comparing experimental data and results where NaNs are
    // possible
    if(SimTK::isNaN(found) && SimTK::isNaN(expected))
        return;
    if (found < expected - tolerance || found > expected + tolerance)
        throw OpenSim::Exception(message, file, line);
}

template <typename T,
    typename std::enable_if<std::is_integral<T>::value>::type* = nullptr >
    void ASSERT_EQUAL(T expected,
                      T found,
                      std::string file = "",
                      int line = -1,
                      std::string message = "") {
    if (found != expected)
        throw OpenSim::Exception(message, file, line);
}

template <typename T,
    typename std::enable_if<!std::is_arithmetic<T>::value>::type* = nullptr >
    void ASSERT_EQUAL(T expected,
                      T found,
                      T tolerance,
                      std::string file = "",
                      int line = -1,
                      std::string message = "") {
    // if both values are NaN treat them as equivalent 
    if (found.isNaN() && expected.isNaN() )
        return;
    if (found < expected - tolerance || found > expected + tolerance)
        throw OpenSim::Exception(message, file, line);
}

template<int M, typename ELT, int STRIDE>
void ASSERT_EQUAL(const SimTK::Vec<M, ELT, STRIDE>& vecA,
                  const SimTK::Vec<M, ELT, STRIDE>& vecB,
                  const std::string& file = "",
                  int line = -1,
                  const std::string& message = "") {
    try {
        // if both values are NaN treat them as being equivalent
        if (vecA.isNaN() && vecB.isNaN())
            return;
        SimTK_TEST_EQ(vecA, vecB);
    } catch(const SimTK::Exception::Assert&) {
        throw OpenSim::Exception(message, file, line);
    }
}
template<int M, typename ELT, int STRIDE>
void ASSERT_EQUAL(const SimTK::Vec<M, ELT, STRIDE>& vecA,
                  const SimTK::Vec<M, ELT, STRIDE>& vecB,
                  double tolerance,
                  const std::string& file = "",
                  int line = -1,
                  const std::string& message = "") {
    try {
        // if both values are NaN treat them as being equivalent
        if (vecA.isNaN() && vecB.isNaN())
            return;
        SimTK_TEST_EQ_TOL(vecA, vecB, tolerance);
    } catch(const SimTK::Exception::Assert&) {
        throw OpenSim::Exception(message, file, line);
    }
}

template<typename Container, typename T>
void ASSERT_EQUAL( const Container& vecA,
                   const Container& vecB,
                    T tolerance,
                    std::string file = "",
                    int line = -1,
                    std::string message = "") {

    if (vecA.size() != vecB.size()) {
        throw OpenSim::Exception(message, file, line);
    }
    else {
        for (int i = 0; i < (int)vecA.size(); ++i) {
            // if both values are NaN treat them as being equivalent
            if ( SimTK::isNaN(vecA[i]) && SimTK::isNaN(vecB[i]) )
                continue;
            if (vecA[i] < vecB[i] - tolerance || vecA[i] > vecB[i] + tolerance) {
                throw OpenSim::Exception(message, file, line);
            }
        }
    }
}

inline void ASSERT(bool cond, 
                   std::string file="", 
                   int line=-1, 
                   std::string message="Exception") {
    if (!cond) throw OpenSim::Exception(message, file, line);
}
/**
 * Check this storage object against a standard storage object using the
 * specified tolerances. If RMS error for any column is outside the
 * tolerance, throw an Exception.
 */
inline void CHECK_STORAGE_AGAINST_STANDARD(const OpenSim::Storage& result, 
                                    const OpenSim::Storage& standard, 
                                    const std::vector<double>& tolerances, 
                                    const std::string& testFile, 
                                    const int testFileLine, 
                                    const std::string& errorMessage)
{
    std::vector<std::string> columnsUsed;
    std::vector<double> comparisons;
    result.compareWithStandard(standard, columnsUsed, comparisons);

    size_t ncolumns = columnsUsed.size();

    ASSERT(ncolumns > 0, testFile, testFileLine, 
           errorMessage + "- no common columns to compare!");

    for (size_t i = 0; i < ncolumns; ++i) {
        std::cout << "column:    " << columnsUsed[i] << std::endl;
        std::cout << "RMS error: " << comparisons[i] << std::endl;
        std::cout << "tolerance: " << tolerances[i] << std::endl << std::endl;
        ASSERT(comparisons[i] < tolerances[i], testFile, testFileLine, 
               errorMessage);
    }
}

// Informed by googletest.
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

// MESSAGE is a std::string; the assert passes if the expected exception is
// thrown and the exception's message contains MESSAGE.
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

inline OpenSim::Object* randomize(OpenSim::Object* obj)
{
    using namespace OpenSim;
    using namespace std;

    if (obj==nullptr) return 0; // maybe empty tag
    std::stringstream stream;
    stream << rand();
    obj->setName(obj->getConcreteClassName()+stream.str());
    // Cycle thru properties and based on type, populate with a random valid 
    // value.
     for (int p=0; p < obj->getNumProperties(); ++p) {
        AbstractProperty& ap = obj->updPropertyByIndex(p); 
        //cout << ap.getName() << "=" << ap.toString() << endl;
        // Check return values from Property API for debugging purposes
        bool isList = ap.isListProperty();
        // bool t2 = ap.isObjectProperty();
        // bool t3 = ap.isOneObjectProperty();
        // bool t4 = ap.isOneValueProperty();
        string ts = ap.getTypeName();
        //cout << ts << endl;
        if (ap.isOptionalProperty()) 
            continue;
        if (ts == "bool"&& !isList) 
            ap.updValue<bool>() = (rand() % 2 == 0);
        else if (ts == "integer"&& !isList) 
            ap.updValue<int>() = rand();
        else if (ts == "double" && !isList) 
            ap.updValue<double>() = (double)rand()/RAND_MAX;
        else if (ts == "Vec3" && !isList) {
            Property<SimTK::Vec3>& prop = Property<SimTK::Vec3>::updAs(ap);
            prop = SimTK::Vec3(abs(rand()), abs(rand()), abs(rand()));
        } else if (ts == "Vec6" && !isList) {
            // Only property that uses a Vec6 is the inertia property
            // Might as well select valid inertias for the purpose of testing
            Property<SimTK::Vec6>& prop = Property<SimTK::Vec6>::updAs(ap);
            double Ixx = abs(rand());
            double Ixy = 0.01*Ixx;
            prop = SimTK::Vec6(Ixx, Ixx, Ixx, Ixy, Ixy, Ixy);
        } else if (ts == "string") {
            // We cannot use an arbitrary string for ExpressionBasedBushingForce 
            // properties since they must contain specific variable names (e.g.,
            // "theta_x", "delta_x", etc.). 
            if (obj->getConcreteClassName() != "ExpressionBasedBushingForce") {
                string base("ABCXYZ");
                if (isList) {
                    stringstream val;
                    val << base << "_" << ap.size();
                    ap.appendValue<string>(val.str());
                } else {
                    ap.updValue<string>() = base;
                }
            }
        } else if (ts == "double" && isList && ap.getMaxListSize() < 20) {
            for (int i=0; i< ap.getMaxListSize(); ++i)
                ap.updValue<double>(i) = (double) rand() / RAND_MAX;
        } else if (ts == "Function") {
            //FunctionSet's objects getTypeName() returns "Function"
            //which is wrong! This is a HACK to test that we aren't
            //treating the PropertyObjArray<Function> as a Function.
            PropertyObjArray<Function>* propObjArray =
                dynamic_cast<PropertyObjArray<Function>*>(&ap);
            if (propObjArray){
                if (propObjArray->size()){
                    randomize(&(propObjArray->updValueAsObject(0)));
                }
            }
            else{
                Property<Function>& prop = Property<Function>::updAs(ap);
                LinearFunction f;
                randomize(&f);
                prop = f;
            }
            
        } else if (ap.isObjectProperty() && !isList) {
            randomize(&ap.updValueAsObject(0));
            if (ap.isUnnamedProperty())
                ap.updValueAsObject(0).setName("");
            ap.setValueIsDefault(false);
        } else {
            //cerr << "Unrecognized Property:"<< ap.getName()<< ":" 
            //     << ap.toString() << endl;
        }
     }
     return obj;
}

// Change version number of the file to 1 so that Storage can read it.
// Storage can only read files with version <= 1. Returns 'true' if
// version number was changed. Returns 'false' if no change.
// This function can be removed when Storage class is removed.
inline bool revertToVersionNumber1(const std::string& filenameOld,
                                   const std::string& filenameNew) {
    std::regex versionline{ R"([ \t]*version[ \t]*=[ \t]*2[ \t]*)" };
    std::ifstream fileOld{ filenameOld };
    std::ofstream fileNew{ filenameNew };
    std::string line{};
    bool changedVersion{false};
    while (std::getline(fileOld, line)) {
        if (std::regex_match(line, versionline)) {
            fileNew << "version=1\n";
            changedVersion = true;
        } else
            fileNew << line << "\n";
    }
    return changedVersion;
}

/** A debugging utility for investigating muscle equilibrium failures.
    For a given muscle at a given state report how the muscle fiber and
    tendon force varies with fiber-length. Also, report the difference, 
    which represents the function that the muscle equilibrium solver is
    trying to find a root (zero) for. The intended use is to invoke this
    method when the muscle fails to compute the equilibrium fiber-length,
    so that one can plot the equilibrium force error vs. fiber-length
    to help diagnose the cause of the failure. The force-velocity 
    multiplier is assumed to be 1.0 (e.g. static fiber) unless otherwise
    specified.*/
template <typename T = OpenSim::ActivationFiberLengthMuscle>
void reportTendonAndFiberForcesAcrossFiberLengths(const T& muscle,
    const SimTK::State& state, const double fiberVelocityMultiplier = 1.0)
{
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

//==========================================================================
// Table comparison helpers (formerly OpenSim/Moco/tests/Testing.h)
//==========================================================================

// Helper functions for comparing vectors.
// ---------------------------------------
inline SimTK::Vector interp(const OpenSim::TimeSeriesTable& actualTable,
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
inline void compare(const OpenSim::TimeSeriesTable& actualTable,
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
inline void rootMeanSquare(
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
