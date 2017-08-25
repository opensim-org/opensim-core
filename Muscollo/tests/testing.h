#ifndef MUSCOLLO_TESTING_H
#define MUSCOLLO_TESTING_H

#include <OpenSim/OpenSim.h>


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
    // For this problem, there's only 1 column in this table.
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

#endif // MUSCOLLO_TESTING_H
