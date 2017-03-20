#ifndef TOMU_TESTING_H
#define TOMU_TESTING_H

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
    OpenSim::PiecewiseLinearFunction expectedFunc(expectedTable.getNumRows(),
                                                  expectedTime.data(),
                                                  &expectedCol[0]);
    SimTK::Vector expected(actualTable.getNumRows());
    for (size_t i = 0; i < actualTable.getNumRows(); ++i) {
        const auto& time = actualTime[i];
        expected[i] = expectedFunc.calcValue(SimTK::Vector(1, time));
    }
    return expected;
};
// Compare each element.
void compare(const OpenSim::TimeSeriesTable& actualTable,
             const OpenSim::TimeSeriesTable& expectedTable,
             const std::string& expectedColumnLabel,
             double tol) {
    // For this problem, there's only 1 column in this table.
    const auto& actual = actualTable.getDependentColumnAtIndex(0);
    SimTK::Vector expected = interp(actualTable, expectedTable,
                                    expectedColumnLabel);
    //for (size_t i = 0; i < actualTable.getNumRows(); ++i) {
    //    std::cout << "DEBUG " << actual[i] << " " << expected[i]
    //            << std::endl;
    //}
    SimTK_TEST_EQ_TOL(actual, expected, tol);
};
// A weaker check. Compute the root mean square of the error between the
// trajectory optimization and the inverse solver and ensure it is below a
// tolerance.
void rootMeanSquare(
        const OpenSim::TimeSeriesTable& actualTable,
        const OpenSim::TimeSeriesTable& expectedTable,
        const std::string& expectedColumnLabel,
        double tol) {
    const auto& actual = actualTable.getDependentColumnAtIndex(0);
    SimTK::Vector expected = interp(actualTable, expectedTable,
                                    expectedColumnLabel);
    SimTK_TEST((actual - expected).normRMS() < tol);
};

#endif // TOMU_TESTING_H
