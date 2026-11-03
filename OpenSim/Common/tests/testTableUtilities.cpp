/* -------------------------------------------------------------------------- *
 *                     OpenSim:  testTableUtilities.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2026 Stanford University and the Authors                *
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

#include <OpenSim/Common/TableUtilities.h>

#include <tests/Testing.h>

#include <catch2/catch_all.hpp>

#include <cmath>
#include <format>
#include <vector>

using namespace OpenSim;

namespace {

// Creates a table whose columns are sine curves with differing amplitudes,
// offsets, frequencies, and phases.
TimeSeriesTable createSineTable() {
    const int numColumns = 5;
    const int numRows = 500;

    std::vector<double> times;
    times.reserve(numRows);
    SimTK::Matrix data(numRows, numColumns, 0.0);
    for (int irow = 0; irow < numRows; ++irow) {
        const double time = 0.01 * irow;
        times.push_back(time);
        for (int icol = 0; icol < numColumns; ++icol) {
            const double amplitude = 0.5 * (icol + 1);
            const double frequency = 0.2 * (icol + 1);
            const double phase = 0.3 * icol;
            const double offset = 0.25 * icol;
            data(irow, icol) =
                    amplitude * std::sin(
                            2.0 * SimTK::Pi * frequency * time + phase) +
                    offset;
        }
    }

    std::vector<std::string> labels;
    labels.reserve(numColumns);
    for (int icol = 0; icol < numColumns; ++icol) {
        labels.push_back(std::format("col_{}", icol));
    }

    TimeSeriesTable table(times, data, labels);
    table.addTableMetaData<std::string>("inDegrees", "no");
    return table;
}

} // namespace

TEST_CASE("findNonFiniteElements") {
    TimeSeriesTable table = createSineTable();
    const int numRows = (int)table.getNumRows();
    const std::vector<std::pair<int, int>> expected = {
        {3, 0}, {5, 1}, {17, 2}, {40, 1}, {numRows - 1, 3}};

    SECTION("A table of finite values has no non-finite elements") {
        CHECK(TableUtilities::findNonFiniteElements(table).empty());
    }

    SECTION("NaN values are found at the expected locations") {
        for (const auto& [irow, icol] : expected) {
            table.updDependentColumnAtIndex(icol)[irow] = SimTK::NaN;
        }
        CHECK(TableUtilities::findNonFiniteElements(table) == expected);
    }

    SECTION("Infinite values are found at the expected locations") {
        for (const auto& [irow, icol] : expected) {
            table.updDependentColumnAtIndex(icol)[irow] =
                    irow % 2 ? SimTK::Infinity : -SimTK::Infinity;
        }
        CHECK(TableUtilities::findNonFiniteElements(table) == expected);
    }
}

TEST_CASE("hampel") {
    const double numStandardDeviations = 5.0;
    TimeSeriesTable table = createSineTable();;
    const int numRows = (int)table.getNumRows();
    const int numAdjacentSamples = 3;
    const double outlierOffset = 100.0;

    SECTION("A table of smooth curves has no outliers") {
        CHECK(TableUtilities::hampel(
                table, numAdjacentSamples, numStandardDeviations).empty());
    }

    SECTION("Outliers are found at the expected locations") {
        const std::vector<std::pair<int, int>> expected = {
                {2, 3}, {7, 0}, {100, 1}, {numRows - 3, 3}};
        for (const auto& [irow, icol] : expected) {
            table.updDependentColumnAtIndex(icol)[irow] += outlierOffset;
        }
        CHECK(TableUtilities::hampel(table, numAdjacentSamples,
                      numStandardDeviations) == expected);
    }

    SECTION("Non-finite elements are reported and excluded from the window") {
        table.updDependentColumnAtIndex(0)[4] = SimTK::NaN;
        table.updDependentColumnAtIndex(0)[9] = SimTK::Infinity;
        table.updDependentColumnAtIndex(0)[6] += outlierOffset;

        const std::vector<std::pair<int, int>> expected = {
                {4, 0}, {6, 0}, {9, 0}};
        CHECK(TableUtilities::hampel(table, numAdjacentSamples,
                      numStandardDeviations) == expected);
    }

    SECTION("A window that is mostly non-finite still screens what is left") {
        std::vector<std::pair<int, int>> expected;
        for (int irow = 1; irow < 25; irow += 2) {
            table.updDependentColumnAtIndex(0)[irow] = SimTK::Infinity;
            expected.emplace_back(irow, 0);
        }
        table.updDependentColumnAtIndex(0)[22] += outlierOffset;
        expected.emplace_back(22, 0);
        std::sort(expected.begin(), expected.end());

        CHECK(TableUtilities::hampel(table, numAdjacentSamples,
                      numStandardDeviations) == expected);
    }

    SECTION("A constant column has no outliers") {
        for (int irow = 0; irow < numRows; ++irow) {
            table.updDependentColumnAtIndex(1)[irow] = 0.375;
        }
        CHECK(TableUtilities::hampel(table, numAdjacentSamples,
                      numStandardDeviations).empty());
    }

    SECTION("An element differing from a constant window is an outlier") {
        for (int irow = 0; irow < numRows; ++irow) {
            table.updDependentColumnAtIndex(1)[irow] = 0.375;
        }
        table.updDependentColumnAtIndex(1)[200] = 0.376;

        const std::vector<std::pair<int, int>> expected = {{200, 1}};
        CHECK(TableUtilities::hampel(table, numAdjacentSamples,
                      numStandardDeviations) == expected);
    }

    SECTION("An element is screened against the scaled MAD of its window") {
        // The column [0, 0, 0, -1, -1, 1, 1, -2, 2, 2] has a median of 0 and a
        // MAD of 1.0, and any added value with magnitude greater than 2 will
        // not change the median or MAD. Test that adding a value just above the
        // threshold is reported as an outlier, and adding a value just below
        // the threshold is not reported.
        const double madScaleFactor = 1.482602218505602;
        const double threshold = numStandardDeviations * madScaleFactor;
        auto screen = [&](double value) {
            const std::vector<double> values = {0.0, 0.0, 0.0, -1.0, -1.0, 1.0,
                    1.0, -2.0, 2.0, 2.0, value};
            const int numValues = (int)values.size();
            SimTK::Matrix column(numValues, 1, 0.0);
            for (int irow = 0; irow < numValues; ++irow) {
                column(irow, 0) = values[irow];
            }
            return TableUtilities::hampel(
                    column, numValues - 1, numStandardDeviations);
        };

        // Test a value just above the threshold.
        // std::nextafter(threshold, value) returns the next representable value
        // (one ULP) after 'threshold' in the direction of 'value'.
        const std::vector<std::pair<int, int>> expected = {{10, 0}};
        CHECK(screen(std::nextafter(threshold, SimTK::Infinity)) == expected);
        // Test a value just below the threshold.
        CHECK(screen(std::nextafter(threshold, 0.0)).empty());
    }

    SECTION("Nearly constant columns reject nothing when a floor is given") {
        // A window of nearly identical values has a median absolute deviation
        // near zero, so without a floor a difference of one unit in the last
        // place is reported as an outlier.
        const double value = 0.375;
        for (int icol = 0; icol < 3; ++icol) {
            for (int irow = 0; irow < numRows; ++irow) {
                table.updDependentColumnAtIndex(icol)[irow] =
                        (irow % 3 == icol) ? std::nextafter(value, 1.0)
                                           : value;
            }
        }
        // With no floor the perturbed elements are reported.
        CHECK(!TableUtilities::hampel(table, numAdjacentSamples,
                       numStandardDeviations, 0.0)
                       .empty());
        // A floor above the size of the difference suppresses perturbed
        // elements.
        CHECK(TableUtilities::hampel(table, numAdjacentSamples,
                      numStandardDeviations, 1e-4)
                      .empty());
    }

    SECTION("The floor does not suppress genuine outliers") {
        table.updDependentColumnAtIndex(2)[150] += outlierOffset;
        const std::vector<std::pair<int, int>> expected = {{150, 2}};
        CHECK(TableUtilities::hampel(table, numAdjacentSamples,
                      numStandardDeviations, 1e-4) == expected);
    }

    SECTION("The matrix overload reports rows relative to the block") {
        // A block view of a larger matrix can be screened without copying it.
        const int blockSize = 25;
        table.updDependentColumnAtIndex(1)[blockSize + 8] += outlierOffset;

        const auto& data = table.getMatrix();
        const auto block = data.block(
                blockSize, 0, blockSize, (int)table.getNumColumns());

        const std::vector<std::pair<int, int>> expected = {{8, 1}};
        CHECK(TableUtilities::hampel(block, blockSize - 1,
                      numStandardDeviations) == expected);

        // Screening the whole table instead reports the same element at its
        // row within the table.
        const std::vector<std::pair<int, int>> expectedWhole = {
                {blockSize + 8, 1}};
        CHECK(TableUtilities::hampel(table, numAdjacentSamples,
                      numStandardDeviations) == expectedWhole);
    }

    SECTION("Invalid arguments throw") {
        CHECK_THROWS_WITH(TableUtilities::hampel(table, 0),
                Catch::Matchers::ContainsSubstring("'numAdjacentSamples'"));
        CHECK_THROWS_WITH(TableUtilities::hampel(table, -1),
                Catch::Matchers::ContainsSubstring("'numAdjacentSamples'"));
        CHECK_THROWS_WITH(TableUtilities::hampel(table, 10.0, -1.0),
                Catch::Matchers::ContainsSubstring("'numStandardDeviations'"));
        CHECK_THROWS_WITH(
                TableUtilities::hampel(table, numAdjacentSamples, 3.0, -1.0),
                Catch::Matchers::ContainsSubstring("'minimumDeviation'"));
    }

    SECTION("Non-finite arguments throw") {
        for (const double value : {SimTK::NaN, SimTK::Infinity,
                     -SimTK::Infinity}) {
            CHECK_THROWS_WITH(
                    TableUtilities::hampel(table, numAdjacentSamples, value),
                    Catch::Matchers::ContainsSubstring(
                            "'numStandardDeviations'"));
            CHECK_THROWS_WITH(TableUtilities::hampel(table,
                                      numAdjacentSamples,
                                      numStandardDeviations, value),
                    Catch::Matchers::ContainsSubstring("'minimumDeviation'"));
        }
    }
}
