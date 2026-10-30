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
    const TimeSeriesTable original = createSineTable();
    TimeSeriesTable table = original;
    const int numRows = (int)table.getNumRows();
    const std::vector<std::pair<int, int>> expected = {
        {0, 3}, {1, 5}, {1, 40}, {2, 17}, {3, numRows - 1}};

    SECTION("A table of finite values has no non-finite elements") {
        CHECK(TableUtilities::findNonFiniteElements(table).empty());
    }

    SECTION("NaN values are found at the expected locations") {
        for (const auto& [icol, irow] : expected) {
            table.updDependentColumnAtIndex(icol)[irow] = SimTK::NaN;
        }
        CHECK(TableUtilities::findNonFiniteElements(table) == expected);
    }

    SECTION("Infinite values are found at the expected locations") {
        for (const auto& [icol, irow] : expected) {
            table.updDependentColumnAtIndex(icol)[irow] =
                    irow % 2 ? SimTK::Infinity : -SimTK::Infinity;
        }
        CHECK(TableUtilities::findNonFiniteElements(table) == expected);
    }
}

TEST_CASE("hampel") {
    const double numStandardDeviations = 5.0;
    const TimeSeriesTable original = createSineTable();
    TimeSeriesTable table = original;
    const int numRows = (int)table.getNumRows();
    const int numAdjacentSamples = 10;
    const double outlierOffset = 100.0;

    SECTION("A table of smooth curves has no outliers") {
        CHECK(TableUtilities::hampel(
                table, numAdjacentSamples, numStandardDeviations).empty());
    }

    SECTION("Outliers are found at the expected locations") {
        const std::vector<std::pair<int, int>> expected = {
                {0, 7}, {1, 100}, {3, 2}, {3, numRows - 3}};
        for (const auto& [icol, irow] : expected) {
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
                {0, 4}, {0, 6}, {0, 9}};
        CHECK(TableUtilities::hampel(table, numAdjacentSamples,
                      numStandardDeviations) == expected);
    }

    SECTION("A window that is mostly non-finite still screens what is left") {
        std::vector<std::pair<int, int>> expected;
        for (int irow = 1; irow < 25; irow += 2) {
            table.updDependentColumnAtIndex(0)[irow] = SimTK::Infinity;
            expected.emplace_back(0, irow);
        }
        table.updDependentColumnAtIndex(0)[22] += outlierOffset;
        expected.emplace_back(0, 22);
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

        const std::vector<std::pair<int, int>> expected = {{1, 200}};
        CHECK(TableUtilities::hampel(table, numAdjacentSamples,
                      numStandardDeviations) == expected);
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
        const std::vector<std::pair<int, int>> expected = {{2, 150}};
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

        const std::vector<std::pair<int, int>> expected = {{1, 8}};
        CHECK(TableUtilities::hampel(block, blockSize - 1,
                      numStandardDeviations) == expected);

        // Screening the whole table instead reports the same element at its
        // row within the table.
        const std::vector<std::pair<int, int>> expectedWhole = {
                {1, blockSize + 8}};
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
}
