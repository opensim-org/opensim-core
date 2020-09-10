/* -------------------------------------------------------------------------- *
 *                            OpenSim:  testDataTable.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Authors:                                                                   *
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
#include <iostream>

#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#define CATCH_CONFIG_MAIN
#include <OpenSim/Auxiliary/catch.hpp>
#include <OpenSim/Common/CommonUtilities.h>
#include <OpenSim/Common/PiecewiseLinearFunction.h>
#include <OpenSim/Common/TableUtilities.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/TimeSeriesTable.h>

using namespace SimTK;
using namespace OpenSim;
using OpenSim::Exception;

TEST_CASE("DataTable") {

    // Default construct, add metadata to columns, append rows one at a time.

    ValueArray<std::string> labels{};
    for(unsigned i = 1; i <= 5; ++i)
        labels.upd().push_back(SimTK::Value<std::string>{std::to_string(i)});

    ValueArray<unsigned> col_index{};
    for(unsigned i = 1; i <= 5; ++i)
        col_index.upd().push_back(SimTK::Value<unsigned>{i});

    DataTable::DependentsMetaData dep_metadata{};
    dep_metadata.setValueArrayForKey("labels", labels);
    dep_metadata.setValueArrayForKey("column-index", col_index);

    DataTable::IndependentMetaData ind_metadata{};
    ind_metadata.setValueForKey("labels", std::string{"0"});
    ind_metadata.setValueForKey("column-index", unsigned{0});

    TimeSeriesTable table{};
    {
        ASSERT(!table.hasColumnLabels());
        table.setColumnLabels({"0", "1", "2", "3"});
        ASSERT(table.hasColumnLabels());
        ASSERT(table.hasColumn("1"));
        ASSERT(table.hasColumn("2"));
        ASSERT(!table.hasColumn("column-does-not-exist"));

        table.setColumnLabel(0, "zero");
        table.setColumnLabel(2, "two");

        ASSERT(table.getColumnLabel(0) == "zero");
        ASSERT(table.getColumnLabel(2) == "two");

        table.setColumnLabel(0, "0");
        table.setColumnLabel(2, "2");

        const auto& labels = table.getColumnLabels();
        for(size_t i = 0; i < labels.size(); ++i)
            if(labels.at(i) != std::to_string(i))
                throw OpenSim::Exception{"Test failed: labels.at(i) != "
                                         "std::to_string(i)"};

        for(size_t i = 0; i < labels.size(); ++i)
            if(table.getColumnIndex(labels.at(i)) != i)
                throw OpenSim::Exception{
                        "Test failed: table.getColumnIndex(labels.at(i)) != i"};
    }

    // Test exceptions (table should be empty here).
    SimTK_TEST_MUST_THROW_EXC(table.getDependentColumnAtIndex(0),
                              OpenSim::EmptyTable);
    SimTK_TEST_MUST_THROW_EXC(table.updDependentColumnAtIndex(0),
                              OpenSim::EmptyTable);

    table.setDependentsMetaData(dep_metadata);
    table.setIndependentMetaData(ind_metadata);

    SimTK::RowVector_<double> row{5, double{0}};

    for(unsigned i = 0; i < 5; ++i)
        table.appendRow(0.00 + 0.25 * i, row + i);

    for(unsigned i = 0; i < 5; ++i)
        table.updRowAtIndex(i) += 1;

    for(unsigned i = 0; i < 5; ++i)
        table.updRow(0 + 0.25 * i) -= 1;

    try {
        table.appendRow(0.5, row);
    } catch (OpenSim::Exception&) {}

    // Test exceptions (table should have 5 rows and 5 dependent columns here).
    SimTK_TEST_MUST_THROW_EXC(table.getRowAtIndex(10),
                              OpenSim::RowIndexOutOfRange);
    SimTK_TEST_MUST_THROW_EXC(table.updRowAtIndex(10),
                              OpenSim::RowIndexOutOfRange);
    SimTK_TEST_MUST_THROW_EXC(table.getDependentColumnAtIndex(10),
                              OpenSim::ColumnIndexOutOfRange);
    SimTK_TEST_MUST_THROW_EXC(table.updDependentColumnAtIndex(10),
                              OpenSim::ColumnIndexOutOfRange);

    const auto& avgRow = table.averageRow(0.2, 0.8);
    for(int i = 0; i < avgRow.ncol(); ++i)
        OPENSIM_THROW_IF(std::abs(avgRow[i] - 2) > 1e-8/*epsilon*/,
                         OpenSim::Exception,
                         "Test failed: averageRow() failed.");

    const auto& nearRow = table.getNearestRow(0.55);
    for(int i = 0; i < nearRow.ncol(); ++i)
        ASSERT(nearRow[i] == 2);

    table.updNearestRow(0.55) += 2;
    table.updNearestRow(0.55) -= 2;
    for(int i = 0; i < nearRow.ncol(); ++i)
        ASSERT(nearRow[i] == 2);

    table.updMatrix() += 2;
    table.updMatrixBlock(0, 0, table.getNumRows(), table.getNumColumns()) -= 2;

    table.updTableMetaData().setValueForKey("DataRate", 600);
    table.updTableMetaData().setValueForKey("Filename",
                                            std::string{"/path/to/file"});

    ASSERT(table.hasColumn(0));
    ASSERT(table.hasColumn(2));
    ASSERT(!table.hasColumn(100));

    // Print out the DataTable to console.
    std::cout << table << std::endl;
    std::cout << table.toString({}, {"1", "4"});
    std::cout << table.toString({-1, -2}, {"1", "4"});

    // Retrieve added metadata and rows to check.
    REQUIRE(table.getNumRows() == unsigned{5});

    REQUIRE(table.getNumColumns() == unsigned{5});

    const auto& dep_metadata_ref = table.getDependentsMetaData();

    const auto& labels_ref = dep_metadata_ref.getValueArrayForKey("labels");
    for(unsigned i = 0; i < 5; ++i)
        CHECK(labels_ref[i].getValue<std::string>() == std::to_string(i + 1));
    {
        const auto& labels = table.getColumnLabels();
        for(unsigned i = 0; i < 5; ++i)
            CHECK(labels.at(i) == std::to_string(i + 1));
    }

    const auto& col_index_ref
        = dep_metadata_ref.getValueArrayForKey("column-index");
    for(unsigned i = 0; i < 5; ++i)
        CHECK(col_index_ref[i].getValue<unsigned>() == i + 1);

    const auto& ind_metadata_ref = table.getIndependentMetaData();

    CHECK(ind_metadata_ref.getValueForKey("labels").getValue<std::string>() ==
            std::string{"0"});
    CHECK(ind_metadata_ref.getValueForKey("column-index")
                    .getValue<unsigned>() == unsigned{0});

    table.updDependentColumnAtIndex(0) += 2;
    table.updDependentColumnAtIndex(2) += 2;
    table.updDependentColumn("1") -= 2;
    table.updDependentColumn("3") -= 2;

    for(unsigned i = 0; i < 5; ++i) {
        for(unsigned j = 0; j < 5; ++j) {
            const auto row_i_1 = table.getRowAtIndex(i);
            CHECK(row_i_1[j] == (row + i)[j]);

            const auto row_i_2 = table.getRow(0 + 0.25 * i);
            CHECK(row_i_2[j] == (row + i)[j]);

            const auto col_i = table.getDependentColumnAtIndex(i);
            CHECK(col_i[j] == j);
        }
    }

    // Append columns to DataTable.
    table.removeDependentsMetaDataForKey("column-index");
    table.appendColumn("6", {0, 1, 2, 3, 4});
    table.appendColumn("7", std::vector<double>{0, 1, 2, 3, 4});

    // ASSERT(table.getNumRows() == 5 && table.getNumColumns() == 7);

    const auto& tab_metadata_ref = table.getTableMetaData();
    CHECK(tab_metadata_ref.getValueForKey("DataRate").getValue<int>() == 600);
    CHECK(tab_metadata_ref.getValueForKey("Filename").getValue<std::string>() ==
            std::string{"/path/to/file"});

    {
    std::cout << "Test feeding rows of one table to another [double]."
              << std::endl;
    TimeSeriesTable tableCopy{};
    tableCopy.setColumnLabels(table.getColumnLabels());
    for(unsigned row = 0; row < table.getNumRows(); ++row)
        tableCopy.appendRow(table.getIndependentColumn()[row],
                            table.getRowAtIndex(row));

    ASSERT(tableCopy.getNumColumns() == table.getNumColumns());
    ASSERT(tableCopy.getNumRows()    == table.getNumRows());
    for(unsigned r = 0; r < table.getNumRows(); ++r)
        for(unsigned c = 0; c < table.getNumColumns(); ++c)
            ASSERT(tableCopy.getRowAtIndex(r)[c] ==
                       table.getRowAtIndex(r)[c]);
    }

    std::cout << "Test numComponentsPerElement()." << std::endl;
    ASSERT((static_cast<AbstractDataTable&&>
            (DataTable_<double, double    >{})).
            numComponentsPerElement() == 1);
    ASSERT((static_cast<AbstractDataTable&&>
            (DataTable_<double, Vec3      >{})).
            numComponentsPerElement() == 3);
    ASSERT((static_cast<AbstractDataTable&&>
            (DataTable_<double, UnitVec3  >{})).
            numComponentsPerElement() == 3);
    ASSERT((static_cast<AbstractDataTable&&>
            (DataTable_<double, Quaternion>{})).
            numComponentsPerElement() == 4);
    ASSERT((static_cast<AbstractDataTable&&>
            (DataTable_<double, SpatialVec>{})).
            numComponentsPerElement() == 6);

    {
        std::cout << "Test DataTable flattening constructor for Vec3."
                  << std::endl;
        DataTable_<double, Vec3> tableVec3{};
        tableVec3.setColumnLabels({"col0", "col1", "col2"});
        tableVec3.appendRow(0.1, {{1, 1, 1}, {2, 2, 2}, {3, 3, 3}});
        tableVec3.appendRow(0.2, {{3, 3, 3}, {1, 1, 1}, {2, 2, 2}});
        tableVec3.appendRow(0.3, {{2, 2, 2}, {3, 3, 3}, {1, 1, 1}});

        std::cout << tableVec3 << std::endl;

        DataTable_<double, double> tableDouble{tableVec3};
        std::vector<std::string> expLabels{"col0_1", "col0_2", "col0_3",
                                           "col1_1", "col1_2", "col1_3",
                                           "col2_1", "col2_2", "col2_3"};
        ASSERT(tableDouble.getColumnLabels()   == expLabels);
        ASSERT(tableDouble.getNumRows()        == 3);
        ASSERT(tableDouble.getNumColumns()     == 9);
        {
            const auto& row0 = tableDouble.getRowAtIndex(0);
            const auto& row1 = tableDouble.getRowAtIndex(1);
            const auto& row2 = tableDouble.getRowAtIndex(2);
            ASSERT(row0[0] == 1);
            ASSERT(row1[0] == 3);
            ASSERT(row2[0] == 2);
            ASSERT(row0[8] == 3);
            ASSERT(row1[8] == 2);
            ASSERT(row2[8] == 1);
        }

        {
        std::cout << "Test feeding rows of one table to another [Vec3]."
                  << std::endl;
        DataTableVec3 tableVec3Copy{};
        tableVec3Copy.setColumnLabels(tableVec3.getColumnLabels());
        for(unsigned row = 0; row < tableVec3.getNumRows(); ++row)
            tableVec3Copy.appendRow(tableVec3.getIndependentColumn()[row],
                                    tableVec3.getRowAtIndex(row));

        ASSERT(tableVec3Copy.getNumColumns() == tableVec3.getNumColumns());
        ASSERT(tableVec3Copy.getNumRows()    == tableVec3.getNumRows());
        for(unsigned r = 0; r < tableVec3.getNumRows(); ++r) {
            for(unsigned c = 0; c < tableVec3.getNumColumns(); ++c) {
                ASSERT(tableVec3Copy.getRowAtIndex(r)[c][0] ==
                           tableVec3.getRowAtIndex(r)[c][0]);
                ASSERT(tableVec3Copy.getRowAtIndex(r)[c][1] ==
                           tableVec3.getRowAtIndex(r)[c][1]);
                ASSERT(tableVec3Copy.getRowAtIndex(r)[c][2] ==
                           tableVec3.getRowAtIndex(r)[c][2]);
            }
        }
        }

        std::cout << "Test DataTable flatten() for Vec3." << std::endl;
        auto tableFlat = tableVec3.flatten({"_x", "_y", "_z"});
        expLabels = {"col0_x", "col0_y", "col0_z",
                     "col1_x", "col1_y", "col1_z",
                     "col2_x", "col2_y", "col2_z"};
        ASSERT(tableFlat.getColumnLabels()   == expLabels);
        ASSERT(tableFlat.getNumRows()        == 3);
        ASSERT(tableFlat.getNumColumns()     == 9);
        {
            const auto& row0 = tableFlat.getRowAtIndex(0);
            const auto& row1 = tableFlat.getRowAtIndex(1);
            const auto& row2 = tableFlat.getRowAtIndex(2);
            ASSERT(row0[0] == 1);
            ASSERT(row1[0] == 3);
            ASSERT(row2[0] == 2);
            ASSERT(row0[8] == 3);
            ASSERT(row1[8] == 2);
            ASSERT(row2[8] == 1);
        }

        std::cout << tableFlat << std::endl;

        std::cout << "Test DataTable flattening constructor for Quaternion."
                  << std::endl;
        DataTable_<double, Quaternion> tableQuat{};
        tableQuat.setColumnLabels({"col0", "col1", "col2"});
        tableQuat.appendRow(0.1, {{1, 1, 1, 1}, {2, 2, 2, 2}, {3, 3, 3, 3}});
        tableQuat.appendRow(0.2, {{3, 3, 3, 3}, {1, 1, 1, 1}, {2, 2, 2, 2}});
        tableQuat.appendRow(0.3, {{2, 2, 2, 2}, {3, 3, 3, 3}, {1, 1, 1, 1}});

        std::cout << tableQuat << std::endl;

        tableDouble = tableQuat;
        ASSERT(tableDouble.getColumnLabels().size() == 12);
        ASSERT(tableDouble.getNumRows()             == 3);
        ASSERT(tableDouble.getNumColumns()          == 12);

        std::cout << "Test DataTable flattening constructor for UnitVec3."
                  << std::endl;
        DataTable_<double, Vec3> tableUnitVec3{};
        tableUnitVec3.setColumnLabels({"col0", "col1", "col2"});
        tableUnitVec3.appendRow(0.1, {{1, 1, 1}, {2, 2, 2}, {3, 3, 3}});
        tableUnitVec3.appendRow(0.2, {{3, 3, 3}, {1, 1, 1}, {2, 2, 2}});
        tableUnitVec3.appendRow(0.3, {{2, 2, 2}, {3, 3, 3}, {1, 1, 1}});

        std::cout << tableUnitVec3 << std::endl;

        tableDouble = tableUnitVec3;
        ASSERT(tableDouble.getColumnLabels().size() == 9);
        ASSERT(tableDouble.getNumRows()             == 3);
        ASSERT(tableDouble.getNumColumns()          == 9);

        std::cout << "Test DataTable flattening constructor for SpatialVec."
                  << std::endl;
        DataTable_<double, SpatialVec> tableSpatialVec{};
        tableSpatialVec.setColumnLabels({"col0", "col1", "col2"});
        tableSpatialVec.appendRow(0.1, {{{1, 1, 1}, {1, 1, 1}},
                                        {{2, 2, 2}, {2, 2, 2}},
                                        {{3, 3, 3}, {3, 3, 3}}});
        tableSpatialVec.appendRow(0.2, {{{3, 3, 3}, {3, 3, 3}},
                                        {{1, 1, 1}, {1, 1, 1}},
                                        {{2, 2, 2}, {2, 2, 2}}});
        tableSpatialVec.appendRow(0.3, {{{2, 2, 2}, {2, 2, 2}},
                                        {{3, 3, 3}, {3, 3, 3}},
                                        {{1, 1, 1}, {1, 1, 1}}});

        std::cout << tableSpatialVec << std::endl;

        tableDouble = tableSpatialVec;
        ASSERT(tableDouble.getColumnLabels().size() == 18);
        ASSERT(tableDouble.getNumRows()             == 3);
        ASSERT(tableDouble.getNumColumns()          == 18);

        std::cout << tableDouble << std::endl;
    }
    {
        std::cout << "Test TimeSeriesTable flattening constructor for Vec3"
                  << std::endl;
        TimeSeriesTable_<Vec3> tableVec3{};
        tableVec3.setColumnLabels({"col0", "col1", "col2"});
        tableVec3.appendRow(0.1, {{1, 1, 1}, {2, 2, 2}, {3, 3, 3}});
        tableVec3.appendRow(0.2, {{3, 3, 3}, {1, 1, 1}, {2, 2, 2}});
        tableVec3.appendRow(0.3, {{2, 2, 2}, {3, 3, 3}, {1, 1, 1}});

        const auto& avgRowVec3 = tableVec3.averageRow(0.1, 0.2);
        for(int i = 0; i < 3; ++i)
            OPENSIM_THROW_IF(std::abs(avgRowVec3[0][i] - 2) > 1e-8/*epsilon*/,
                             OpenSim::Exception,
                             "Test failed: averageRow() failed.");

        const auto& nearRowVec3 = tableVec3.getNearestRow(0.29);
        for(int i = 0; i < 3; ++i)
            ASSERT(nearRowVec3[0][i] == 2);
        tableVec3.updNearestRow(0.29) += SimTK::Vec3{2};
        tableVec3.updNearestRow(0.29) -= SimTK::Vec3{2};
        for(int i = 0; i < 3; ++i)
            ASSERT(nearRowVec3[0][i] == 2);

        std::cout << tableVec3 << std::endl;

        TimeSeriesTable_<double> tableDouble{tableVec3};
        std::vector<std::string> expLabels{"col0_1", "col0_2", "col0_3",
                                           "col1_1", "col1_2", "col1_3",
                                           "col2_1", "col2_2", "col2_3"};
        ASSERT(tableDouble.getColumnLabels() == expLabels);
        ASSERT(tableDouble.getNumRows()      == 3);
        ASSERT(tableDouble.getNumColumns()   == 9);
        {
            const auto& row0 = tableDouble.getRowAtIndex(0);
            const auto& row1 = tableDouble.getRowAtIndex(1);
            const auto& row2 = tableDouble.getRowAtIndex(2);
            ASSERT(row0[0] == 1);
            ASSERT(row1[0] == 3);
            ASSERT(row2[0] == 2);
            ASSERT(row0[8] == 3);
            ASSERT(row1[8] == 2);
            ASSERT(row2[8] == 1);
        }

        std::cout << tableDouble << std::endl;

        std::cout << "Test TimeSeriesTable flatten() for Vec3." << std::endl;
        auto tableFlat = tableVec3.flatten({"_x", "_y", "_z"});
        expLabels = {"col0_x", "col0_y", "col0_z",
                     "col1_x", "col1_y", "col1_z",
                     "col2_x", "col2_y", "col2_z"};
        ASSERT(tableFlat.getColumnLabels()   == expLabels);
        ASSERT(tableFlat.getNumRows()        == 3);
        ASSERT(tableFlat.getNumColumns()     == 9);
        {
            const auto& row0 = tableFlat.getRowAtIndex(0);
            const auto& row1 = tableFlat.getRowAtIndex(1);
            const auto& row2 = tableFlat.getRowAtIndex(2);
            ASSERT(row0[0] == 1);
            ASSERT(row1[0] == 3);
            ASSERT(row2[0] == 2);
            ASSERT(row0[8] == 3);
            ASSERT(row1[8] == 2);
            ASSERT(row2[8] == 1);
        }

        std::cout << tableFlat << std::endl;

        std::cout << "Test TimeSeriesTable flattening constructor for "
                     "Quaternion" << std::endl;
        TimeSeriesTable_<Quaternion> tableQuat{};
        tableQuat.setColumnLabels({"col0", "col1", "col2"});
        tableQuat.appendRow(0.1, {{1, 1, 1, 1}, {2, 2, 2, 2}, {3, 3, 3, 3}});
        tableQuat.appendRow(0.2, {{3, 3, 3, 3}, {1, 1, 1, 1}, {2, 2, 2, 2}});
        tableQuat.appendRow(0.3, {{2, 2, 2, 2}, {3, 3, 3, 3}, {1, 1, 1, 1}});

        const auto& avgRowQuat = tableQuat.averageRow(0.1, 0.2);
        for(int i = 0; i < 4; ++i) {
            OPENSIM_THROW_IF(std::abs(avgRowQuat[0][i] - 0.5) > 1e-8/*epsilon*/,
                             OpenSim::Exception,
                             "Test failed: averageRow() failed.");
        }

        const auto& nearRowQuat = tableQuat.getNearestRow(0.29);
        for(int i = 0; i < 4; ++i)
            ASSERT(std::abs(nearRowQuat[0][i] - 0.5) < 1e-8/*eps*/);

        std::cout << tableQuat << std::endl;

        tableDouble = tableQuat;
        ASSERT(tableDouble.getColumnLabels().size() == 12);
        ASSERT(tableDouble.getNumRows()             == 3);
        ASSERT(tableDouble.getNumColumns()          == 12);

        std::cout << tableDouble << std::endl;

        std::cout << "Test TimeSeriesTable flattening constructor for UnitVec3"
                  << std::endl;
        TimeSeriesTable_<Vec3> tableUnitVec3{};
        tableUnitVec3.setColumnLabels({"col0", "col1", "col2"});
        tableUnitVec3.appendRow(0.1, {{1, 1, 1}, {2, 2, 2}, {3, 3, 3}});
        tableUnitVec3.appendRow(0.2, {{3, 3, 3}, {1, 1, 1}, {2, 2, 2}});
        tableUnitVec3.appendRow(0.3, {{2, 2, 2}, {3, 3, 3}, {1, 1, 1}});

        std::cout << tableUnitVec3 << std::endl;

        tableDouble = tableUnitVec3;
        ASSERT(tableDouble.getColumnLabels().size() == 9);
        ASSERT(tableDouble.getNumRows()             == 3);
        ASSERT(tableDouble.getNumColumns()          == 9);

        std::cout << tableDouble << std::endl;

        std::cout << "Test TimeSeriesTable flattening constructor for "
                     "SpatialVec" << std::endl;
        TimeSeriesTable_<SpatialVec> tableSpatialVec{};
        tableSpatialVec.setColumnLabels({"col0", "col1", "col2"});
        tableSpatialVec.appendRow(0.1, {{{1, 1, 1}, {1, 1, 1}},
                                        {{2, 2, 2}, {2, 2, 2}},
                                        {{3, 3, 3}, {3, 3, 3}}});
        tableSpatialVec.appendRow(0.2, {{{3, 3, 3}, {3, 3, 3}},
                                        {{1, 1, 1}, {1, 1, 1}},
                                        {{2, 2, 2}, {2, 2, 2}}});
        tableSpatialVec.appendRow(0.3, {{{2, 2, 2}, {2, 2, 2}},
                                        {{3, 3, 3}, {3, 3, 3}},
                                        {{1, 1, 1}, {1, 1, 1}}});

        const auto& avgRowSVec = tableSpatialVec.averageRow(0.1, 0.2);
        for(int i = 0; i < 3; ++i) {
            OPENSIM_THROW_IF(std::abs(avgRowSVec[0][0][i] - 2) > 1e-8/*eps*/,
                             OpenSim::Exception,
                             "Test failed: averageRow() failed.");
        }

        const auto& nearRowSVec = tableSpatialVec.getNearestRow(0.29);
        for(int i = 0; i < 3; ++i)
            ASSERT(nearRowSVec[0][0][i] == 2);

        tableSpatialVec.updNearestRow(0.29) += SimTK::SpatialVec{{2, 2, 2},
                                                                 {2, 2, 2}};
        tableSpatialVec.updNearestRow(0.29) -= SimTK::SpatialVec{{2, 2, 2},
                                                                 {2, 2, 2}};
        for(int i = 0; i < 3; ++i)
            ASSERT(nearRowSVec[0][0][i] == 2);

        std::cout << tableSpatialVec << std::endl;

        tableDouble = tableSpatialVec;
        ASSERT(tableDouble.getColumnLabels().size() == 18);
        ASSERT(tableDouble.getNumRows()             == 3);
        ASSERT(tableDouble.getNumColumns()          == 18);

        std::cout << tableDouble << std::endl;
    }
    {
        std::cout << "Test DataTable packing." << std::endl;
        DataTable_<double, double> tableDouble{};
        tableDouble.setColumnLabels({"col0_x", "col0_y", "col0_z",
                                     "col1_x", "col1_y", "col1_z",
                                     "col2_x", "col2_y", "col2_z",
                                     "col3_x", "col3_y", "col3_z"});
        tableDouble.appendRow(1, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1});
        tableDouble.appendRow(2, {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2});
        tableDouble.appendRow(3, {3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3});
        tableDouble.addTableMetaData("string", std::string{"string"});
        tableDouble.addTableMetaData("int", 10);

        ASSERT(tableDouble.getColumnLabels().size() == 12);
        ASSERT(tableDouble.getNumRows()             == 3);
        ASSERT(tableDouble.getNumColumns()          == 12);

        std::cout << tableDouble << std::endl;

        std::cout << "Test DataTable packing for Vec3 with suffix specified."
                  << std::endl;
        auto tableVec3_1 = tableDouble.pack<SimTK::Vec3>({"_x", "_y", "_z"});
        std::vector<std::string> expLabels{"col0", "col1", "col2", "col3"};
        ASSERT(tableVec3_1.getColumnLabels() == expLabels);
        ASSERT(tableVec3_1.getNumRows()      == 3);
        ASSERT(tableVec3_1.getNumColumns()   == 4);
        ASSERT(tableVec3_1.getTableMetaData<std::string>("string") == "string");
        ASSERT(tableVec3_1.getTableMetaData<int>("int")            == 10);
        std::cout << tableVec3_1 << std::endl;

        std::cout << "Test DataTable packing for Vec3 with suffix unspecified."
                  << std::endl;
        auto tableVec3_2 = tableDouble.pack<SimTK::Vec3>();
        ASSERT(tableVec3_2.getColumnLabels() == expLabels);
        ASSERT(tableVec3_2.getNumRows()      == 3);
        ASSERT(tableVec3_2.getNumColumns()   == 4);
        ASSERT(tableVec3_2.getTableMetaData<std::string>("string") == "string");
        ASSERT(tableVec3_2.getTableMetaData<int>("int")            == 10);
        std::cout << tableVec3_2 << std::endl;

        std::cout << "Test DataTable packing for UnitVec3." << std::endl;
        auto tableUVec3 = tableDouble.pack<SimTK::UnitVec3>();
        ASSERT(tableUVec3.getColumnLabels() == expLabels);
        ASSERT(tableUVec3.getNumRows()      == 3);
        ASSERT(tableUVec3.getNumColumns()   == 4);
        ASSERT(tableUVec3.getTableMetaData<std::string>("string") == "string");
        ASSERT(tableUVec3.getTableMetaData<int>("int")            == 10);
        std::cout << tableUVec3 << std::endl;

        std::cout << "Test DataTable packing for Quaternion." << std::endl;
        tableDouble.setColumnLabels({"col0.0", "col0.1", "col0.2", "col0.3",
                                     "col1.0", "col1.1", "col1.2", "col1.3",
                                     "col2.0", "col2.1", "col2.2", "col2.3"});
        auto tableQuat = tableDouble.pack<SimTK::Quaternion>();
        expLabels = {"col0", "col1", "col2"};
        ASSERT(tableQuat.getColumnLabels() == expLabels);
        ASSERT(tableQuat.getNumRows()      == 3);
        ASSERT(tableQuat.getNumColumns()   == 3);
        ASSERT(tableQuat.getTableMetaData<std::string>("string") == "string");
        ASSERT(tableQuat.getTableMetaData<int>("int")            == 10);
        std::cout << tableQuat << std::endl;

        std::cout << "Test DataTable packing for SpatialVec" << std::endl;
        tableDouble.setColumnLabels({"col0.0", "col0.1", "col0.2",
                                     "col0.3", "col0.4", "col0.5",
                                     "col1.0", "col1.1", "col1.2",
                                     "col1.3", "col1.4", "col1.5"});
        auto tableSVec = tableDouble.pack<SimTK::SpatialVec>();
        expLabels = {"col0", "col1"};
        ASSERT(tableSVec.getColumnLabels() == expLabels);
        ASSERT(tableSVec.getNumRows()      == 3);
        ASSERT(tableSVec.getNumColumns()   == 2);
        ASSERT(tableSVec.getTableMetaData<std::string>("string") == "string");
        ASSERT(tableSVec.getTableMetaData<int>("int")            == 10);
        std::cout << tableSVec << std::endl;
    }
    {
        std::cout << "Test TimeSeriesTable packing." << std::endl;
        TimeSeriesTable_<double> tableDouble{};
        tableDouble.setColumnLabels({"col0_x", "col0_y", "col0_z",
                                     "col1_x", "col1_y", "col1_z",
                                     "col2_x", "col2_y", "col2_z",
                                     "col3_x", "col3_y", "col3_z"});
        tableDouble.appendRow(1, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1});
        tableDouble.appendRow(2, {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2});
        tableDouble.appendRow(3, {3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3});
        tableDouble.addTableMetaData("string", std::string{"string"});
        tableDouble.addTableMetaData("int", 10);

        ASSERT(tableDouble.getColumnLabels().size() == 12);
        ASSERT(tableDouble.getNumRows()             == 3);
        ASSERT(tableDouble.getNumColumns()          == 12);

        std::cout << tableDouble << std::endl;

        auto tableDoubleCopy = tableDouble;
        Vector row2{ tableDoubleCopy.getRowAtIndex(2).transpose() };
        tableDoubleCopy.removeRowAtIndex(1);
        Vector row1{ tableDoubleCopy.getRowAtIndex(1).transpose() };
        ASSERT_EQUAL(row1, row2, 0.0);
        tableDoubleCopy.removeRowAtIndex(0);
        Vector row0{ tableDoubleCopy.getRowAtIndex(0).transpose() };
        ASSERT_EQUAL(row0, row2, 0.0);

        std::cout << "Test TimeSeriesTable packing for Vec3 with suffix"
                     " specified."
                  << std::endl;
        TimeSeriesTable_<SimTK::Vec3> tableVec3_1 =
            tableDouble.pack<SimTK::Vec3>({"_x", "_y", "_z"});
        std::vector<std::string> expLabels{"col0", "col1", "col2", "col3"};
        ASSERT(tableVec3_1.getColumnLabels() == expLabels);
        ASSERT(tableVec3_1.getNumRows()      == 3);
        ASSERT(tableVec3_1.getNumColumns()   == 4);
        ASSERT(tableVec3_1.getTableMetaData<std::string>("string") == "string");
        ASSERT(tableVec3_1.getTableMetaData<int>("int")            == 10);
        std::cout << tableVec3_1 << std::endl;

        std::cout << "Test TimeSeriesTable packing for Vec3 with suffix"
                     " unspecified."
                  << std::endl;
        TimeSeriesTable_<SimTK::Vec3> tableVec3_2 =
            tableDouble.pack<SimTK::Vec3>();
        ASSERT(tableVec3_2.getColumnLabels() == expLabels);
        ASSERT(tableVec3_2.getNumRows()      == 3);
        ASSERT(tableVec3_2.getNumColumns()   == 4);
        ASSERT(tableVec3_2.getTableMetaData<std::string>("string") == "string");
        ASSERT(tableVec3_2.getTableMetaData<int>("int")            == 10);
        std::cout << tableVec3_2 << std::endl;

        std::cout << "Test TimeSeriesTable packing for UnitVec3." << std::endl;
        auto tableUVec3 = tableDouble.pack<SimTK::UnitVec3>();
        ASSERT(tableUVec3.getColumnLabels() == expLabels);
        ASSERT(tableUVec3.getNumRows()      == 3);
        ASSERT(tableUVec3.getNumColumns()   == 4);
        ASSERT(tableUVec3.getTableMetaData<std::string>("string") == "string");
        ASSERT(tableUVec3.getTableMetaData<int>("int")            == 10);
        std::cout << tableUVec3 << std::endl;

        std::cout << "Test TimeSeriesTable packing for Quaternion." << std::endl;
        tableDouble.setColumnLabels({"col0.0", "col0.1", "col0.2", "col0.3",
                                     "col1.0", "col1.1", "col1.2", "col1.3",
                                     "col2.0", "col2.1", "col2.2", "col2.3"});
        TimeSeriesTable_<SimTK::Quaternion> tableQuat =
            tableDouble.pack<SimTK::Quaternion>();
        expLabels = {"col0", "col1", "col2"};
        ASSERT(tableQuat.getColumnLabels() == expLabels);
        ASSERT(tableQuat.getNumRows()      == 3);
        ASSERT(tableQuat.getNumColumns()   == 3);
        ASSERT(tableQuat.getTableMetaData<std::string>("string") == "string");
        ASSERT(tableQuat.getTableMetaData<int>("int")            == 10);
        std::cout << tableQuat << std::endl;

        std::cout << "Test TimeSeriesTable packing for SpatialVec" << std::endl;
        tableDouble.setColumnLabels({"col0.0", "col0.1", "col0.2",
                                     "col0.3", "col0.4", "col0.5",
                                     "col1.0", "col1.1", "col1.2",
                                     "col1.3", "col1.4", "col1.5"});
        TimeSeriesTable_<SimTK::SpatialVec> tableSVec =
            tableDouble.pack<SimTK::SpatialVec>();
        expLabels = {"col0", "col1"};
        ASSERT(tableSVec.getColumnLabels() == expLabels);
        ASSERT(tableSVec.getNumRows()      == 3);
        ASSERT(tableSVec.getNumColumns()   == 2);
        ASSERT(tableSVec.getTableMetaData<std::string>("string") == "string");
        ASSERT(tableSVec.getTableMetaData<int>("int")            == 10);
        std::cout << tableSVec << std::endl;
    }

    {
        std::cout << "Test that TimeSeriesTable can have 0 columns."
                  << std::endl;
        TimeSeriesTable table(std::vector<double>{1.5, 2.5, 3.5});
        std::vector<std::string> expLabels = {};
        ASSERT(table.getColumnLabels() == expLabels);
        ASSERT(table.getNumRows()      == 3);
        ASSERT(table.getNumColumns()   == 0);

        // Can append an empty row.
        table.appendRow(4.5, {});
        ASSERT(table.getNumRows()      == 4);
        ASSERT(table.getNumColumns()   == 0);

        table.removeRowAtIndex(3);
        ASSERT(table.getNumRows()      == 3);
        ASSERT(table.getNumColumns()   == 0);

        table.appendRow(4.5, {});
        ASSERT(table.getNumRows()      == 4);
        ASSERT(table.getNumColumns()   == 0);

        // Cannot append a non-empty row.
        SimTK_TEST_MUST_THROW_EXC(table.appendRow(5.5, {6.1}),
                                  IncorrectNumColumns);

        // Can append a column to a table that has no columns yet.
        table.appendColumn("col1", {5.4, 5.3, 5.6, 5.8});
        ASSERT(table.getNumRows()      == 4);
        ASSERT(table.getNumColumns()   == 1);

        // Appending a column with an incorrect number of rows.
        SimTK_TEST_MUST_THROW_EXC(table.appendColumn("col2", {5.4, 5.3, 5.6}),
                                  IncorrectNumRows);

        // Can appendRow after appendColumn.
        table.appendRow(5.5, {1.3});
        ASSERT(table.getNumRows()      == 5);
        ASSERT(table.getNumColumns()   == 1);

        // Can create an empty table by providing an empty indVec.
        TimeSeriesTable emptyTable(std::vector<double>{});
        // Bu we can't append columns to an empty table.
        SimTK_TEST_MUST_THROW_EXC(emptyTable.appendColumn("col0", {}),
                                  InvalidCall);

        // Ensure that we are validating the time stamps are increasing
        SimTK_TEST_MUST_THROW_EXC(
                TimeSeriesTable(std::vector<double>{1.5, 1.6, 1.6}),
                TimestampGreaterThanEqualToNext);
        SimTK_TEST_MUST_THROW_EXC(
                TimeSeriesTable(std::vector<double>{1.5, 1.6, 1.4}),
                TimestampGreaterThanEqualToNext);
        SimTK_TEST_MUST_THROW_EXC(table.appendRow(-0.3, {0.6}),
                TimestampLessThanEqualToPrevious);
    }

    {
        //Test removing columns and rows and their performance;
        int nr = 50000;
        int nc = 25;
        std::vector<double> indColumn(size_t(nr), 1.0);
        SimTK::Matrix huge(nr, nc, SimTK::NaN);
        std::vector<std::string> labels(size_t(nc), "");
        int c = 0;
        for (auto& label : labels) {
            label = "c" + std::to_string(c++);
        }
        for (int r = 0; r < nr; ++r)
            indColumn[r] = 0.001*r;

        TimeSeriesTable table{ indColumn, huge, labels };

        std::clock_t t0 = std::clock();
        for (int i = 1; i < nc; ++i)
            table.removeColumnAtIndex(1);

        double dTc = 1.e3*(std::clock() - t0) / CLOCKS_PER_SEC;

        std::cout << "\tRemoving columns took:"   << dTc << "ms" << std::endl;

        TimeSeriesTable table2{ indColumn, huge, labels };

        t0 = std::clock();
        for (int i = 1; i < nc; ++i)
            table2.removeRowAtIndex(1);

        double dTr = 1.e3*(std::clock() - t0) / CLOCKS_PER_SEC;

        std::cout << "\tRemoving rows took:" << dTr << "ms" << std::endl;
    }
}

TEST_CASE("TableUtilities::checkNonUniqueLabels") {
    CHECK_THROWS_AS(TableUtilities::checkNonUniqueLabels({"a", "a"}),
                    NonUniqueLabels);
}

TEST_CASE("TableUtilities::isInDegrees") {
    SECTION("no inDegrees metadata") {
        CHECK_THROWS_WITH(TableUtilities::isInDegrees(TimeSeriesTable()),
                Catch::Contains("Table does not have 'inDegrees' metadata."));
    }
    SECTION("inDegrees=invalid") {
        TimeSeriesTable table;
        table.addTableMetaData("inDegrees", std::string("invalid"));
        CHECK_THROWS_WITH(!TableUtilities::isInDegrees(table),
                Catch::Contains("Expected table's 'inDegrees' metadata"));
    }
    Storage sto;
    SECTION("inDegrees=yes") {
        sto.setInDegrees(true);
        TimeSeriesTable table = sto.exportToTable();
        CHECK(TableUtilities::isInDegrees(table));
    }
    SECTION("inDegrees=no") {
        sto.setInDegrees(false);
        TimeSeriesTable table = sto.exportToTable();
        CHECK(!TableUtilities::isInDegrees(table));
    }
}

TEST_CASE("TableUtilities::findStateLabelIndex") {
    CHECK(TableUtilities::findStateLabelIndex(
                  std::vector<std::string>{"hip_flexion"},
                  "/jointset/hip/hip_flexion/value") == 0);
    CHECK(TableUtilities::findStateLabelIndex(
                  std::vector<std::string>{"hip_flexion_u"},
                  "/jointset/hip/hip_flexion/speed") == 0);
    CHECK(TableUtilities::findStateLabelIndex(
                  std::vector<std::string>{"vasti.activation"},
                  "/forceset/vasti/activation") == 0);
    CHECK(TableUtilities::findStateLabelIndex(
                  std::vector<std::string>{"vasti.fiber_length"},
                  "/forceset/vasti/fiber_length") == 0);

    CHECK(TableUtilities::findStateLabelIndex(
                  std::vector<std::string>{"hip_flexin"},
                  "/jointset/hip/hip_flexion/value") == -1);
}

TEST_CASE("TableUtilities::filterLowpass") {
    const int numRows = 100;

    // Create a table with random values, and write the table to a file.
    std::vector<double> time(numRows);
    SimTK::Vector timeVec = createVectorLinspace(numRows, 0, 1);
    std::copy_n(timeVec.getContiguousScalarData(), numRows, time.data());
    TimeSeriesTable table(time);
    table.appendColumn("a", SimTK::Test::randVector(numRows));
    STOFileAdapter::write(table, "testFilterLowpass.sto");

    // Filter the table.
    TableUtilities::filterLowpass(table, 6.0);

    // Load the original table as a storage, and filter the storage.
    Storage sto("testFilterLowpass.sto");
    sto.lowpassIIR(6.0);

    // The filtered table and storage should match.
    SimTK::Vector filteredStorageColumn(numRows);
    double* columnData = filteredStorageColumn.updContiguousScalarData();
    sto.getDataColumn("a", columnData);
    const auto& filteredTableColumn = table.getDependentColumnAtIndex(0);
    for (int i = 0; i < numRows; ++i) {
        CHECK(filteredStorageColumn[i] == Approx(filteredTableColumn[i]));
    }
}

TEST_CASE("TableUtilities::pad") {
    Storage sto("test.sto");
    TimeSeriesTable paddedTable = sto.exportToTable();
    TableUtilities::pad(paddedTable, 1);

    sto.pad(1);
    TimeSeriesTable paddedSto = sto.exportToTable();

    CHECK(paddedTable.getIndependentColumn() ==
            paddedSto.getIndependentColumn());
    REQUIRE(paddedTable.getNumRows() == paddedSto.getNumRows());
    REQUIRE(paddedTable.getNumColumns() == paddedSto.getNumColumns());
    for (int irow = 0; irow < (int)paddedTable.getNumRows(); ++irow) {
        for (int icol = 0; icol < (int)paddedTable.getNumColumns(); ++icol) {
            CHECK(paddedTable.getMatrix().getElt(irow, icol) ==
                    paddedSto.getMatrix().getElt(irow, icol));
        }
    }
}

TEST_CASE("TableUtilities::resample") {
    TimeSeriesTable table(std::vector<double>{0.0, 1, 2});
    table.appendColumn("a", {1.0, 0.5, 0.0});
    {
        TimeSeriesTable resampled =
                TableUtilities::resample(table, createVector({0.5}));
        REQUIRE(resampled.getNumRows() == 1);
        REQUIRE(resampled.getNumColumns() == 1);
        CHECK(resampled.getIndependentColumn()[0] == 0.5);

        const VectorView view = resampled.getDependentColumnAtIndex(0);
        CHECK(view[0] == Approx(0.75));
    }
    {
        TimeSeriesTable resampled = TableUtilities::resampleWithInterval(table,
                0.4);
        REQUIRE(resampled.getNumRows() == 6);
        REQUIRE(resampled.getNumColumns() == 1);
        const auto& time = resampled.getIndependentColumn();
        CHECK(time[0] == Approx(0).margin(1e-10));
        CHECK(time[1] == Approx(0.4));
        CHECK(time[2] == Approx(0.8));
        CHECK(time[3] == Approx(1.2));
        CHECK(time[4] == Approx(1.6));
        CHECK(time[5] == Approx(2.0));

        const auto& column = resampled.getDependentColumnAtIndex(0);
        CHECK(column[0] == Approx(1.0));
        CHECK(column[1] == Approx(0.8));
        CHECK(column[2] == Approx(0.6));
        CHECK(column[3] == Approx(0.4));
        CHECK(column[4] == Approx(0.2));
        CHECK(column[5] == Approx(0.0).margin(1e-10));
    }
}
