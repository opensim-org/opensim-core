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
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <iostream>

int main() {
    using namespace SimTK;
    using namespace OpenSim;
    using OpenSim::Exception;

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
                throw Exception{"Test failed: labels.at(i) != "
                                "std::to_string(i)"};

        for(size_t i = 0; i < labels.size(); ++i)
            if(table.getColumnIndex(labels.at(i)) != i)
                throw Exception{"Test failed: "
                                "table.getColumnIndex(labels.at(i)) != i"};
    }
    // Print out the DataTable to console.
    try {
        std::cout << table << std::endl;
        throw Exception{"Test failed: Exception expected."};
    } catch(const OpenSim::EmptyTable&) {}

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

    const auto& avgRow = table.averageRow(0.2, 0.8);
    for(int i = 0; i < avgRow.ncol(); ++i)
        OPENSIM_THROW_IF(std::abs(avgRow[i] - 2) > 1e-8/*epsilon*/,
                         Exception,
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
    if(table.getNumRows() != unsigned{5})
        throw Exception{"Test Failed: table.getNumRows() != unsigned{5}"};

    if(table.getNumColumns() != unsigned{5})
        throw Exception{"Test Failed: table.getNumColumns() != unsigned{5}"};

    const auto& dep_metadata_ref = table.getDependentsMetaData();

    const auto& labels_ref = dep_metadata_ref.getValueArrayForKey("labels");
    for(unsigned i = 0; i < 5; ++i)
        if(labels_ref[i].getValue<std::string>() != std::to_string(i + 1))
            throw Exception{"Test failed: labels_ref[i].getValue<std::string>()"
                    " != std::to_string(i + 1)"};
    {
    const auto& labels = table.getColumnLabels();
    for(unsigned i = 0; i < 5; ++i)
        if(labels.at(i) != std::to_string(i + 1))
            throw Exception{"Test failed: labels[i].getValue<std::string>()"
                    " != std::to_string(i + 1)"};
    }

    const auto& col_index_ref 
        = dep_metadata_ref.getValueArrayForKey("column-index");
    for(unsigned i = 0; i < 5; ++i)
        if(col_index_ref[i].getValue<unsigned>() != i + 1)
            throw Exception{"Test failed: col_index_ref[i].getValue<unsigned>()"
                    " != i + 1"};

    const auto& ind_metadata_ref = table.getIndependentMetaData();
    
    if(ind_metadata_ref.getValueForKey("labels").getValue<std::string>() 
       != std::string{"0"})
        throw Exception{"Test failed: ind_metadata_ref.getValueForKey"
                "(\"labels\").getValue<std::string>() != std::string{\"0\"}"};
    if(ind_metadata_ref.getValueForKey("column-index").getValue<unsigned>()
       != unsigned{0})
        throw Exception{"Test failed: ind_metadata_ref.getValueForKey"
                "(\"column-index\").getValue<unsigned>() != unsigned{0}"};

    table.updDependentColumnAtIndex(0) += 2;
    table.updDependentColumnAtIndex(2) += 2;
    table.updDependentColumn("1") -= 2;
    table.updDependentColumn("3") -= 2;

    for(unsigned i = 0; i < 5; ++i) {
        for(unsigned j = 0; j < 5; ++j) {
            const auto row_i_1 = table.getRowAtIndex(i);
            if(row_i_1[j] != (row + i)[j])
                throw Exception{"Test failed: row_i_1[j] != (row + i)[j]"};

            const auto row_i_2 = table.getRow(0 + 0.25 * i);
            if(row_i_2[j] != (row + i)[j])
                throw Exception{"Test failed: row_i_2[j] != (row + i)[j]"};

            const auto col_i = table.getDependentColumnAtIndex(i);
            if(col_i[j] != j)
                throw Exception{"Test failed: table.getDependentColumnAtIndex"
                        "(i)[j] != j"};
        }
    }

    // Append columns to DataTable.
    table.removeDependentsMetaDataForKey("column-index");
    table.appendColumn("6", {0, 1, 2, 3, 4});
    table.appendColumn("7", std::vector<double>{0, 1, 2, 3, 4});

    // ASSERT(table.getNumRows() == 5 && table.getNumColumns() == 7);

    const auto& tab_metadata_ref = table.getTableMetaData();
    if(tab_metadata_ref.getValueForKey("DataRate").getValue<int>() 
       != 600)
        throw Exception{"Test failed: tab_metadata_ref.getValueForKey"
                "(\"DataRate\").getValue<int>() != 600"};
    if(tab_metadata_ref.getValueForKey("Filename").getValue<std::string>()
       != std::string{"/path/to/file"})
        throw Exception{"Test failed: tab_metadata_ref.getValueForKey"
                "(\"Filename\").getValue<std::string>() != std::string"
                "{\"/path/to/file\"}"};

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
                             Exception,
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
                             Exception,
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
                             Exception,
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

    return 0;
}
