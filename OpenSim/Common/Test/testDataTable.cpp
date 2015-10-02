/* -------------------------------------------------------------------------- *
 *                            OpenSim:  testDataTable.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
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

#include <OpenSim/Common/TimeSeriesTable.h>

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

    table.updTableMetaData().setValueForKey("DataRate", 600);
    table.updTableMetaData().setValueForKey("Filename", 
                                            std::string{"/path/to/file"});

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

    for(unsigned i = 0; i < 5; ++i) {
        for(unsigned j = 0; j < 5; ++j) {
            const auto row_i_1 = table.getRowAtIndex(i);
            if(row_i_1[j] != (row + i)[j])
                throw Exception{"Test failed: row_i_1[j] != (row + i)[j]"};

            const auto row_i_2 = table.getRow(0 + 0.25 * i);
            if(row_i_2[j] != (row + i)[j])
                throw Exception{"Test failed: row_i_2[j] != (row + i)[j]"};

            if(table.getDependentColumnAtIndex(i)[j] != j)
                throw Exception{"Test failed: table.getDependentColumnAtIndex"
                        "(i)[j] != j"};
        }
    }

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

    return 0;
}
