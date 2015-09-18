/* -------------------------------------------------------------------------- *
 *                            OpenSim:  DataTable.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
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
    assert(table.getNumRows() == unsigned{5});
    assert(table.getNumColumns() == unsigned{5});

    const auto& dep_metadata_ref = table.getDependentsMetaData();

    const auto& labels_ref = dep_metadata_ref.getValueArrayForKey("labels");
    for(unsigned i = 0; i < 5; ++i)
        assert(labels_ref[i].getValue<std::string>() == std::to_string(i + 1));

    const auto& col_index_ref 
        = dep_metadata_ref.getValueArrayForKey("column-index");
    for(unsigned i = 0; i < 5; ++i)
        assert(col_index_ref[i].getValue<unsigned>() == i + 1);

    const auto& ind_metadata_ref = table.getIndependentMetaData();
    
    assert(ind_metadata_ref.getValueForKey("labels").getValue<std::string>() 
           == std::string{"0"});
    assert(ind_metadata_ref.getValueForKey("column-index").getValue<unsigned>()
           == unsigned{0});

    for(unsigned i = 0; i < 5; ++i) {
        assert(table.getIndependentColumn()[i] == 0 + 0.25 * i);
        for(unsigned j = 0; j < 5; ++j) {
            assert(table.getRowAtIndex(i)[j] == (row + i)[j]);
            assert(table.getRow(0 + 0.25 * i)[j] == (row + i)[j]);
            assert(table.getDependentColumnAtIndex(i)[j] == j);
        }
    }

    const auto& tab_metadata_ref = table.getTableMetaData();
    assert(tab_metadata_ref.getValueForKey("DataRate").getValue<int>() 
           == 600);
    assert(tab_metadata_ref.getValueForKey("Filename").getValue<std::string>()
           == std::string{"/path/to/file"});

    return 0;
}
