/* -------------------------------------------------------------------------- *
 *                            OpenSim:  exampleTRCFileAdapter.cpp             *
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

#include "OpenSim/Common/Adapters.h"

int main() {
    std::string filename{"dataWithNaNsOfDifferentCases.trc"};

    // There are two ways to read the file:
    // (1) Use the specific adapter to read the file. This requires you to know
    //     the format of the file. Use a TRCFileAdapter to read TRC file.
    // (2) Use the table constructor to read the file and let it take care
    //     of picking the right Adapter for the given file format.

    // First method. Use the TRCFileAdapter. The result is a collection
    // of tables. For a TRC file, this collection will contain just one table. 
    // This table will be an AbstractDataTable. It will have to be casted to the
    //  concrete type to access the data.
    OpenSim::TRCFileAdapter trcfileadapter{};
    auto abstable1 = trcfileadapter.read(filename).at("markers");
    OpenSim::TimeSeriesTableVec3& table1 = *(static_cast<OpenSim::TimeSeriesTableVec3*>(abstable1.get()));
    // Second method. Knowing that the file contains TimeSeriesTableVec3, 
    // create a TimeSeriesTableVec3 instance using the table's constructor
    auto table2 = OpenSim::TimeSeriesTableVec3(filename);

    // From this point on, both table1 and table2 represent the same type of
    // DataTable and so both support the same operations. Below code 
    // demonstrates operations on only table1 but they are applicable to table2
    // as well. 

    // Metadata of the table.
    std::cout << table1.
                 getTableMetaData().
                 getValueForKey("DataRate").
                 getValue<std::string>() << std::endl;

    std::cout << table1.
                 getTableMetaData().
                 getValueForKey("Units").
                 getValue<std::string>() << std::endl;

    // Column labels of the table.
    auto& labels = table1.
                   getDependentsMetaData().
                   getValueArrayForKey("labels");
    for(size_t i = 0; i < labels.size(); ++i)
        std::cout << labels[i].getValue<std::string>() << "\t";
    std::cout << std::endl;

    // Dimensions of the table.
    std::cout << table1.getNumRows() << "\t"
              << table1.getNumColumns() << std::endl;

    // Individual rows of the table.
    for(size_t i = 0; i < table1.getNumRows(); ++i)
        std::cout << table1.getRowAtIndex(i) << std::endl;

    // See documentation for TimeSeriesTable for full set of operations
    // possible for table1 and table2.

    return 0;
}
