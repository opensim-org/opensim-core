/* -------------------------------------------------------------------------- *
 *                            OpenSim:  example2.cpp                          *
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

#include "OpenSim/Common/TimeSeriesTable.h"
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <iostream>

// This example demonstrates creating TimeSeriesTable from a DataTable.
int main() {
    using namespace OpenSim;

    DataTable dataTable{};

    // Add column labels to the table. 
    dataTable.setColumnLabels({"0", "1", "2", "3", "4"});

    // Append rows to the table.

    SimTK::RowVector_<double> row0{5, double{0}};
    
    dataTable.appendRow(0.00, row0);

    auto row1 = row0 + 1;

    dataTable.appendRow(0.25, row1);

    auto row2 = row1 + 1;

    dataTable.appendRow(0.50, row2);

    auto row3 = row2 + 1;

    dataTable.appendRow(0.75, row3);
    
    auto row4 = row3 + 1;

    dataTable.appendRow(1.00, row4);

    // Following construction of TimeSeriesTable succeeds because the DataTable
    // has its independent column strictly increasing.
    TimeSeriesTable timeseries_table1{dataTable};

    // Editing the DataTable to not have strictly increasing independent column
    // will fail the construction of TimeSeriesTable.
    auto row5 = row4 + 1;
    
    dataTable.appendRow(0.9, row5); // 0.9 is less than previous value (1.00).

    // Print out the DataTable to console. Use this for debugging only.
    std::cout << dataTable << std::endl;

    // Construction of TimeSeriesTable fails because independent column is not
    // strictly increasing. 
    ASSERT_THROW(OpenSim::Exception,
                 TimeSeriesTable timeseries_table2{dataTable});

    // Edit the entry in the independent column to make the column strictly
    // increasing.
    dataTable.setIndependentValueAtIndex(5, 1.25);
    // TimeSeriesTable construction now successful. 
    TimeSeriesTable timeseries_table3{dataTable};

    // Print out the table to console. Use this for debugging only.
    std::cout << timeseries_table3 << std::endl;

    return 0;
}
