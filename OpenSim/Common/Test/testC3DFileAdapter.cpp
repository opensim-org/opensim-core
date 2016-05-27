/* -------------------------------------------------------------------------- *
 *                            OpenSim:  testC3DFileAdapter.cpp                *
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

#include "OpenSim/Common/C3DFileAdapter.h"

#include "OpenSim/Common/TRCFileAdapter.h"

#include <vector>
#include <unordered_map>
#include <cstdlib>
#include <chrono>
#include <thread>


void compare_tables(const OpenSim::TimeSeriesTableVec3& table1,
                    const OpenSim::TimeSeriesTableVec3& table2) {
    using namespace OpenSim;
    OPENSIM_THROW_IF(table1.getColumnLabels() != table2.getColumnLabels(),
                     Exception,
                     "Column labels are not the same for tables.");
    OPENSIM_THROW_IF(table1.getIndependentColumn() != 
                     table2.getIndependentColumn(),
                     Exception,
                     "Independent columns are not the same for tables.");
    const auto& matrix1 = table1.getMatrix();
    const auto& matrix2 = table2.getMatrix();
    for(int r = 0; r < matrix1.nrow(); ++r)
        for(int c = 0; c < matrix1.ncol(); ++c)
            OPENSIM_THROW_IF(matrix1.getElt(r, c) != 
                             matrix2.getElt(r, c),
                             Exception,
                             "Matrices are not the same for tables.");
}


void test(const std::string filename) {
    using namespace OpenSim;
    auto tables = C3DFileAdapter::read(filename);

    auto& marker_table = tables.at("markers");
    auto&  force_table = tables.at("forces");

    {
        using namespace OpenSim;
        using MT = TimeSeriesTableVec3;
        using FT = TimeSeriesTableVec3;

        MT marker_table1{filename, "markers"};
        FT  force_table1{filename, "forces"};

        compare_tables(*marker_table, marker_table1);
        compare_tables( *force_table,  force_table1);
    }

    if(marker_table->getNumRows() != 0) {
        marker_table->updTableMetaData().setValueForKey("Units", 
                                                        std::string{"mm"});
        TRCFileAdapter trc_adapter{};
        trc_adapter.write(*marker_table, filename + ".markers.trc");
    }

    if(force_table->getNumRows() != 0) {
        force_table->updTableMetaData().setValueForKey("Units", 
                                                       std::string{"mm"});
        TRCFileAdapter trc_adapter{};
        trc_adapter.write(*force_table, filename + ".forces.trc");
    }
}

int main() {
    std::vector<std::string> filenames{};
    filenames.push_back("jogging.c3d");
    filenames.push_back("singleLeglanding_2.c3d");
    filenames.push_back("aStaticTrial_2.c3d");
    filenames.push_back("aStaticTrial.c3d");
    filenames.push_back("doubleLegLanding.c3d");
    filenames.push_back("sideCutting.c3d");
    filenames.push_back("singleLegLanding.c3d");
    filenames.push_back("walking2.c3d");
    filenames.push_back("walking5.c3d");

    for(const auto& filename : filenames) {
        test(filename);
    }

    return 0;
}
