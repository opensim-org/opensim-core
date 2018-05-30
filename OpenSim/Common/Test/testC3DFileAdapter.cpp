/* -------------------------------------------------------------------------- *
 *                       OpenSim:  testC3DFileAdapter.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

#include <vector>
#include <unordered_map>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <cmath>

template<typename ETY = SimTK::Real>
void compare_tables(const OpenSim::TimeSeriesTable_<ETY>& table1,
            const OpenSim::TimeSeriesTable_<ETY>& table2,
            const double tolerance = SimTK::SignificantReal) {
    using namespace OpenSim;
    try {
        OPENSIM_THROW_IF(table1.getColumnLabels() != table2.getColumnLabels(),
                         Exception,
                         "Column labels are not the same for tables.");

        ASSERT_EQUAL( table1.getIndependentColumn(), 
                      table2.getIndependentColumn(), tolerance,
                       __FILE__, __LINE__,
                         "Independent columns are not equivalent.");
    } catch (const OpenSim::KeyNotFound&) {}

    const auto& matrix1 = table1.getMatrix();
    const auto& matrix2 = table2.getMatrix();

    for(int r = 0; r < matrix1.nrow(); ++r)
        for(int c = 0; c < matrix1.ncol(); ++c) {
            auto elt1 = matrix1.getElt(r, c); 
            auto elt2 = matrix2.getElt(r, c);

            ASSERT_EQUAL(elt1, elt2, tolerance, __FILE__, __LINE__,
                "Elements at row, " + std::to_string(r) + " col, " +
                std::to_string(c) + " failed to have matching value.");
        }
}


void test(const std::string filename) {
    using namespace OpenSim;
    
    std::clock_t startTime = std::clock();
    auto tables = C3DFileAdapter::read(filename,
        C3DFileAdapter::ForceLocation::Origin);

    std::cout << "\nC3DFileAdapter '" << filename << "' read time = " 
        << 1.e3*(std::clock() - startTime) / CLOCKS_PER_SEC 
        << "ms\n" << std::endl;

    auto& marker_table = tables.at("markers");
    auto&  force_table = tables.at("forces");

    size_t ext = filename.rfind(".");
    std::string base = filename.substr(0, ext);

    const std::string marker_file = base + "_markers.trc";
    const std::string forces_file = base + "_grfs.sto";

    ASSERT(marker_table->getNumRows() > 0, __FILE__, __LINE__,
        "Failed to read marker data from " + filename);

    marker_table->updTableMetaData().setValueForKey("Units", 
                                                    std::string{"mm"});
    TRCFileAdapter trc_adapter{};
    trc_adapter.write(*marker_table, marker_file);

    ASSERT(force_table->getNumRows() > 0, __FILE__, __LINE__,
        "Failed to read forces data from " + filename);

    force_table->updTableMetaData().setValueForKey("Units", 
                                                    std::string{"mm"});
    STOFileAdapter sto_adapter{};
    sto_adapter.write((force_table->flatten()), forces_file);

    // Verify that marker data was written out and can be read in
    auto markers = trc_adapter.read(marker_file);
    auto std_markers = trc_adapter.read("std_" + marker_file);
    // Compare C3DFileAdapter read-in and written marker data
    compare_tables<SimTK::Vec3>(markers, *marker_table);
    // Compare C3DFileAdapter written marker data to standard
    // Note std exported from Mokka with only 5 decimal places 
    compare_tables<SimTK::Vec3>(markers, std_markers, 1e-4);

    std::cout << marker_file << " equivalent to std_"
        << marker_file << std::endl;

    // Verify that grfs data was written out and can be read in
    auto forces = sto_adapter.read(forces_file);
    auto std_forces = sto_adapter.read("std_" + forces_file);
    // Compare C3DFileAdapter read-in and written forces data
    compare_tables<SimTK::Vec3>(forces.pack<SimTK::Vec3>(), *force_table);
    // Compare C3DFileAdapter written forces data to standard
    // Note std generated using MATLAB C3D processing scripts 
    compare_tables(forces, std_forces, SimTK::SqrtEps);

    std::cout << forces_file << " equivalent to std_"
        << forces_file << std::endl;

    auto tables2 = C3DFileAdapter::read(filename,
        C3DFileAdapter::ForceLocation::COP);
    auto& force_table_cop = tables2.at("forces");

    sto_adapter.write(force_table_cop->flatten(), "cop_"+ forces_file);

    auto std_forces_cop = sto_adapter.read("std_cop_" + forces_file);
    // Compare C3DFileAdapter written forces data to standard
    // Note std generated using MATLAB C3D processing scripts 
    compare_tables<SimTK::Vec3>(*force_table_cop, 
                                std_forces_cop.pack<SimTK::Vec3>(),
                                SimTK::SqrtEps);

    std::cout << forces_file << " converted to COP is equivalent to std_cop_"
        << forces_file << std::endl;

    std::cout << "\ntestC3DFileAdapter '" << filename << "' completed in "
        << 1.e3*(std::clock() - startTime) / CLOCKS_PER_SEC
        << "ms\n" << std::endl;
}

int main() {
    std::vector<std::string> filenames{};
    filenames.push_back("walking2.c3d");
    filenames.push_back("walking5.c3d");

    for(const auto& filename : filenames) {
        std::cout << "Test reading '" + filename + "'." << std::endl;
        try {
            test(filename);
        }
        catch (const std::exception& ex) {
            std::cout << "testC3DFileAdapter FAILED: " << ex.what() << std::endl;
            return 1;
        }
    }

    std::cout << "\nAll testC3DFileAdapter cases passed." << std::endl;

    return 0;
}
