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
#include "OpenSim/Common/STOFileAdapter.h"
#include "OpenSim/Common/TRCFileAdapter.h"
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <thread>
#include <unordered_map>
#include <vector>

#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/Common/Stopwatch.h>

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
                "Element at row, " + std::to_string(r) + " col, " +
                std::to_string(c) + " failed to have matching value.");
        }
}

template<typename ETY = SimTK::Real>
void downsample_table(OpenSim::TimeSeriesTable_<ETY>& table,
    const unsigned int increment) {
    for (size_t r = table.getNumRows() - 2; r > 0; --r) {
        if (r%increment)
            table.removeRowAtIndex(r);
    }
}


void test(const std::string filename) {
    using namespace OpenSim;
    using namespace std;

    // The walking C3D files included in this test should not take more
    // than 200ms (ezc3d) on most hardware.
    // We make the max time 200ms to account for potentially slower CI machines.
    const long long MaximumLoadTimeInNs = SimTK::secToNs(0.200);
    
    Stopwatch watch;
    C3DFileAdapter c3dFileAdapter{};
    auto tables = c3dFileAdapter.read(filename);
    long long loadTime = watch.getElapsedTimeInNs();

    cout << "\tC3DFileAdapter '" << filename << "' loaded in " 
        << watch.formatNs(loadTime) << endl;

/*  Disabled performance test because Travis CI is consistently unable to
    meet this timing requirement. Consider PR#2221 to address this issue
    longer term.
    #ifdef NDEBUG
    ASSERT(loadTime < MaximumLoadTimeInNs, __FILE__, __LINE__,
        "Unable to load '" + filename + "' within " + 
        to_string(MaximumLoadTimeInNs) + "ns.");
    #endif
*/

    std::shared_ptr<TimeSeriesTableVec3> marker_table = c3dFileAdapter.getMarkersTable(tables);
    std::shared_ptr<TimeSeriesTableVec3> force_table = c3dFileAdapter.getForcesTable(tables);
    downsample_table(*marker_table, 10);
    downsample_table(*force_table, 100);

    size_t ext = filename.rfind(".");
    std::string base = filename.substr(0, ext);

    const std::string marker_file = base + "_markers.trc";
    const std::string forces_file = base + "_grfs.sto";

    ASSERT(marker_table->getNumRows() > 0, __FILE__, __LINE__,
        "Failed to read marker data from " + filename);

    marker_table->updTableMetaData().setValueForKey("Units", 
                                                    std::string{"mm"});
    TRCFileAdapter trc_adapter{};
    watch.reset();
    trc_adapter.write(*marker_table, marker_file);
    cout << "\tWrote '" << marker_file << "' in "
        << watch.getElapsedTimeFormatted() << endl;

    ASSERT(force_table->getNumRows() > 0, __FILE__, __LINE__,
        "Failed to read forces data from " + filename);

    force_table->updTableMetaData().setValueForKey("Units", 
                                                    std::string{"mm"});
    STOFileAdapter sto_adapter{};
    watch.reset();
    sto_adapter.write((force_table->flatten()), forces_file);
    cout << "\tWrote'" << forces_file << "' in "
        << watch.getElapsedTimeFormatted() << endl;

    // Verify that marker data was written out and can be read in
    watch.reset();
    TimeSeriesTable_<SimTK::Vec3> markers(marker_file);
    TimeSeriesTable_<SimTK::Vec3> std_markers("std_" + marker_file);
    cout << "\tRead'" << marker_file << "' and its standard in "
        << watch.getElapsedTimeFormatted() << endl;

    // Compare C3DFileAdapter read-in and written marker data
    compare_tables<SimTK::Vec3>(markers, *marker_table);
    // Compare C3DFileAdapter written marker data to standard
    // Note std exported from Mokka with only 5 decimal places 
    compare_tables<SimTK::Vec3>(markers, std_markers, 1e-4);

    cout << "\tMarkers " << marker_file << " equivalent to standard." << endl;

    // Verify that grfs data was written out and can be read in
    TimeSeriesTable forces(forces_file);
    TimeSeriesTable std_forces("std_" + forces_file);
    // Compare C3DFileAdapter read-in and written forces data
    compare_tables<SimTK::Vec3>(forces.pack<SimTK::Vec3>(), 
                                *force_table,
                                SimTK::SqrtEps);
    // Compare C3DFileAdapter written forces data to standard
    // Note std generated using MATLAB C3D processing scripts 
    compare_tables(forces, std_forces, SimTK::SqrtEps);

    cout << "\tForces " << forces_file << " equivalent to standard." << endl;

    watch.reset();
    // Reread in C3D file with forces resolved to the COP
    auto c3dFileAdapter2 = C3DFileAdapter{};
    c3dFileAdapter2.setLocationForForceExpression(
            C3DFileAdapter::ForceLocation::CenterOfPressure);
    auto tables2 = c3dFileAdapter2.read(filename);
    
    loadTime = watch.getElapsedTimeInNs();
    cout << "\tC3DFileAdapter '" << filename << "' read with forces at COP in "
        << watch.formatNs(loadTime) << endl;

    std::shared_ptr<TimeSeriesTableVec3> force_table_cop =
            c3dFileAdapter.getForcesTable(tables2);
    downsample_table(*force_table_cop, 100);

    sto_adapter.write(force_table_cop->flatten(), "cop_"+ forces_file);

    TimeSeriesTable std_forces_cop("std_cop_" + forces_file);
    // Compare C3DFileAdapter written forces data to standard
    // Note std generated using MATLAB C3D processing scripts 
    compare_tables<SimTK::Vec3>(*force_table_cop, 
                                std_forces_cop.pack<SimTK::Vec3>(),
                                SimTK::SqrtEps);

    cout << "\tcop_" << forces_file << " is equivalent to its standard."<< endl;
}

int main() {
    SimTK_START_TEST("testC3DFileAdapter");
        SimTK_SUBTEST1(test, "walking2.c3d");
        SimTK_SUBTEST1(test, "walking5.c3d");
    SimTK_END_TEST();
}
