/* -------------------------------------------------------------------------- *
 *                       OpenSim:  testIMUHelper.cpp                          *
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

#include "OpenSim/Common/DataAdapter.h"
#include "OpenSim/Common/IMUHelper.h"
#include "OpenSim/Common/STOFileAdapter.h"


using namespace OpenSim;
/**
    auto& marker_table = tables.at("markers");
    auto&  force_table = tables.at("forces");
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
    std::clock_t t0 = std::clock();
    trc_adapter.write(*marker_table, marker_file);
    cout << "\tWrote '" << marker_file << "' in "
        << 1.e3*(std::clock() - t0) / CLOCKS_PER_SEC << "ms" << endl;

    ASSERT(force_table->getNumRows() > 0, __FILE__, __LINE__,
        "Failed to read forces data from " + filename);

    force_table->updTableMetaData().setValueForKey("Units", 
                                                    std::string{"mm"});
    STOFileAdapter sto_adapter{};
    t0 = std::clock();
    sto_adapter.write((force_table->flatten()), forces_file);
    cout << "\tWrote'" << forces_file << "' in "
        << 1.e3*(std::clock() - t0) / CLOCKS_PER_SEC << "ms" << endl;

    // Verify that marker data was written out and can be read in
    t0 = std::clock();
    auto markers = trc_adapter.read(marker_file);
    auto std_markers = trc_adapter.read("std_" + marker_file);
    cout << "\tRead'" << marker_file << "' and its standard in "
        << 1.e3*(std::clock() - t0) / CLOCKS_PER_SEC << "ms" << endl;

    // Compare C3DFileAdapter read-in and written marker data
    compare_tables<SimTK::Vec3>(markers, *marker_table);
    // Compare C3DFileAdapter written marker data to standard
    // Note std exported from Mokka with only 5 decimal places 
    compare_tables<SimTK::Vec3>(markers, std_markers, 1e-4);

    cout << "\tMarkers " << marker_file << " equivalent to standard." << endl;

    // Verify that grfs data was written out and can be read in
    auto forces = sto_adapter.read(forces_file);
    auto std_forces = sto_adapter.read("std_" + forces_file);
    // Compare C3DFileAdapter read-in and written forces data
    compare_tables<SimTK::Vec3>(forces.pack<SimTK::Vec3>(), 
                                *force_table,
                                SimTK::SqrtEps);
    // Compare C3DFileAdapter written forces data to standard
    // Note std generated using MATLAB C3D processing scripts 
    compare_tables(forces, std_forces, SimTK::SqrtEps);

    cout << "\tForces " << forces_file << " equivalent to standard." << endl;

    
    t0 = std::clock();
    // Reread in C3D file with forces resolved to the COP 
    auto tables2 = C3DFileAdapter::read(filename,
        C3DFileAdapter::ForceLocation::CenterOfPressure);
    
    loadTime = 1.e3*(std::clock() - t0) / CLOCKS_PER_SEC;
    cout << "\tC3DFileAdapter '" << filename << "' read with forces at COP in "
        << loadTime << "ms" << endl;

    #ifdef NDEBUG
    ASSERT(loadTime < MaximumLoadTimeInMS, __FILE__, __LINE__,
        "Unable to load '" + filename + "' within " +
        to_string(MaximumLoadTimeInMS) + "ms.");
    #endif

    auto& force_table_cop = tables2.at("forces");
    downsample_table(*force_table_cop, 100);

    sto_adapter.write(force_table_cop->flatten(), "cop_"+ forces_file);

    auto std_forces_cop = sto_adapter.read("std_cop_" + forces_file);
    // Compare C3DFileAdapter written forces data to standard
    // Note std generated using MATLAB C3D processing scripts 
    compare_tables<SimTK::Vec3>(*force_table_cop, 
                                std_forces_cop.pack<SimTK::Vec3>(),
                                SimTK::SqrtEps);

    cout << "\tcop_" << forces_file << " is equivalent to its standard."<< endl;

    cout << "\ttestIMUHelper '" << filename << "' completed in "
        << 1.e3*(std::clock() - startTime) / CLOCKS_PER_SEC  << "ms" << endl;
        */

int main() {

    try {
        std::map<std::string, std::string> mapFileNameToIMUName;
        mapFileNameToIMUName["00B421AF"] = "shank";
        mapFileNameToIMUName["00B4227B"] = "thigh";
        mapFileNameToIMUName["00B42263"] = "toe";

        const std::string folder = "C:/Users/Ayman-NMBL/Downloads/IMU/IMU/";
        const std::string trial = "MT_012005D6_025-000_";
        DataAdapter::OutputTables tables = IMUHelper::readXsensTrial(folder, trial, mapFileNameToIMUName);
        // Write tables to sto files
        // Accelerations
        std::shared_ptr<AbstractDataTable> accelTable = tables.at(IMUHelper::_accelerations);
        const TimeSeriesTableVec3& accelTableTyped = dynamic_cast<const TimeSeriesTableVec3&>(*accelTable);
        STOFileAdapterVec3::write(accelTableTyped, folder + trial+ "accelerations.sto");
        // Magenometer
        std::shared_ptr<AbstractDataTable> magTable = tables.at(IMUHelper::_magnetometers);
        const TimeSeriesTableVec3& magTableTyped = dynamic_cast<const TimeSeriesTableVec3&>(*magTable);
        STOFileAdapterVec3::write(magTableTyped, folder + trial + "magnetometers.sto");
        // Gyro
        std::shared_ptr<AbstractDataTable> gyroTable = tables.at(IMUHelper::_gyros);
        const TimeSeriesTableVec3& gyroTableTyped = dynamic_cast<const TimeSeriesTableVec3&>(*gyroTable);
        STOFileAdapterVec3::write(gyroTableTyped, folder + trial + "gyros.sto");
        // Orientation
        std::shared_ptr<AbstractDataTable> orientationTable = tables.at(IMUHelper::_orientations);
        const TimeSeriesTableQuaternion& quatTableTyped = dynamic_cast<const TimeSeriesTableQuaternion&>(*orientationTable);
        STOFileAdapterQuaternion::write(quatTableTyped, folder + trial + "quaternions.sto");

    }
    catch (const std::exception& ex) {
        std::cout << "testIMUHelper FAILED: " << ex.what() << std::endl;
        return 1;
    }

    std::cout << "\nAll testIMUHelper cases passed." << std::endl;

    return 0;
}
