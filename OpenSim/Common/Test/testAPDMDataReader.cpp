/* -------------------------------------------------------------------------- *
 *                       OpenSim:  testAPDMDataReader.cpp                    *
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
#include "OpenSim/Common/APDMDataReader.h"
#include "OpenSim/Common/STOFileAdapter.h"
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>


using namespace OpenSim;


int main() {

    try {
        APDMDataReaderSettings readerSettings;
        std::vector<std::string> imu_names{ "torso", "pelvis" };
        std::vector<std::string> names_in_experiment{ "Static", "Middle" };
        // Programmatically add items to name mapping, write to xml
        for (int index = 0; index < imu_names.size(); ++index) {
            ExperimentalSensor  nextSensor(names_in_experiment[index], imu_names[index]);
            readerSettings.append_ExperimentalSensors(nextSensor);
        }
        readerSettings.print("apdm_reader.xml");
        // read xml we wrote into a new APDMDataReader to readTrial
        APDMDataReaderSettings reconstructFromXML("apdm_reader.xml");
        APDMDataReader reader(reconstructFromXML);
        DataAdapter::OutputTables tables = reader.extendRead("imuData01csv.csv");
        // Write tables to sto files
        // Accelerations
        const TimeSeriesTableVec3& accelTableTyped =
            APDMDataReader::getLinearAccelerationsTable(tables);
        STOFileAdapterVec3::write(accelTableTyped, "accelerations.sto");
        const SimTK::RowVectorView_<SimTK::Vec3>& rvv = accelTableTyped.getRowAtIndex(0);
        SimTK::Vec3 fromTable = accelTableTyped.getRowAtIndex(0)[0];
        SimTK::Vec3 fromFile = SimTK::Vec3{ 0.102542184,0.048829611,9.804986382 };
        double tolerance = SimTK::Eps;
        ASSERT_EQUAL(fromTable, fromFile, tolerance);
        // test last row as well to make sure all data is read correctly, 
        // size is as expected
        size_t numRows = accelTableTyped.getIndependentColumn().size();
        ASSERT(numRows==1024);
        fromTable = accelTableTyped.getRowAtIndex(numRows - 1)[0];
        fromFile = SimTK::Vec3{ 0.158696249,0.298471016,9.723807335 };
        ASSERT_EQUAL(fromTable, fromFile, tolerance);
        // Magenometer
        const TimeSeriesTableVec3& magTableTyped =
            APDMDataReader::getMagneticHeadingTable(tables);
        STOFileAdapterVec3::write(magTableTyped, "magnetometers.sto");
        fromTable = magTableTyped.getRowAtIndex(0)[0];
        fromFile = SimTK::Vec3{ 31.27780876,13.46964874,-62.79244003 };
        ASSERT_EQUAL(fromTable, fromFile, tolerance);
        // Gyro
        const TimeSeriesTableVec3& gyroTableTyped =
            APDMDataReader::getAngularVelocityTable(tables);
        STOFileAdapterVec3::write(gyroTableTyped, "gyros.sto");
        fromTable = gyroTableTyped.getRowAtIndex(0)[0];
        fromFile = SimTK::Vec3{ 0.002136296, 0.008331553,-0.008972442 };
        ASSERT_EQUAL(fromTable, fromFile, tolerance);
        // Orientation
        const TimeSeriesTableQuaternion& quatTableTyped =
            APDMDataReader::getOrientationsTable(tables);
        STOFileAdapterQuaternion::write(quatTableTyped, "quaternions.sto");
        SimTK::Quaternion quatFromTable = quatTableTyped.getRowAtIndex(0)[0];
        SimTK::Quaternion quatFromFile = SimTK::Quaternion(0.979286375, 0.000865605, -0.005158994, -0.202412525);
        ASSERT_EQUAL(quatFromTable, quatFromFile, 1e-6);
     }
    catch (const std::exception& ex) {
        std::cout << "testAPDMDataReader FAILED: " << ex.what() << std::endl;
        return 1;
    }

    std::cout << "\n All testAPDMDataReader cases passed." << std::endl;

    return 0;
}
