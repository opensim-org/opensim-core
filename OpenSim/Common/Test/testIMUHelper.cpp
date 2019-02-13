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
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>


using namespace OpenSim;
/* Raw data from 00B421AF
PacketCounter	SampleTimeFine	Year	Month	Day	Second	UTC_Nano	UTC_Year	UTC_Month	UTC_Day	UTC_Hour	UTC_Minute	UTC_Second	UTC_Valid	Acc_X	Acc_Y	Acc_Z	Gyr_X	Gyr_Y	Gyr_Z	Mag_X	Mag_Y	Mag_Z	Mat[1][1]	Mat[2][1]	Mat[3][1]	Mat[1][2]	Mat[2][2]	Mat[3][2]	Mat[1][3]	Mat[2][3]	Mat[3][3]
03583														3.030769	5.254238	-7.714005	0.005991	-0.032133	0.022713	-0.045410	-0.266113	0.897217	0.609684	0.730843	0.306845	0.519480	-0.660808	0.541732	0.598686	-0.170885	-0.782543
*/

int main() {

    try {
        std::map<std::string, std::string> mapFileNameToIMUName;
        std::vector<std::string> imu_names{ "shank", "thigh", "calcn", "toe" };
        mapFileNameToIMUName["00B421AF"] = imu_names[0];
        mapFileNameToIMUName["00B4227B"] = imu_names[1];
        mapFileNameToIMUName["00B42263"] = imu_names[2];
        mapFileNameToIMUName["00B42268"] = imu_names[3];

        const std::string folder = "";
        const std::string trial = "MT_012005D6_031-000_";
        DataAdapter::OutputTables tables = IMUHelper::readXsensTrial(folder, trial, mapFileNameToIMUName);
        // Write tables to sto files
        // Accelerations
        std::shared_ptr<AbstractDataTable> accelTable = tables.at(IMUHelper::_accelerations);
        const TimeSeriesTableVec3& accelTableTyped = dynamic_cast<const TimeSeriesTableVec3&>(*accelTable);
        STOFileAdapterVec3::write(accelTableTyped, folder + trial+ "accelerations.sto");
        const SimTK::RowVectorView_<SimTK::Vec3>& rvv = accelTableTyped.getRowAtIndex(0);
        SimTK::Vec3 fromTable = accelTableTyped.getRowAtIndex(0)[0];
        SimTK::Vec3 fromFile = SimTK::Vec3{ 3.030769, 5.254238, -7.714005 };
        double tolerance = SimTK::Eps;
        ASSERT_EQUAL(fromTable,fromFile, tolerance);
        // Magenometer
        std::shared_ptr<AbstractDataTable> magTable = tables.at(IMUHelper::_magnetometers);
        const TimeSeriesTableVec3& magTableTyped = dynamic_cast<const TimeSeriesTableVec3&>(*magTable);
        STOFileAdapterVec3::write(magTableTyped, folder + trial + "magnetometers.sto");
        fromTable = magTableTyped.getRowAtIndex(0)[0];
        fromFile = SimTK::Vec3{ -0.045410, - 0.266113, 0.897217 };
        ASSERT_EQUAL(fromTable, fromFile, tolerance);
        // Gyro
        std::shared_ptr<AbstractDataTable> gyroTable = tables.at(IMUHelper::_gyros);
        const TimeSeriesTableVec3& gyroTableTyped = dynamic_cast<const TimeSeriesTableVec3&>(*gyroTable);
        STOFileAdapterVec3::write(gyroTableTyped, folder + trial + "gyros.sto");
        fromTable = gyroTableTyped.getRowAtIndex(0)[0];
        fromFile = SimTK::Vec3{ 0.005991, - 0.032133, 0.022713 };
        ASSERT_EQUAL(fromTable, fromFile, tolerance);
        // Orientation
        std::shared_ptr<AbstractDataTable> orientationTable = tables.at(IMUHelper::_orientations);
        const TimeSeriesTableQuaternion& quatTableTyped = dynamic_cast<const TimeSeriesTableQuaternion&>(*orientationTable);
        STOFileAdapterQuaternion::write(quatTableTyped, folder + trial + "quaternions.sto");

    }
    catch (const std::exception& ex) {
        std::cout << "testIMUHelper FAILED: " << ex.what() << std::endl;
        return 1;
    }

    std::cout << "\n All testIMUHelper cases passed." << std::endl;

    return 0;
}
