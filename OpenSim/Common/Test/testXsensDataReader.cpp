/* -------------------------------------------------------------------------- *
 *                       OpenSim:  testXsensDataReader.cpp                    *
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
#include "OpenSim/Common/IMUDataReader.h"
#include "OpenSim/Common/XsensDataReader.h"
#include "OpenSim/Common/STOFileAdapter.h"
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

#include <catch2/catch_all.hpp>


using namespace OpenSim;
/* Raw data from 00B421AF
PacketCounter<tab>SampleTimeFine<tab>Year<tab>Month<tab>Day<tab>Second<tab>UTC_Nano<tab>UTC_Year<tab>UTC_Month<tab>UTC_Day<tab>UTC_Hour<tab>UTC_Minute<tab>UTC_Second<tab>UTC_Valid<tab>Acc_X<tab>Acc_Y<tab>Acc_Z<tab>Gyr_X<tab>Gyr_Y<tab>Gyr_Z<tab>Mag_X<tab>Mag_Y<tab>Mag_Z<tab>Mat[1][1]<tab>Mat[2][1]<tab>Mat[3][1]<tab>Mat[1][2]<tab>Mat[2][2]<tab>Mat[3][2]<tab>Mat[1][3]<tab>Mat[2][3]<tab>Mat[3][3]
03583<tab><tab><tab><tab><tab><tab><tab><tab><tab><tab><tab><tab><tab><tab>3.030769<tab>5.254238<tab>-7.714005<tab>0.005991<tab>-0.032133<tab>0.022713<tab>-0.045410<tab>-0.266113<tab>0.897217<tab>0.609684<tab>0.730843<tab>0.306845<tab>0.519480<tab>-0.660808<tab>0.541732<tab>0.598686<tab>-0.170885<tab>-0.782543
...
06951<tab><tab><tab><tab><tab><tab><tab><tab><tab><tab><tab><tab><tab><tab>2.657654<tab>5.012634<tab>-7.581414<tab>0.392058<tab>0.353193<tab>-0.712047<tab>-0.114258<tab>-0.155518<tab>0.913330<tab>0.387756<tab>0.877453<tab>0.282349<tab>0.716718<tab>-0.479624<tab>0.506238<tab>0.579621<tab>0.006068<tab>-0.814863
*/

TEST_CASE("XsensDataReader") {
    SECTION("Full Set Basic Processing") {
        XsensDataReaderSettings readerSettings;
        std::vector<std::string> imu_names{"shank", "thigh"};
        std::vector<std::string> file_names{"000_00B421AF", "000_00B4227B"};
        // Programmatically add items to Map, write to xml
        for (int index = 0; index < (int)imu_names.size(); ++index) {
            ExperimentalSensor nextSensor(file_names[index], imu_names[index]);
            readerSettings.append_ExperimentalSensors(nextSensor);
        }
        readerSettings.updProperty_trial_prefix() = "MT_012005D6_031-";
        readerSettings.print("reader2xml.xml");
        // read xml we wrote into a new XsensDataReader to readTrial
        XsensDataReader reconstructFromXML(
                XsensDataReaderSettings("reader2xml.xml"));
        DataAdapter::OutputTables tables = reconstructFromXML.read("./");
        std::string folder = readerSettings.get_data_folder();
        std::string trial = readerSettings.get_trial_prefix();
        // Write tables to sto files
        // Accelerations
        const TimeSeriesTableVec3& accelTableTyped =
                reconstructFromXML.getLinearAccelerationsTable(tables);
        STOFileAdapterVec3::write(
                accelTableTyped, folder + trial + "accelerations.sto");
        const SimTK::RowVectorView_<SimTK::Vec3>& rvv =
                accelTableTyped.getRowAtIndex(0);
        SimTK::Vec3 fromTable = accelTableTyped.getRowAtIndex(0)[0];
        SimTK::Vec3 fromFile = SimTK::Vec3{3.030769, 5.254238, -7.714005};
        double tolerance = SimTK::Eps;
        REQUIRE(fromTable.size() == fromFile.size());
        for (int i = 0; i < fromTable.size(); ++i) {
            REQUIRE_THAT(fromTable[i],
                    Catch::Matchers::WithinAbs(fromFile[i], tolerance));
        }
        // test last row as well to make sure all data is read correctly,
        // size is as expected
        int numRows = accelTableTyped.getIndependentColumn().size();
        fromTable = accelTableTyped.getRowAtIndex(numRows - 1)[0];
        fromFile = SimTK::Vec3{2.657654, 5.012634, -7.581414};
        REQUIRE(fromTable.size() == fromFile.size());
        for (int i = 0; i < fromTable.size(); ++i) {
            REQUIRE_THAT(fromTable[i],
                    Catch::Matchers::WithinAbs(fromFile[i], tolerance));
        }
        // Magenometer
        const TimeSeriesTableVec3& magTableTyped =
                reconstructFromXML.getMagneticHeadingTable(tables);
        STOFileAdapterVec3::write(
                magTableTyped, folder + trial + "magnetometers.sto");
        fromTable = magTableTyped.getRowAtIndex(0)[0];
        fromFile = SimTK::Vec3{-0.045410, -0.266113, 0.897217};
        REQUIRE(fromTable.size() == fromFile.size());
        for (int i = 0; i < fromTable.size(); ++i) {
            REQUIRE_THAT(fromTable[i],
                    Catch::Matchers::WithinAbs(fromFile[i], tolerance));
        }
        // ASSERT_EQUAL(fromTable, fromFile, tolerance);
        // Gyro
        const TimeSeriesTableVec3& gyroTableTyped =
                reconstructFromXML.getAngularVelocityTable(tables);
        STOFileAdapterVec3::write(gyroTableTyped, folder + trial + "gyros.sto");
        fromTable = gyroTableTyped.getRowAtIndex(0)[0];
        fromFile = SimTK::Vec3{0.005991, -0.032133, 0.022713};
        REQUIRE(fromTable.size() == fromFile.size());
        for (int i = 0; i < fromTable.size(); ++i) {
            REQUIRE_THAT(fromTable[i],
                    Catch::Matchers::WithinAbs(fromFile[i], tolerance));
        }
        // Orientation
        const TimeSeriesTableQuaternion& quatTableTyped =
                reconstructFromXML.getOrientationsTable(tables);
        STOFileAdapterQuaternion::write(
                quatTableTyped, folder + trial + "quaternions.sto");
        SimTK::Quaternion quat = quatTableTyped.getRowAtIndex(0)[1];
        // Convert back to orientation matrix and compare with gold standard
        // data in file
        //-0.444898<tab>0.895542<tab>0.008444
        // 0.333934<tab>0.157132<tab>0.929407
        // 0.830996<tab>0.416311<tab>-0.368959
        std::vector<double> rotationVectorInFile{-0.444898, 0.895542, 0.008444,
                0.333934, 0.157132, 0.929407, 0.830996, 0.416311, -0.368959};
        SimTK::Rotation rot;
        rot.setRotationFromQuaternion(quat);
        tolerance =
                1e-6; // empirically determined due to conversion back and forth
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                // Matrix is stored column major
                REQUIRE_THAT(rotationVectorInFile[i * 3 + j],
                        Catch::Matchers::WithinAbs(rot[j][i], tolerance));
            }
        }
    }
    SECTION("Only Orientation") {
        // Now test the case where only orientation data is available, rest is
        // missing
        XsensDataReaderSettings readOrientationsOnly;
        ExperimentalSensor nextSensor("000_00B421ED", "test");
        readOrientationsOnly.append_ExperimentalSensors(nextSensor);
        readOrientationsOnly.updProperty_trial_prefix() =
                "MT_012005D6-000_sit_to_stand-";
        DataAdapter::OutputTables tables2 =
                XsensDataReader(readOrientationsOnly).read("./");
        const auto accelTable2 =
                tables2.at(XsensDataReader::LinearAccelerations);
        CHECK(accelTable2->getNumRows() == 0);
        CHECK(tables2.at(IMUDataReader::MagneticHeading)->getNumRows() == 0);
        CHECK(tables2.at(IMUDataReader::AngularVelocity)->getNumRows() == 0);
        // Now a file with extra comment in header as reported by user, has 3
        // rows
        XsensDataReaderSettings readerSettings3;
        ExperimentalSensor nextSensor2("test1_00B421E6", "test");
        readerSettings3.append_ExperimentalSensors(nextSensor2);
        DataAdapter::OutputTables tables3 =
                XsensDataReader(readerSettings3).read("./");
        auto accelTable3 = tables3.at(XsensDataReader::LinearAccelerations);
        CHECK(accelTable3->getNumRows() == 3);
    }
    SECTION("Exported with Current (2022-2025) Format") {
        // Now a file with latest format
        XsensDataReaderSettings readerSettings4;
        ExperimentalSensor nextSensor4("MT_01200454_000-000_00B40DE4", "test");
        readerSettings4.append_ExperimentalSensors(nextSensor4);
        DataAdapter::OutputTables tables4 =
                XsensDataReader(readerSettings4).read("./");
        auto accelTable4 = tables4.at(XsensDataReader::LinearAccelerations);
        CHECK(accelTable4->getNumRows() == 4);
    }
    SECTION("Exported from MTManager2020.0.2") {
        // Now a file exported from MTManager2020.0.2
        XsensDataReaderSettings readerSettings5;
        ExperimentalSensor nextSensor5("MT_01200312-002-000_00B474BA", "test");
        readerSettings5.append_ExperimentalSensors(nextSensor5);
        XsensDataReader reader5(readerSettings5);
        DataAdapter::OutputTables tables5 = reader5.read("./");
        auto accelTable5 = tables5.at(XsensDataReader::LinearAccelerations);
        CHECK(accelTable5->getNumRows() == 5);
    }
    SECTION("Exported from MTManager4.6") {

        XsensDataReaderSettings readerSettings;
        const std::vector<OpenSim::ExperimentalSensor> expSens = {
                OpenSim::ExperimentalSensor("_00B42D4D", "femur_l_imu"),
                OpenSim::ExperimentalSensor("_00B42D4E", "calcn_l_imu")};
        for (const auto& sensor : expSens) {
            readerSettings.append_ExperimentalSensors(sensor);
        }
        readerSettings.updProperty_trial_prefix() = "MT46_01-000";

        XsensDataReader reader(readerSettings);
        DataAdapter::OutputTables tables = reader.read("./");
        auto accelTable = tables.at(XsensDataReader::LinearAccelerations);
        REQUIRE(accelTable->getNumRows() == 972);

        auto gyroscopeTable = tables.at(XsensDataReader::AngularVelocity);
        REQUIRE(gyroscopeTable->getNumRows() == 972);

        auto orientationTable = tables.at(XsensDataReader::Orientations);
        REQUIRE(orientationTable->getNumRows() == 972);
    }
}

TEST_CASE("XsensDataReader MT Manager 2022 Data Formats") {
    SECTION("Acceleration Only") {
        XsensDataReaderSettings readerSettings;
        readerSettings.append_ExperimentalSensors(
                OpenSim::ExperimentalSensor("_00B42D4B", "head_imu"));
        readerSettings.updProperty_trial_prefix() = "MT2022_ACC";
        readerSettings.set_sampling_rate(60.0);

        XsensDataReader reader(readerSettings);
        DataAdapter::OutputTables tables = reader.read("./");
        auto accelTable = tables.at(XsensDataReader::LinearAccelerations);
        REQUIRE(accelTable->getNumRows() == 362);
        const TimeSeriesTableVec3& accelTableTyped =
                reader.getLinearAccelerationsTable(tables);
        REQUIRE(accelTableTyped.getTableMetaData()
                        .getValueForKey("DataRate")
                        .getValue<std::string>() == "60.000000");

        const auto& times = accelTableTyped.getIndependentColumn();
        double lastTimestamp = times[times.size() - 1]; // last timestamp
        double expectedLastTimestamp =
                (accelTableTyped.getNumRows() - 1) / 60.0;
        REQUIRE_THAT(lastTimestamp,
                Catch::Matchers::WithinAbs(expectedLastTimestamp, SimTK::Eps));
    }
    SECTION("Acceleration, Gyroscope, All Rotation Formats, Tab separated, "
            "NaN") {
        XsensDataReaderSettings readerSettings;
        readerSettings.append_ExperimentalSensors(
                OpenSim::ExperimentalSensor("_00B42D4B", "head_imu"));
        readerSettings.updProperty_trial_prefix() =
                "MT2022_ACC_GYRO_ROT_ALL_TAB_NAN";

        XsensDataReader reader(readerSettings);
        DataAdapter::OutputTables tables = reader.read("./");
        auto accelTable = tables.at(XsensDataReader::LinearAccelerations);
        REQUIRE(accelTable->getNumRows() == 362);

        auto gyroscopeTable = tables.at(XsensDataReader::AngularVelocity);
        REQUIRE(gyroscopeTable->getNumRows() == 362);

        auto orientationTable = tables.at(XsensDataReader::Orientations);
        REQUIRE(orientationTable->getNumRows() == 362);

        const TimeSeriesTableVec3& accelTableTyped =
                reader.getLinearAccelerationsTable(tables);
        const SimTK::RowVectorView_<SimTK::Vec3>& rvv =
                accelTableTyped.getRowAtIndex(0);
        SimTK::Vec3 fromTable = accelTableTyped.getRowAtIndex(0)[0];
        SimTK::Vec3 fromFile = SimTK::Vec3{SimTK::NaN, -0.040975, 9.922524};
        double tolerance = SimTK::Eps;

        REQUIRE(fromTable.size() == fromFile.size());
        REQUIRE(std::isnan(fromTable[0]));
        REQUIRE_THAT(fromTable[1],
                Catch::Matchers::WithinAbs(fromFile[1], tolerance));
        REQUIRE_THAT(fromTable[2],
                Catch::Matchers::WithinAbs(fromFile[2], tolerance));
    }
    SECTION("Acceleration, Gyroscope, Euler Angles, Tab separated") {
        XsensDataReaderSettings readerSettings;
        readerSettings.append_ExperimentalSensors(
                OpenSim::ExperimentalSensor("_00B42D4F", "radius_r_imu"));
        readerSettings.updProperty_trial_prefix() =
                "MT2022_ACC_GYRO_ROT_EULER_TAB";

        XsensDataReader reader(readerSettings);
        DataAdapter::OutputTables tables = reader.read("./");
        auto accelTable = tables.at(XsensDataReader::LinearAccelerations);
        REQUIRE(accelTable->getNumRows() == 362);

        auto gyroscopeTable = tables.at(XsensDataReader::AngularVelocity);
        REQUIRE(gyroscopeTable->getNumRows() == 362);

        auto orientationTable = tables.at(XsensDataReader::Orientations);
        REQUIRE(orientationTable->getNumRows() == 362);
    }
    SECTION("Acceleration, Gyroscope, Rotation Matrix, Comma separated") {
        XsensDataReaderSettings readerSettings;
        readerSettings.append_ExperimentalSensors(
                OpenSim::ExperimentalSensor("_00B42D4B", "head_imu"));
        readerSettings.updProperty_trial_prefix() =
                "MT2022_ACC_GYRO_ROT_MAT_COMMA";

        XsensDataReader reader(readerSettings);
        DataAdapter::OutputTables tables = reader.read("./");
        auto accelTable = tables.at(XsensDataReader::LinearAccelerations);
        REQUIRE(accelTable->getNumRows() == 362);

        auto gyroscopeTable = tables.at(XsensDataReader::AngularVelocity);
        REQUIRE(gyroscopeTable->getNumRows() == 362);

        auto orientationTable = tables.at(XsensDataReader::Orientations);
        REQUIRE(orientationTable->getNumRows() == 362);
    }
    SECTION("Acceleration, Gyroscope, Rotation Matrix, Semicolon separated") {
        XsensDataReaderSettings readerSettings;
        readerSettings.append_ExperimentalSensors(
                OpenSim::ExperimentalSensor("_00B42D4B", "head_imu"));
        readerSettings.updProperty_trial_prefix() =
                "MT2022_ACC_GYRO_ROT_MAT_SEMICOLON";

        XsensDataReader reader(readerSettings);
        DataAdapter::OutputTables tables = reader.read("./");
        auto accelTable = tables.at(XsensDataReader::LinearAccelerations);
        REQUIRE(accelTable->getNumRows() == 362);

        auto gyroscopeTable = tables.at(XsensDataReader::AngularVelocity);
        REQUIRE(gyroscopeTable->getNumRows() == 362);

        auto orientationTable = tables.at(XsensDataReader::Orientations);
        REQUIRE(orientationTable->getNumRows() == 362);
    }
    SECTION("Acceleration, Gyroscope, Rotation Matrix, Space separated") {
        XsensDataReaderSettings readerSettings;
        readerSettings.append_ExperimentalSensors(
                OpenSim::ExperimentalSensor("_00B42D4B", "head_imu"));
        readerSettings.updProperty_trial_prefix() =
                "MT2022_ACC_GYRO_ROT_MAT_SPACE";

        XsensDataReader reader(readerSettings);
        DataAdapter::OutputTables tables = reader.read("./");
        auto accelTable = tables.at(XsensDataReader::LinearAccelerations);
        REQUIRE(accelTable->getNumRows() == 362);

        auto gyroscopeTable = tables.at(XsensDataReader::AngularVelocity);
        REQUIRE(gyroscopeTable->getNumRows() == 362);

        auto orientationTable = tables.at(XsensDataReader::Orientations);
        REQUIRE(orientationTable->getNumRows() == 362);
    }
}
