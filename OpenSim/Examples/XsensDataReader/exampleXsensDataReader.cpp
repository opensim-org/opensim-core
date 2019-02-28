/* -------------------------------------------------------------------------- *
 *                            OpenSim:  exampleXsensDataReader.cpp            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2019 Stanford University and the Authors                *
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
#include "OpenSim/Common/MapObject.h"
#include "OpenSim/Common/IMUHelper.h"
#include "OpenSim/Common/STOFileAdapter.h"


int main() {
    using namespace OpenSim;

    MapObject mapXsensName2ModelName("mapXsensNameToOpenSimIMU.xml");
    const std::string folder = "";
    const std::string trial = "MT_012005D6_031-000_";
    DataAdapter::OutputTables tables = IMUHelper::readXsensTrial(folder, trial, mapXsensName2ModelName);
    // Write tables to sto files
    // Accelerations
    std::shared_ptr<AbstractDataTable> accelTable = tables.at(IMUHelper::LinearAccelerations);
    const TimeSeriesTableVec3& accelTableTyped = dynamic_cast<const TimeSeriesTableVec3&>(*accelTable);
    STOFileAdapterVec3::write(accelTableTyped, folder + trial + "accelerations.sto");

    // Magenometer
    std::shared_ptr<AbstractDataTable> magTable = tables.at(IMUHelper::MagneticHeading);
    const TimeSeriesTableVec3& magTableTyped = dynamic_cast<const TimeSeriesTableVec3&>(*magTable);
    STOFileAdapterVec3::write(magTableTyped, folder + trial + "magnetometers.sto");
 
    // Gyro
    std::shared_ptr<AbstractDataTable> gyroTable = tables.at(IMUHelper::AngularVelocity);
    const TimeSeriesTableVec3& gyroTableTyped = dynamic_cast<const TimeSeriesTableVec3&>(*gyroTable);
    STOFileAdapterVec3::write(gyroTableTyped, folder + trial + "gyros.sto");

    // Orientation
    std::shared_ptr<AbstractDataTable> orientationTable = tables.at(IMUHelper::Orientations);
    const TimeSeriesTableQuaternion& quatTableTyped = dynamic_cast<const TimeSeriesTableQuaternion&>(*orientationTable);
    STOFileAdapterQuaternion::write(quatTableTyped, folder + trial + "quaternions.sto");
    return 0;
}
