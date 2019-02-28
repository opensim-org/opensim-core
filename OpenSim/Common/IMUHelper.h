/* -------------------------------------------------------------------------- *
 *                          OpenSim:  IMUHelper.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
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

#ifndef OPENSIM_IMU_HELPER_H_
#define OPENSIM_IMU_HELPER_H_

// For definition of Exceptions to be used
#include "DataAdapter.h"
#include "FileAdapter.h"
#include "TimeSeriesTable.h"
#include "Event.h"
#include "MapObject.h"
/** @file
* This file defines Helper class for reading or writing data files from IMU Makers.
*/
#include <map>

namespace OpenSim {

/** IMUHelper is a utility class that reads and writes files produced by IMU manufacturers
    and produces datatables from them. This is intended to help consume IMU outputs,
    files but no plan to create manufacturer specific formatted files.*/
class OSIMCOMMON_API IMUHelper {
public:
    IMUHelper()                              = default;
    IMUHelper(const IMUHelper&)            = default;
    IMUHelper(IMUHelper&&)                 = default;
    IMUHelper& operator=(const IMUHelper&) = default;
    IMUHelper& operator=(IMUHelper&&)      = default;
    virtual ~IMUHelper()                     = default;

    static const std::string _orientations;   // name of table for orientation data
    static const std::string _linearAccelerations;  // name of table for acceleration data
    static const std::string _magnetometers;  // name of table for magnetometer data
    static const std::string _gyros;          // name of table for gyro data

    /** Read all files with the given folder name, with common prefix. Produce a list of tables 
    depending on the contents of the files read. One table for rotations, one for Gyro
    one for Magnetometer data, one for Accelerometer data. */
    static DataAdapter::OutputTables readXsensTrial(const std::string& folderName, const std::string& prefix, 
        const MapObject& filenameToModelIMUMap);
    
private:
    /**
     * Find index of searchString in tokens
     */
    static int find_index(std::vector<std::string>& tokens, const std::string& searchString);
};

} // OpenSim namespace

#endif // OPENSIM_IMU_HELPER_H_
