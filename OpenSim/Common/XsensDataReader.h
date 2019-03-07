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

/** XsensDataReader is a class that reads files produced by IMU manufacturer Xsens
    and produces datatables from them. This is intended to help consume IMU outputs.*/
class OSIMCOMMON_API XsensDataReader {
public:
    XsensDataReader() = default;
    XsensDataReader(const XsensDataReader&)            = default;
    XsensDataReader(XsensDataReader&&)                 = default;
    XsensDataReader& operator=(const XsensDataReader&) = default;
    XsensDataReader& operator=(XsensDataReader&&)      = default;
    virtual ~XsensDataReader()                   = default;

    static const std::string Orientations;   // name of table for orientation data
    static const std::string LinearAccelerations;  // name of table for acceleration data
    static const std::string MagneticHeading;  // name of table for data from Magnetometer (Magnetic Heading)
    static const std::string AngularVelocity;  // name of table for gyro data (AngularVelocity)

    /** Typically, Xsens can export a trial as one .mtb file (binary that we can't parse) or as collection of
    collection of ASCII text files that are tab delimited, one per sensor. All of these files have 
    a common prefix that indicates the trial, and a suffix that indicates the sensor (fixed across trials).
    Example: MT_012005D6_031-000_00B421AF.txt, MT_012005D6_031-000_00B4227B.txt for trial MT_012005D6_031-000_
    and sensors 00B421AF, 00B4227B respectively.
    The function below read all files with the given folder name, with common prefix (same trial). It roduces a 
    list of tables depending on the contents of the files read. 
    - One table for rotations, 
    - one for LinearAccelerations
    - one for MagneticHeading data, 
    - one for AngularVelocity data. */
    static DataAdapter::OutputTables readTrial(const std::string& folderName, const std::string& prefix, 
        const MapObject& modelIMUToFilenameMap);
    
private:
    /**
     * Find index of searchString in tokens
     */
    static int find_index(std::vector<std::string>& tokens, const std::string& searchString);
};

} // OpenSim namespace

#endif // OPENSIM_IMU_HELPER_H_
