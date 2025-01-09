#ifndef OPENSIM_IMU_DATA_READER_H_
#define OPENSIM_IMU_DATA_READER_H_

/* -------------------------------------------------------------------------- *
 *                          OpenSim:  IMUDataReader.h                      *
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
#include "osimCommonDLL.h"
#include "TimeSeriesTable.h"
#include "DataAdapter.h"

/** @file
* This file defines common base class for various IMU DataReader
* classes that support different IMU providers
*/

namespace OpenSim {


class OSIMCOMMON_API IMUDataReader : public DataAdapter {

public:
    
    IMUDataReader() = default;
    IMUDataReader(const IMUDataReader&)            = default;
    IMUDataReader(IMUDataReader&&)                 = default;
    IMUDataReader& operator=(const IMUDataReader&) = default;
    IMUDataReader& operator=(IMUDataReader&&)      = default;
    virtual ~IMUDataReader()                   = default;

    static const std::string Orientations;         // name of table for orientation data
    static const  std::string LinearAccelerations;  // name of table for acceleration data
    static const std::string MagneticHeading;      // name of table for data from Magnetometer (Magnetic Heading)
    static const std::string AngularVelocity;      // name of table for gyro data (AngularVelocity)
   /**
     * Custom accessors to retrieve tables of proper types without requiring users/scripters to cast.
     * Scripting friendly */
     /** get table of Orientations as TimeSeriesTableQuaternion */
    static const TimeSeriesTable_<SimTK::Quaternion_<double>>& getOrientationsTable(const DataAdapter::OutputTables& tables) {
        return dynamic_cast<const TimeSeriesTableQuaternion&>(*tables.at(Orientations));
    }
    /** get table of LinearAccelerations as TimeSeriesTableVec3 */
    static const TimeSeriesTableVec3& getLinearAccelerationsTable(const DataAdapter::OutputTables& tables) {
        return dynamic_cast<const TimeSeriesTableVec3&>(*tables.at(LinearAccelerations));
    }
    /** get table of MagneticHeading as TimeSeriesTableVec3 */
    static const TimeSeriesTableVec3& getMagneticHeadingTable(const DataAdapter::OutputTables& tables) {
        return dynamic_cast<const TimeSeriesTableVec3&>(*tables.at(MagneticHeading));
    }
    /** get table of AngularVelocity as TimeSeriesTableVec3 */
    static const TimeSeriesTableVec3& getAngularVelocityTable(const DataAdapter::OutputTables& tables) {
        return dynamic_cast<const TimeSeriesTableVec3&>(*tables.at(AngularVelocity));
    }
protected:
    /** create a map of names to TimeSeriesTables. MetaData contains dataRate.
     * The result can be passed to accessors above to get individual TimeSeriesTable(s)
     * If a matrix has nrows = 0 then an empty table is created.
     */
    DataAdapter::OutputTables createTablesFromMatrices(double dataRate, 
        const std::vector<std::string>& labels, const std::vector<double>& times,
        const SimTK::Matrix_<SimTK::Quaternion>& rotationsData, 
        const SimTK::Matrix_<SimTK::Vec3>& linearAccelerationData, 
        const SimTK::Matrix_<SimTK::Vec3>& magneticHeadingData, 
        const SimTK::Matrix_<SimTK::Vec3>& angularVelocityData) const;
};

} // OpenSim namespace

#endif // OPENSIM_IMU_DATA_READER_H_
