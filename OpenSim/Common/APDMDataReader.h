#ifndef OPENSIM_APDM_DATA_READER_H_
#define OPENSIM_APDM_DATA_READER_H_

/* -------------------------------------------------------------------------- *
 *                          OpenSim:  APDMDataReader.h                        *
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

// For definition of Exceptions to be used
#include "Object.h"
#include "ExperimentalSensor.h"
#include "APDMDataReaderSettings.h"
#include "TimeSeriesTable.h"
#include "IMUDataReader.h"

/** @file
* This file defines class for reading data files from IMU maker APDM and 
* producing in memory tables to be consumed by the OpenSim tools/pipelines
*/

namespace OpenSim {

/** APDMDataReader is a class that reads files produced by IMU manufacturer APDM
    and produces datatables from them. This is intended to help consume IMU outputs.*/
class OSIMCOMMON_API APDMDataReader : public IMUDataReader {
public:
    // Default Constructor
    APDMDataReader() = default;
    // Constructor that takes a APDMDataReaderSettings object
    APDMDataReader(const APDMDataReaderSettings& settings) {
        _settings = settings;
    }
    virtual ~APDMDataReader() = default;

    APDMDataReader* clone() const override;

    // Ordered labels provided by APDM
    static const std::vector<std::string> acceleration_labels;
    static const std::vector<std::string> angular_velocity_labels;
    static const std::vector<std::string> magnetic_heading_labels;
    static const std::vector<std::string> orientation_labels;
    // Header associated with Time
    static const std::string TimeLabel;

protected:
    /** Typically, APDM can export a trial as one .h5 file (binary that we don't parse as of now) or as .csv
    ASCII text file that is comma delimited, grouped in order by sensor.
    The function below reads the csv file . It produces a 
    list of tables depending on the contents of the file read. 
    - One table for rotations, 
    - one for LinearAccelerations
    - one for MagneticHeading data, 
    - one for AngularVelocity data. 
    - Barometer and Temperature data is ignored for now
     
     @see IMUDataReader class for utilities to extract/access specific table(s)
    */
    DataAdapter::OutputTables extendRead(const std::string& fileName) const override;

    /** Implements writing functionality, not implemented.                         */
    virtual void extendWrite(const DataAdapter::InputTables& tables,
        const std::string& sinkName) const override {};
    /**
     * Method to get const reference to the internal APDMDataReaderSettings object
     * maintained by this reader.
     */
    const APDMDataReaderSettings& getSettings() const {
        return _settings; 
    }
   /**
    * Method to get writable reference to the internal APDMDataReaderSettings object
    * maintained by this reader, to allow modification after construction.
    */
    APDMDataReaderSettings& updSettings() {
        return _settings;
    }
 private:
    /**
     * This data member encapsulates all the serializable settings for the Reader;
     */
    APDMDataReaderSettings _settings;
    // Utility function to locate data based on labels
    void find_start_column(std::vector<std::string> tokens, 
        std::vector<std::string> search_labels,
        const std::string& sensorName,
        std::vector<int>& indices, bool newFormat=false) const;
};

} // OpenSim namespace

#endif // OPENSIM_APDM_DATA_READER_H_
