#ifndef OPENSIM_XSENS_DATA_READER_H_
#define OPENSIM_XSENS_DATA_READER_H_

/* -------------------------------------------------------------------------- *
 *                          OpenSim:  XsensDataReader.h                       *
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
#include "DataAdapter.h"
#include "XsensDataReaderSettings.h"
#include "TimeSeriesTable.h"
#include "IMUDataReader.h"
/** @file
* This file defines class for reading data files from IMU maker Xsens and 
* producing in memory tables to be consumed by the OpenSim tools/pipelines
*/

namespace OpenSim {

/** XsensDataReader is a class that reads files produced by IMU manufacturer
   Xsens and produces datatables from them. This is intended to help consume IMU
   outputs.*/
class OSIMCOMMON_API XsensDataReader : public IMUDataReader {
public:
    // Default Constructor
    XsensDataReader() = default;
    // Constructor that takes a XsensDataReaderSettings object
    XsensDataReader(const XsensDataReaderSettings& settings) {
        _settings = settings;
    }
    virtual ~XsensDataReader() = default;

    XsensDataReader* clone() const override;
protected:
    /** Typically, Xsens can export a trial as one .mtb file (binary that we 
    can't parse) or as collection of ASCII text files that are tab delimited, 
    one per sensor. All of these files have a common prefix that indicates the 
    trial, and a suffix that indicates the sensor (fixed across trials).
    Example: MT_012005D6_031-000_00B421AF.txt, MT_012005D6_031-000_00B4227B.txt 
    for trial MT_012005D6_031-000_and sensors 00B421AF, 00B4227B respectively.
    The function below reads all files in the given folder name, with common 
    prefix (same trial). It produces a list of tables depending on the contents 
    of the files read. 
    - One table for Orientations, 
    - one table for LinearAccelerations
    - one table for MagneticHeading data, 
    - one table for AngularVelocity data. 
    If data is missing, an empty table is returned. 
    */
    DataAdapter::OutputTables extendRead(const std::string& folderName) const override;

    /** Implements writing functionality, not implemented. */
    virtual void extendWrite(const DataAdapter::InputTables& tables,
        const std::string& sinkName) const override {};
public:
    /**
     * Method to get const reference to the internal XsensDataReaderSettings object
     * maintained by this reader.
     */
    const XsensDataReaderSettings& getSettings() const {
        return _settings; 
    }
   /**
    * Method to get writable reference to the internal XsensDataReaderSettings object
    * maintained by this reader, to allow modification after construction.
    */
    XsensDataReaderSettings& updSettings() {
        return _settings;
    }
 private:
    /**
     * Find index of searchString in tokens
     */
    static int find_index(std::vector<std::string>& tokens, const std::string& searchString);
    /**
     * This data member encapsulates all the serializable settings for the Reader;
     */
    XsensDataReaderSettings _settings;
};

} // OpenSim namespace

#endif // OPENSIM_XSENS_DATA_READER_H_
