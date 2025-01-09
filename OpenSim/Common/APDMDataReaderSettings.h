#ifndef OPENSIM_APDM_DATA_READER_SETTINGS_H_
#define OPENSIM_APDM_DATA_READER_SETTINGS_H_

/* -------------------------------------------------------------------------- *
 *                          OpenSim:  APDMDataReaderSettings.h               *
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
/** @file
* This file defines class for configuring the reading data files from IMU maker APDM.
*/

namespace OpenSim {

/** APDMDataReaderSettings is a class that reads files produced by IMU manufacturer APDM
    and produces datatables from them. This is intended to help consume IMU outputs.*/
class OSIMCOMMON_API APDMDataReaderSettings : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(APDMDataReaderSettings, Object);
public:
    OpenSim_DECLARE_LIST_PROPERTY(ExperimentalSensors, ExperimentalSensor,
        "List of Experimental sensors and desired associated column labels in resulting tables");

public:
    // Default Constructor
    APDMDataReaderSettings() {
        constructProperties();
    };
    // Constructor from XML file
    APDMDataReaderSettings(const std::string& xmlFile) : Object(xmlFile, false) {
        constructProperties();
        updateFromXMLDocument();
    };

    virtual ~APDMDataReaderSettings() = default;
    
private:
    void constructProperties() {
        constructProperty_ExperimentalSensors();
    }
};

} // OpenSim namespace

#endif // OPENSIM_APDM_DATA_READER_SETTINGS_H_
