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
        std::vector<std::string> imu_names{ "shank", "thigh" };
        std::vector<std::string> names_in_experiment{ "Static", "Middle" };
        // Programmatically add items to Map, write to xml
        for (int index = 0; index < imu_names.size(); ++index) {
            ExperimentalSensor  nextSensor(names_in_experiment[index], imu_names[index]);
            readerSettings.append_ExperimentalSensors(nextSensor);
        }
        readerSettings.print("reader2xml.xml");
        // read xml we wrote into a new XsensDataReader to readTrial
        APDMDataReaderSettings reconstructFromXML(APDMDataReaderSettings("reader2xml.xml"));
        APDMDataReader reader;
        reader.updSettings() = reconstructFromXML;
        reader.extendRead("imuData01csv.csv");
     }
    catch (const std::exception& ex) {
        std::cout << "testAPDMDataReader FAILED: " << ex.what() << std::endl;
        return 1;
    }

    std::cout << "\n All testAPDMDataReader cases passed." << std::endl;

    return 0;
}
