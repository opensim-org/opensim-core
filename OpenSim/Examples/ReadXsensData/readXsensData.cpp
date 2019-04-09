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
#include "OpenSim/Common/XsensDataReader.h"
#include "OpenSim/Common/STOFileAdapter.h"

using namespace std;

static void PrintUsage(const char *aProgName, ostream &aOStream);

int main(int argc, char* argv[]) {
    using namespace OpenSim;
    if (argc < 2) {
        PrintUsage(argv[0], cout);
        return(-1);
    }
    std::string folder = "";
    std::string trial = ""; 
    std::string xsensReaderConfigFile = "";
    for (int i = 1; i < argc; i++) {
        string option = argv[i];
        if (option == "-F")
            folder = argv[i + 1];
        else if (option == "-T")
            trial = argv[i + 1];
        else if (option == "-S")
            xsensReaderConfigFile = argv[i + 1];
    }
    try {
        XsensDataReaderSettings readerSettings(xsensReaderConfigFile);
        if (!trial.empty()) readerSettings.upd_trial_prefix() = trial;
        if (!folder.empty()) readerSettings.upd_data_folder() = folder;
        DataAdapter::OutputTables tables = XsensDataReader(readerSettings).extendRead(folder);
        // Write tables to sto files
        // Accelerations
        std::shared_ptr<AbstractDataTable> accelTable = tables.at(XsensDataReader::LinearAccelerations);
        const TimeSeriesTableVec3& accelTableTyped = dynamic_cast<const TimeSeriesTableVec3&>(*accelTable);
        STOFileAdapterVec3::write(accelTableTyped, folder + trial + "accelerations.sto");

        // MagneticHeading
        std::shared_ptr<AbstractDataTable> magneticHeadingTable = tables.at(XsensDataReader::MagneticHeading);
        const TimeSeriesTableVec3& magTableTyped = dynamic_cast<const TimeSeriesTableVec3&>(*magneticHeadingTable);
        STOFileAdapterVec3::write(magTableTyped, folder + trial + "magnetometers.sto");
 
        // AngularVelocity
        std::shared_ptr<AbstractDataTable> angularVelocityTable = tables.at(XsensDataReader::AngularVelocity);
        const TimeSeriesTableVec3& gyroTableTyped = dynamic_cast<const TimeSeriesTableVec3&>(*angularVelocityTable);
        STOFileAdapterVec3::write(gyroTableTyped, folder + trial + "gyros.sto");

        // Orientation
        std::shared_ptr<AbstractDataTable> orientationTable = tables.at(XsensDataReader::Orientations);
        const TimeSeriesTableQuaternion& quatTableTyped = dynamic_cast<const TimeSeriesTableQuaternion&>(*orientationTable);
        STOFileAdapterQuaternion::write(quatTableTyped, folder + trial + "quaternions.sto");
    }
    catch (const std::exception& ex) {
        std::cout << "Xsens Data Reading Failed to run due to the following Exception: "
            << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
//_____________________________________________________________________________
/**
* Print the usage for this application
*/
void PrintUsage(const char *aProgName, std::ostream &aOStream)
{
    aOStream << "\n\n" << aProgName << "\n\n";
    aOStream << "Option              Argument         Action / Notes\n";
    aOStream << "------              --------         --------------\n";
    aOStream << "-S readerSpec.xml  Name of XML file used to configure the reader. \n";
    aOStream << "-F folderName       Folder containing trial data, to override value in readerSpec.xml. \n";
    aOStream << "-T trialName        Common prefix to data file names to override value in readerSpec.xml.\n";
}
