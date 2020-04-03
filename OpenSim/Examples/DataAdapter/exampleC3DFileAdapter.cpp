/* -------------------------------------------------------------------------- *
 *                            OpenSim:  exampleC3DFileAdapter.cpp             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2020 Stanford University and the Authors                *
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

#include "OpenSim/Common/C3DFileAdapter.h"

#include "OpenSim/Common/TRCFileAdapter.h"

#include <vector>
#include <unordered_map>
#include <cstdlib>
#include <chrono>

int main() {
    using namespace OpenSim;

    std::string filename{"walking2.c3d"};

    C3DFileAdapter c3d_adapter{};
    auto tables = c3d_adapter.read(filename);

    auto marker_table = c3d_adapter.getMarkersTable(tables);
    auto force_table = c3d_adapter.getForcesTable(tables);

    if(marker_table->getNumRows() != 0) {
        std::cout << "--------------Markers-----------------" << std::endl;

    std::cout << "Dim: " 
              << marker_table->getNumRows() << " "
              << marker_table->getNumColumns() 
              << std::endl;
    std::cout << "DataRate: " 
              << marker_table->getTableMetaData().
                               getValueForKey("DataRate").
                               getValue<std::string>()
              << std::endl;
    std::cout << "Units: " 
              << marker_table->getTableMetaData().
                               getValueForKey("Units").
                               getValue<std::string>()
              << std::endl << std::endl;
    std::cout << marker_table->getRow(0) << std::endl;

    auto& events_table = marker_table->getTableMetaData().
                                      getValueForKey("events").
                                 getValue<std::vector<OpenSim::Event>>();
    for(const auto& elem : events_table)
        std::cout << "label: " << elem.label << " | "
                  << "time: " << elem.time << " | "
                  << "frame: " << elem.frame << " | "
                  << "description: " << elem.description << "\n";

    auto& labels = marker_table->getDependentsMetaData().
                                 getValueArrayForKey("labels");
    for(size_t i = 0; i < labels.size(); ++i)
        std::cout << labels[i].getValue<std::string>() << " ";
    std::cout << "\n";
    }

    if(force_table->getNumRows() != 0) {
        std::cout << "--------------Forces-----------------" << std::endl;

    std::cout << "Dim: "
              << force_table->getNumRows() << " "
              << force_table->getNumColumns()
              << std::endl;
    std::cout << "DataRate: "
              << force_table->getTableMetaData().
                              getValueForKey("DataRate").
                              getValue<std::string>()
              << std::endl;
    std::cout << "CalibrationMatrices: \n";
    for(const auto& elem : force_table->getTableMetaData().
            getValueForKey("CalibrationMatrices").
            getValue<std::vector<SimTK::Matrix_<double>>>())
        std::cout << elem << std::endl;
    std::cout << "Corners: \n";
    for(const auto& elem : force_table->getTableMetaData().
            getValueForKey("Corners").
            getValue<std::vector<SimTK::Matrix_<double>>>())
        std::cout << elem << std::endl;
    std::cout << "Origins: \n";
    for(const auto& elem : force_table->getTableMetaData().
            getValueForKey("Origins").
            getValue<std::vector<SimTK::Matrix_<double>>>())
        std::cout << elem << std::endl;
    std::cout << "Types: \n";
    for(const auto& elem : force_table->getTableMetaData().
            getValueForKey("Types").
            getValue<std::vector<unsigned>>())
        std::cout << elem << std::endl;

    const auto& labels = force_table->getDependentsMetaData().
        getValueArrayForKey("labels");
    const auto& units = force_table->getDependentsMetaData().
        getValueArrayForKey("units");
    for(size_t i = 0; i < labels.size(); ++i)
        std::cout << "[ " << labels[i].getValue<std::string>() << " " 
                  << units[i].getValue<std::string>() << " ] ";
    std::cout << std::endl;
    std::cout << force_table->getRow(0) << std::endl;
    }


    return 0;
}
