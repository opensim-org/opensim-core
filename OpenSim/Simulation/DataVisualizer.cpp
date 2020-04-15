/* -------------------------------------------------------------------------- *
 *                           OpenSim:  DataVisualizer.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2020 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                      *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "DataVisualizer.h"
#include <OpenSim/Common/TableSource.h>
#include <OpenSim/Simulation/Model/Model.h> 
#include <OpenSim/Simulation/OpenSense/ExperimentalMarker.h>


using namespace std;
using namespace OpenSim;

void DataVisualizer::showMarkerData(
        const TimeSeriesTableVec3& markerTimeSeries) {
    Model previewWorld;

    // Load the marker data into a TableSource that has markers
    // as its output which each markers occupying its own channel
    TableSourceVec3* markersSource = new TableSourceVec3(markerTimeSeries);
    // Add the markersSource Component to the model
    previewWorld.addComponent(markersSource);

    // Get the underlying Table backing the marker Source so we
    // know how many markers we have and their names
    const auto& markerData = markersSource->getTable();
    auto& times = markerData.getIndependentColumn();

    // Create an ExperimentalMarker Component for every column in the markerData
    for (int i = 0; i < int(markerData.getNumColumns()); ++i) {
        auto marker = new ExperimentalMarker();
        marker->setName(markerData.getColumnLabel(i));

        // markers are owned by the model
        previewWorld.addComponent(marker);
        // the time varying location of the marker comes from the markersSource
        // Component
        marker->updInput("location_in_ground")
                .connect(markersSource->getOutput("column").getChannel(
                        markerData.getColumnLabel(i)));
    }

    previewWorld.setUseVisualizer(true);
    SimTK::State& state = previewWorld.initSystem();
    state.updTime() = times[0];

    previewWorld.realizePosition(state);
    previewWorld.getVisualizer().show(state);

    char c;
    std::cout << "Press any key to visualize experimental marker data ..."
              << std::endl;
    std::cin >> c;

    for (size_t j = 0; j < times.size(); j = j + 10) {
        std::cout << "time: " << times[j] << "s" << std::endl;
        state.setTime(times[j]);
        previewWorld.realizePosition(state);
        previewWorld.getVisualizer().show(state);
    }
}