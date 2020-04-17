#ifndef OPENSIM_VISUALIZER_UTILITIES_H_
#define OPENSIM_VISUALIZER_UTILITIES_H_
/* -------------------------------------------------------------------------- *
 *                          OpenSim: DataVisualizer.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
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


#include "osimSimulationDLL.h"
#include <OpenSim/Common/TimeSeriesTable.h>
#include "StatesTrajectory.h"

namespace OpenSim {
class OSIMSIMULATION_API VisualizerUtilities {
public:
    /// @name Visualize a motion of a model using the simbody-visualizer
    /// @{

    /// Play back an existing motion (from the Storage) in the
    /// simbody-visuailzer. The Storage should contain all generalized
    /// coordinates. The visualizer window allows the user to control playback
    /// speed. This function blocks until the user exits the simbody-visualizer
    /// window.
    // TODO handle degrees.
    static void showMotion(Model, Storage);
    /// @}

    /// @name showModel shows a model using the simbody-visualizer
    /// @{

    ///  Visualize the passed in model in a simbody-visualizer window.
    ///  This function blocks until the user exits the simbody-visualizer
    ///  window.
    static void showModel(Model);

    /** Show markers with time histories specified by the passed in 
        TimeSeriesTableVec3. Visualization is shown in the simbody visualizer.
        Function blocks waiting for user to hit a key to start. */
    static void showMarkerData(const TimeSeriesTableVec3&);
};
}

#endif //OPENSIM_VISUALIZER_UTILITIES_H_
