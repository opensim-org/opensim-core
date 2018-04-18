#ifndef OPENSIM_SIMULATION_UTILITIES_H_
#define OPENSIM_SIMULATION_UTILITIES_H_
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  SimulationUtilities.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2018 Stanford University and the Authors                *
 * Author(s): OpenSim Team                                                    *
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

#include "Model/Model.h"
#include "Manager/Manager.h"
#include <simbody/internal/Visualizer_InputListener.h>

namespace OpenSim {

/// @name General-purpose simulation driver for OpenSim models
/// @{
/** Simulate a model from an initial state and return the final state.
    If the model's useVisualizer flag is true, the user is repeatedly prompted
    to either begin simulating or quit. The provided state is not updated but
    the final state is returned at the end of the simulation, when finalTime is
    reached. %Set saveStatesFile=true to save the states to a storage file as:
    "<model_name>_states.sto". */
SimTK::State simulate(Model& model,
    const SimTK::State& initialState,
    double finalTime,
    bool saveStatesFile = false);

/// @}

} // end of namespace OpenSim

#endif // OPENSIM_SIMULATION_UTILITIES_H_
