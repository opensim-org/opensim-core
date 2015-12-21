/* -------------------------------------------------------------------------- *
 *                       OpenSim:  StatesTrajectory.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2015 Stanford University and the Authors                     *
 * Author(s): Chris Dembia                                                    *
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

#include "StatesTrajectory.h"
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

size_t StatesTrajectory::getSize() const {
    return m_states.size();
}

void StatesTrajectory::append(const SimTK::State& state) {
    SimTK_ASSERT(m_states.back().getTime() <= state.getTime(),
            "New state's time (" + std::to_string(state.getTime()) + 
            ") must be equal to or greater than the time for the last "
            "state in the trajectory (" +
            std::to_string(m_states.back().getTime()) + ").");
    m_states.push_back(state);
}

StatesTrajectory StatesTrajectory::createFromStatesStorage(
        const Model& model,
        const Storage& sto,
        bool checkMissingFromStorage,
        bool checkMissingFromModel) {

    // This is what we'll return.
    StatesTrajectory states;

    // Make a copy of the model so that we can get a corresponding state.
    // TODO avoid initSystem() if `model` already has a non-null WorkingState.
    Model localModel(model);

    // We'll keep editing this state as we loop through time.
    auto state = localModel.initSystem();

    // The labels of the columns in the storage file.
    const auto& stoLabels = sto.getColumnLabels();
    int numDependentColumns = stoLabels.getSize() - 1;

    // Working memory.
    SimTK::Vector dependentValues(numDependentColumns);

    // Loop through all rows of the Storage.
    for (int itime = 0; itime < sto.getSize(); ++itime) {

        // Extract data from Storage into the working Vector.
        sto.getData(itime, numDependentColumns, dependentValues);

        // Set the correct time in the state.
        sto.getTime(itime, state.updTime());

        // Fill up current State with the data for the current time.
        for (int icol = 0; icol < numDependentColumns; ++icol) {
            // Storage labels include time at index 0, so add 1 to skip.
            localModel.setStateVariableValue(state,
                     stoLabels[icol + 1], dependentValues[icol]);
        }

        // Make a copy of the edited state and put it in the trajectory.
        states.append(state);
    }

    return states;

    // Adjust configuration to match constraints and other goals
    // TODO model.assemble(s);

    // TODO set NaN for unknown state variables.
}

StatesTrajectory StatesTrajectory::createFromStatesStorage(
        const Model& model,
        const std::string& filepath,
        bool checkMissingFromStorage,
        bool checkMissingFromModel) {
    return createFromStatesStorage(model, Storage(filepath),
            checkMissingFromStorage, checkMissingFromModel);
}
