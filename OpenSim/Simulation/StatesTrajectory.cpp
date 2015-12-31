/* -------------------------------------------------------------------------- *
 *                       OpenSim:  StatesTrajectory.cpp                       *
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
    if (!m_states.empty()) {
        SimTK_APIARGCHECK2_ALWAYS(m_states.back().getTime() <= state.getTime(),
                "StatesTrajectory", "append", 
                "New state's time (%f) must be equal to or greater than the "
                "time for the last state in the trajectory (%f).",
                state.getTime(), m_states.back().getTime()
                );
    }
    m_states.push_back(state);
}

bool StatesTrajectory::consistent() const {
    // An empty or size-1 trajectory is necessarily consistent.
    if (getSize() <= 1) return true;

    const auto& state0 = get(0);

    for (int itime = 1; itime < getSize(); ++itime) {
        const auto& curState = get(itime);

        // TODO this logic should be pushed to the SimTK::State class, so that
        // the check can evolve with the State class.
        // Then the body of this loop would be simply be:
        //      state0.consistentWith(curState);

        if (state0.getNumSubsystems() != curState.getNumSubsystems()) {
            return false;
        }

        // State variables.
        if (state0.getNQ() != curState.getNQ()) {
            return false;
        }
        if (state0.getNU() != curState.getNU()) {
            return false;
        }
        if (state0.getNZ() != curState.getNZ()) {
            return false;
        }

        // Constraints.
        if (state0.getNQErr() != curState.getNQErr()) {
            return false;
        }
        if (state0.getNUErr() != curState.getNUErr()) {
            return false;
        }
        if (state0.getNUDotErr() != curState.getNUDotErr()) {
            return false;
        }
        if (state0.getNMultipliers() != curState.getNMultipliers()) {
            return false;
        }

        // Events.
        if (state0.getNEventTriggers() != curState.getNEventTriggers()) {
            return false;
        }

        // Per-subsystem quantities.
        // TODO we could get rid of the total-over-subsystems checks above, but
        // those checks would let us exit earlier.
        for (SimTK::SubsystemIndex isub(0); isub < state0.getNumSubsystems();
                ++isub) {
            if (state0.getNQ(isub) != curState.getNQ(isub)) {
                return false;
            }
            if (state0.getNU(isub) != curState.getNU(isub)) {
                return false;
            }
            if (state0.getNZ(isub) != curState.getNZ(isub)) {
                return false;
            }
            if (state0.getNQErr(isub) != curState.getNQErr(isub)) {
                return false;
            }
            if (state0.getNUErr(isub) != curState.getNUErr(isub)) {
                return false;
            }
            if (state0.getNUDotErr(isub) != curState.getNUDotErr(isub)) {
                return false;
            }
            if (state0.getNMultipliers(isub) != curState.getNMultipliers(isub)) {
                return false;
            }
            for(SimTK::Stage stage = SimTK::Stage::LowestValid;
                    stage <= SimTK::Stage::HighestRuntime; ++stage) {
                if (state0.getNEventTriggersByStage(isub, stage) !=
                        curState.getNEventTriggersByStage(isub, stage)) {
                    return false;
                }
            }
        }
    }
    return true;
}

bool StatesTrajectory::compatibleWith(const Model& model) {
    // An empty trajectory is necessarily compatible.
    if (getSize() == 0) return true;

    if (!consistent()) return false;

    // Since we now know all the states are consistent with each other, we only
    // need to check if the first one is compatible with the model.
    const auto& state0 = get(0);

    if (model.getNumStateVariables() != state0.getNY()) {
        return false;
    }
    if (model.getNumCoordinates() != state0.getNQ()) {
        return false;
    }
    if (model.getNumSpeeds() != state0.getNU()) {
        return false;
    }
    // TODO number of constraints.

    return true;
}

StatesTrajectory StatesTrajectory::createFromStatesStorage(
        const Model& model,
        const Storage& sto,
        bool allowMissingColumns,
        bool allowExtraColumns) {

    // Assemble the required objects.
    // ==============================

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

    // In case the storage comes from a version of OpenSim prior to 4.0,
    // we must convert the column names from the old state variable names to
    // the new ones that use "paths."
    // TODO auto stoLabels(origStoLabels);
    // TODO convertStatesStorageLabelsToPaths(model, stoLabels);

    // Error checking.
    // ===============
    // Angular quantities must be expressed in radians.
    // TODO we could also manually convert the necessary
    // coordinates/speeds to radians.
    OPENSIM_THROW_IF(sto.isInDegrees(), StatesStorageIsInDegrees);

    // makeStorageLabelsUnique() returns true if labels were unique already.
    // TODO we're making a copy of all the data in order to just check the
    // storage labels; that shouldn't be necessary.
    OPENSIM_THROW_IF(!Storage(sto).makeStorageLabelsUnique(),
            NonUniqueColumnsInStatesStorage);

    // Check if states are missing from the Storage.
    // ---------------------------------------------
    const auto& modelStateNames = localModel.getStateVariableNames();
    std::vector<std::string> missingColumnNames;
    // Also, assemble the names of the states that we will actually set in the
    // trajectory, along with the corresponding state index.
    std::map<int, std::string> statesToFillUp;
    for (int is = 0; is < modelStateNames.getSize(); ++is) {
        // getStateIndex() will check for pre-4.0 column names.
        const int stateIndex = sto.getStateIndex(modelStateNames[is]);
        if (stateIndex == -1) {
            missingColumnNames.push_back(modelStateNames[is]);
        } else {
            statesToFillUp[stateIndex] = modelStateNames[is];
        }
    }
    OPENSIM_THROW_IF(!allowMissingColumns && !missingColumnNames.empty(),
            MissingColumnsInStatesStorage, 
            localModel.getName(), missingColumnNames);

    // Check if the Storage has columns that are not states in the Model.
    // ------------------------------------------------------------------
    if (!allowExtraColumns) {
        if (numDependentColumns > statesToFillUp.size()) {
            std::vector<std::string> extraColumnNames;
            // We want the actual column names, not the state names; the two
            // might be different b/c the state names changed in v4.0.
            for (int ic = 1; ic < stoLabels.getSize(); ++ic) {
                // Has this label been marked as a model state?
                // stateIndex = columnIndex - 1 (b/c of the time column).
                if (statesToFillUp.count(ic - 1) == 0) {
                    extraColumnNames.push_back(stoLabels[ic]);
                }
            }
            OPENSIM_THROW(ExtraColumnsInStatesStorage, localModel.getName(),
                    extraColumnNames);
        }
    }


    // Fill up trajectory.
    // ===================

    // Working memory.
    SimTK::Vector dependentValues(numDependentColumns);

    // Loop through all rows of the Storage.
    for (int itime = 0; itime < sto.getSize(); ++itime) {

        // Extract data from Storage into the working Vector.
        sto.getData(itime, numDependentColumns, dependentValues);

        // Set the correct time in the state.
        sto.getTime(itime, state.updTime());

        // Put in NaNs for state variable values not in the Storage.
        for (const auto& stateName : missingColumnNames) {
            localModel.setStateVariableValue(state, stateName, SimTK::NaN);
        }

        // Fill up current State with the data for the current time.
        for (const auto& kv : statesToFillUp) {
            localModel.setStateVariableValue(state,
                    kv.second, dependentValues[kv.first]);
        }

        // Make a copy of the edited state and put it in the trajectory.
        states.append(state);
    }

    return states;

    // TODO Adjust configuration to match constraints and other goals?
    //      model.assemble(s);
}

StatesTrajectory StatesTrajectory::createFromStatesStorage(
        const Model& model,
        const std::string& filepath) {
    return createFromStatesStorage(model, Storage(filepath));
}




