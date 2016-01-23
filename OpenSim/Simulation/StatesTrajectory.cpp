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

size_t StatesTrajectory::getIndexBefore(const double& time,
        const double& tolerance) {
    OPENSIM_THROW_IF(m_states.empty(), Exception, "Trajectory is empty.");

    const auto it = getIteratorBefore(time, tolerance);

    OPENSIM_THROW_IF(it == m_states.end(),
            TimeOutOfRange, time, "before",
            m_states.front().getTime(), m_states.back().getTime());

    return it - m_states.begin();
}

size_t StatesTrajectory::getIndexAfter(const double& time,
        const double& tolerance) {
    OPENSIM_THROW_IF(m_states.empty(), Exception, "Trajectory is empty.");

    const auto it = getIteratorAfter(time, tolerance);

    OPENSIM_THROW_IF(it == m_states.end(),
            TimeOutOfRange, time, "after",
            m_states.front().getTime(), m_states.back().getTime());

    return it - m_states.begin();
}

StatesTrajectory::IteratorRange StatesTrajectory::getBetween(
        const double& startTime, const double& endTime, 
        const double& tolerance) {
    SimTK_APIARGCHECK2_ALWAYS(startTime <= endTime,
            "StatesTrajectory", "getBetween",
            "startTime (%f) must be less than or equal to endTime (%f)",
            startTime, endTime);

    // If startTime < endTime < front().getTime(), then `start` points to
    // front(), which will cause iterating over the *entire* trajectory. To
    // prevent this, we have to detect if front().getTime() > endTime.
    if (!m_states.empty() && front().getTime() > endTime) {
        return makeIteratorRange(end(), end());
    }

    // Must add one to the last iterator since it's supposed to point past
    // the end.
    return makeIteratorRange(getIteratorAfter(startTime, tolerance),
                             getIteratorBefore(endTime, tolerance) + 1);
}

StatesTrajectory::const_iterator
StatesTrajectory::getIteratorBefore(const double& time,
        const double& tolerance) {
    // Need a custom comparision function to extract the time from the state.
    auto compare = [](const double& t, const SimTK::State& s) {
                return t < s.getTime();
            };
    // upper_bound() finds the first element whose time is greater than the
    // given time.
    // TODO tolerance.
    auto upper = std::upper_bound(m_states.begin(), m_states.end(),
                                  time + tolerance, compare);

    // If the first element is greater than the given time, then there are no
    // elements before the given time.
    if (upper == m_states.begin()) {
        return m_states.end();
    }

    // We step back one element to get the last element whose time is less than
    // or equal to the given time.
    return upper - 1;
}

StatesTrajectory::const_iterator
StatesTrajectory::getIteratorAfter(const double& time,
        const double& tolerance) {
    // Need a custom comparision function to extract the time from the state.
    auto compare = [](const SimTK::State& s, const double& t) {
                return s.getTime() < t;
            };
    // lower_bound() finds the first element whose time is greater than or
    // equal to the given time.
    // If the iterator is end(), there are no states after the given time.
    auto lower = std::lower_bound(m_states.begin(), m_states.end(),
                                  time - tolerance, compare);

    return lower;
}

void StatesTrajectory::append(const SimTK::State& state) {
    if (!m_states.empty()) {

        SimTK_APIARGCHECK2_ALWAYS(m_states.back().getTime() <= state.getTime(),
                "StatesTrajectory", "append", 
                "New state's time (%f) must be equal to or greater than the "
                "time for the last state in the trajectory (%f).",
                state.getTime(), m_states.back().getTime()
                );

        // We assume the trajectory (before appending) is already consistent, 
        // so we only need to check consistency with a single state in the
        // trajectory.
        OPENSIM_THROW_IF(!isConsistent(m_states.back(), state),
          InconsistentState, state.getTime());
    }
    m_states.push_back(state);
}

bool StatesTrajectory::hasIntegrity() const {
    return isNondecreasingInTime() && isConsistent();
}

bool StatesTrajectory::isNondecreasingInTime() const {
    // An empty or size-1 trajectory necessarily has nondecreasing times.
    if (getSize() <= 1) return true;

    for (int itime = 1; itime < getSize(); ++itime) {

        if (get(itime).getTime() < get(itime - 1).getTime()) {
            return false;
        }

    }
    return true;
}

bool StatesTrajectory::isConsistent() const {
    // An empty or size-1 trajectory is necessarily consistent.
    if (getSize() <= 1) return true;

    const auto& state0 = get(0);

    for (int itime = 1; itime < getSize(); ++itime) {

        if (!isConsistent(state0, get(itime))) {
            return false;
        }

    }
    return true;
}

bool StatesTrajectory::isCompatibleWith(const Model& model) {
    // An empty trajectory is necessarily compatible.
    if (getSize() == 0) return true;

    if (!isConsistent()) return false;

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

bool StatesTrajectory::isConsistent(const SimTK::State& stateA,
                                    const SimTK::State& stateB) {
    if (stateA.getNumSubsystems() != stateB.getNumSubsystems()) {
        return false;
    }

    // State variables.
    if (stateA.getNQ() != stateB.getNQ()) {
        return false;
    }
    if (stateA.getNU() != stateB.getNU()) {
        return false;
    }
    if (stateA.getNZ() != stateB.getNZ()) {
        return false;
    }

    // Constraints.
    if (stateA.getNQErr() != stateB.getNQErr()) {
        return false;
    }
    if (stateA.getNUErr() != stateB.getNUErr()) {
        return false;
    }
    if (stateA.getNUDotErr() != stateB.getNUDotErr()) {
        return false;
    }
    if (stateA.getNMultipliers() != stateB.getNMultipliers()) {
        return false;
    }

    // Events.
    if (stateA.getNEventTriggers() != stateB.getNEventTriggers()) {
        return false;
    }

    // Per-subsystem quantities.
    // TODO we could get rid of the total-over-subsystems checks above, but
    // those checks would let us exit earlier.
    for (SimTK::SubsystemIndex isub(0); isub < stateA.getNumSubsystems();
            ++isub) {
        if (stateA.getNQ(isub) != stateB.getNQ(isub)) {
            return false;
        }
        if (stateA.getNU(isub) != stateB.getNU(isub)) {
            return false;
        }
        if (stateA.getNZ(isub) != stateB.getNZ(isub)) {
            return false;
        }
        if (stateA.getNQErr(isub) != stateB.getNQErr(isub)) {
            return false;
        }
        if (stateA.getNUErr(isub) != stateB.getNUErr(isub)) {
            return false;
        }
        if (stateA.getNUDotErr(isub) != stateB.getNUDotErr(isub)) {
            return false;
        }
        if (stateA.getNMultipliers(isub) != stateB.getNMultipliers(isub)) {
            return false;
        }
        for(SimTK::Stage stage = SimTK::Stage::LowestValid;
                stage <= SimTK::Stage::HighestRuntime; ++stage) {
            if (stateA.getNEventTriggersByStage(isub, stage) !=
                    stateB.getNEventTriggersByStage(isub, stage)) {
                return false;
            }
        }
    }
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
    Model localModel(model);

    // We'll keep editing this state as we loop through time.
    auto state = localModel.initSystem();

    // The labels of the columns in the storage file.
    const auto& stoLabels = sto.getColumnLabels();
    int numDependentColumns = stoLabels.getSize() - 1;

    // Error checking.
    // ===============
    // Angular quantities must be expressed in radians.
    // TODO we could also manually convert the necessary coords/speeds to
    // radians.
    OPENSIM_THROW_IF(sto.isInDegrees(), StatesStorageIsInDegrees);

    // If column labels aren't unique, It's unclear which column the user
    // wanted to use for the related state variable.
    OPENSIM_THROW_IF(!sto.storageLabelsAreUnique(),
            NonUniqueColumnsInStatesStorage);

    // To be safe, we don't process storages in which individual rows are
    // missing values.
    OPENSIM_THROW_IF(numDependentColumns != sto.getSmallestNumberOfStates(),
            VaryingNumberOfStatesPerRow,
            numDependentColumns, sto.getSmallestNumberOfStates());

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

    // Reserve the memory we'll need to fit all the states.
    states.m_states.reserve(sto.getSize());

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
}

StatesTrajectory StatesTrajectory::createFromStatesStorage(
        const Model& model,
        const std::string& filepath) {
    return createFromStatesStorage(model, Storage(filepath));
}




