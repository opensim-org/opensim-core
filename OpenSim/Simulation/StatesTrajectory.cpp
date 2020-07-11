/* -------------------------------------------------------------------------- *
 *                       OpenSim:  StatesTrajectory.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

#include <OpenSim/Common/CommonUtilities.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/TableUtilities.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

size_t StatesTrajectory::getSize() const {
    return m_states.size();
}

void StatesTrajectory::clear() {
    m_states.clear();
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
        OPENSIM_THROW_IF(!m_states.back().isConsistent(state),
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

    for (unsigned itime = 1; itime < getSize(); ++itime) {

        if (get(itime).getTime() < get(itime - 1).getTime()) {
            return false;
        }

    }
    return true;
}

bool StatesTrajectory::isConsistent() const {
    // An empty or size-1 trajectory is necessarily consistent.
    if (getSize() <= 1) return true;

    const auto& state0 = operator[](0);

    for (unsigned itime = 1; itime < getSize(); ++itime) {

        if (!state0.isConsistent(operator[](itime))) {
            return false;
        }

    }
    return true;
}

bool StatesTrajectory::isCompatibleWith(const Model& model) const {
    // An empty trajectory is necessarily compatible.
    if (getSize() == 0) return true;

    if (!isConsistent()) return false;

    // Since we now know all the states are consistent with each other, we only
    // need to check if the first one is compatible with the model.
    const auto& state0 = get(0);

    // We only check the number of speeds because OpenSim does not count
    // quaternion slots, while the SimTK State contains quaternion slots even if
    // quaternions are not used.
    if (model.getNumSpeeds() != state0.getNU()) {
        return false;
    }
    // TODO number of constraints.

    return true;
}

// Hide this function from other translation units.
namespace {
    template <typename T>
    std::vector<std::string> createVector(const T& strings) {
        std::vector<std::string> vec;
        for (int i = 0; i < strings.size(); ++i)
            vec.push_back(strings[i]);
        return vec;
    }
}

TimeSeriesTable StatesTrajectory::exportToTable(const Model& model,
        const std::vector<std::string>& requestedStateVars) const {

    OPENSIM_THROW_IF(!isCompatibleWith(model),
                     StatesTrajectory::IncompatibleModel, model);

    // This code is based on DelimFileAdapter::extendRead().
    TimeSeriesTable table;

    // Set the column labels as metadata.
    std::vector<std::string> stateVars = requestedStateVars.empty() ?
            ::createVector(model.getStateVariableNames()) :
            requestedStateVars;
    table.setColumnLabels(stateVars);
    size_t numDepColumns = stateVars.size();

    // Fill up the table with the data.
    for (size_t itime = 0; itime < getSize(); ++itime) {
        const auto& state = get(itime);
        TimeSeriesTable::RowVector row(static_cast<int>(numDepColumns));

        // Get each state variable's value.
        if (requestedStateVars.empty()) {
            // This is *much* faster than getting the values one-by-one.
            row = model.getStateVariableValues(state).transpose();
        } else {
            for (unsigned icol = 0; icol < numDepColumns; ++icol) {
                row[static_cast<int>(icol)] =
                    model.getStateVariableValue(state, stateVars[icol]);
            }
        }

        table.appendRow(state.getTime(), row);
    }

    return table;
}

StatesTrajectory StatesTrajectory::createFromStatesStorage(
        const Model& model,
        const Storage& sto,
        bool allowMissingColumns,
        bool allowExtraColumns,
        bool assemble) {
    return createFromStatesTable(model, sto.exportToTable(),
            allowMissingColumns, allowExtraColumns, assemble);
}

StatesTrajectory StatesTrajectory::createFromStatesTable(
        const Model& model,
        const TimeSeriesTable& table,
        bool allowMissingColumns,
        bool allowExtraColumns,
        bool assemble) {

    // Assemble the required objects.
    // ==============================

    // This is what we'll return.
    StatesTrajectory states;

    // Make a copy of the model so that we can get a corresponding state.
    Model localModel(model);

    // We'll keep editing this state as we loop through time.
    auto state = localModel.initSystem();

    // The labels of the columns in the storage file.
    const auto& tableLabels = table.getColumnLabels();
    int numDependentColumns = (int)table.getNumColumns();

    // Error checking.
    // ===============
    // Angular quantities must be expressed in radians.
    // TODO we could also manually convert the necessary coords/speeds to
    // radians.
    OPENSIM_THROW_IF(TableUtilities::isInDegrees(table), DataIsInDegrees);

    // If column labels aren't unique, it's unclear which column the user
    // wanted to use for the related state variable.
    TableUtilities::checkNonUniqueLabels(tableLabels);

    // Check if states are missing from the Storage.
    // ---------------------------------------------
    const auto& modelStateNames = localModel.getStateVariableNames();
    std::vector<std::string> missingColumnNames;
    // Also, assemble the indices of the states that we will actually set in the
    // trajectory.
    std::map<int, int> statesToFillUp;
    for (int is = 0; is < modelStateNames.getSize(); ++is) {
        // getStateIndex() will check for pre-4.0 column names.
        const int stateIndex = TableUtilities::findStateLabelIndex(
                tableLabels, modelStateNames[is]);
        if (stateIndex == -1) {
            missingColumnNames.push_back(modelStateNames[is]);
        } else {
            statesToFillUp[stateIndex] = is;
        }
    }
    OPENSIM_THROW_IF(!allowMissingColumns && !missingColumnNames.empty(),
            MissingColumns,
            localModel.getName(), missingColumnNames);

    // Check if the Storage has columns that are not states in the Model.
    // ------------------------------------------------------------------
    if (!allowExtraColumns) {
        if ((unsigned)numDependentColumns > statesToFillUp.size()) {
            std::vector<std::string> extraColumnNames;
            // We want the actual column names, not the state names; the two
            // might be different b/c the state names changed in v4.0.
            for (int ic = 1; ic < (int)tableLabels.size(); ++ic) {
                // Has this label been marked as a model state?
                if (statesToFillUp.count(ic) == 0) {
                    extraColumnNames.push_back(tableLabels[ic]);
                }
            }
            OPENSIM_THROW(ExtraColumns, localModel.getName(),
                    extraColumnNames);
        }
    }

    // Fill up trajectory.
    // ===================

    // Reserve the memory we'll need to fit all the states.
    states.m_states.reserve(table.getNumRows());

    // Working memory for state. Initialize so that missing columns end up as
    // NaN.
    SimTK::Vector statesValues(modelStateNames.getSize(), SimTK::NaN);

    // Initialize so that missing columns end up as NaN.
    state.updY().setToNaN();

    // Loop through all rows of the Storage.
    for (int itime = 0; itime < (int)table.getNumRows(); ++itime) {
        const auto& row = table.getRowAtIndex(itime);

        // Set the correct time in the state.
        state.setTime(table.getIndependentColumn()[itime]);

        // Fill up current State with the data for the current time.
        for (const auto& kv : statesToFillUp) {
            // 'first': index for Storage; 'second': index for Model.
            statesValues[kv.second] = row[kv.first];
        }
        localModel.setStateVariableValues(state, statesValues);
        if (assemble) {
            localModel.assemble(state);
        }

        // Make a copy of the edited state and put it in the trajectory.
        states.append(state);
    }

    return states;
}

StatesTrajectory StatesTrajectory::createFromStatesStorage(
        const Model& model,
        const std::string& filepath) {
    return createFromStatesTable(model, TimeSeriesTable(filepath));
}

StatesTrajectory::IncompatibleModel::IncompatibleModel(
        const std::string& file, size_t line,
        const std::string& func, const Model& model)
        : OpenSim::Exception(file, line, func) {
    std::ostringstream msg;
    auto modelName = model.getName().empty() ? "<empty-name>" :
                     model.getName();
    msg << "The provided model '" << modelName << "' is not "
            "compatible with the StatesTrajectory.";
    addMessage(msg.str());
}
