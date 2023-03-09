/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoTrajectory.cpp                                           *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2023 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia, Nicholas Bianco                             *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */
#include "MocoTrajectory.h"

#include "MocoProblem.h"
#include "MocoUtilities.h"

#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

const std::vector<std::string> MocoTrajectory::m_allowedKeys =
        {"states", "controls", "multipliers", "derivatives"};

MocoTrajectory::MocoTrajectory(
        std::vector<std::string> state_names,
        std::vector<std::string> control_names,
        std::vector<std::string> multiplier_names,
        std::vector<std::string> parameter_names)
        : m_state_names(std::move(state_names)),
          m_control_names(std::move(control_names)),
          m_multiplier_names(std::move(multiplier_names)),
          m_parameter_names(std::move(parameter_names)) {}

MocoTrajectory::MocoTrajectory(
        std::vector<std::string> state_names,
        std::vector<std::string> control_names,
        std::vector<std::string> multiplier_names,
        std::vector<std::string> derivative_names,
        std::vector<std::string> parameter_names)
        : m_state_names(std::move(state_names)),
          m_control_names(std::move(control_names)),
          m_multiplier_names(std::move(multiplier_names)),
          m_derivative_names(std::move(derivative_names)),
          m_parameter_names(std::move(parameter_names)) {}

MocoTrajectory::MocoTrajectory(const SimTK::Vector& time,
        std::vector<std::string> state_names,
        std::vector<std::string> control_names,
        std::vector<std::string> multiplier_names,
        std::vector<std::string> parameter_names,
        const SimTK::Matrix& statesTrajectory,
        const SimTK::Matrix& controlsTrajectory,
        const SimTK::Matrix& multipliersTrajectory,
        const SimTK::RowVector& parameters)
        : m_time(time), m_state_names(std::move(state_names)),
          m_control_names(std::move(control_names)),
          m_multiplier_names(std::move(multiplier_names)),
          m_parameter_names(std::move(parameter_names)),
          m_states(statesTrajectory), m_controls(controlsTrajectory),
          m_multipliers(multipliersTrajectory), m_parameters(parameters) {
    OPENSIM_THROW_IF((int)m_state_names.size() != m_states.ncol(), Exception,
            "Inconsistent number of states.");
    OPENSIM_THROW_IF((int)m_control_names.size() != m_controls.ncol(),
            Exception, "Inconsistent number of controls.");
    OPENSIM_THROW_IF((int)m_multiplier_names.size() != m_multipliers.ncol(),
            Exception, "Inconsistent number of multipliers.");
    if (m_states.ncol()) {
        OPENSIM_THROW_IF(time.size() != m_states.nrow(), Exception,
                "Expected states to have {} rows but it has {}.", time.size(),
                m_states.nrow());
    } else {
        m_states.resize(m_time.size(), 0);
    }
    if (m_controls.ncol()) {
        OPENSIM_THROW_IF(time.size() != m_controls.nrow(), Exception,
                "Expected controls to have {} rows but it has {}.", time.size(),
                m_controls.nrow());
    } else {
        m_controls.resize(m_time.size(), 0);
    }
    if (m_multipliers.ncol()) {
        OPENSIM_THROW_IF(time.size() != m_multipliers.nrow(), Exception,
                "Expected multipliers to have {} rows but it has {}.",
                time.size(), m_multipliers.nrow());
    } else {
        m_multipliers.resize(m_time.size(), 0);
    }
    m_derivatives.resize(m_time.size(), 0);
    OPENSIM_THROW_IF((int)m_parameter_names.size() != m_parameters.nelt(),
            Exception, "Inconsistent number of parameters.");
}

MocoTrajectory::MocoTrajectory(const SimTK::Vector& time,
        std::vector<std::string> state_names,
        std::vector<std::string> control_names,
        std::vector<std::string> multiplier_names,
        std::vector<std::string> derivative_names,
        std::vector<std::string> parameter_names,
        const SimTK::Matrix& statesTrajectory,
        const SimTK::Matrix& controlsTrajectory,
        const SimTK::Matrix& multipliersTrajectory,
        const SimTK::Matrix& derivativesTrajectory,
        const SimTK::RowVector& parameters)
        : MocoTrajectory(time, state_names, control_names, multiplier_names,
                  parameter_names, statesTrajectory, controlsTrajectory,
                  multipliersTrajectory, parameters) {
    m_derivative_names = derivative_names;
    m_derivatives = derivativesTrajectory;
    OPENSIM_THROW_IF((int)m_derivative_names.size() != m_derivatives.ncol(),
            Exception, "Inconsistent number of derivatives.");
    if (m_derivatives.ncol()) {
        OPENSIM_THROW_IF((int)time.size() != m_derivatives.nrow(), Exception,
                "Inconsistent number of times in derivatives trajectory.");
    } else {
        m_derivatives.resize(m_time.size(), 0);
    }
}

MocoTrajectory::MocoTrajectory(const SimTK::Vector& time,
        const std::map<std::string, NamesAndData<SimTK::Matrix>>& conVars,
        const NamesAndData<SimTK::RowVector>& parameters)
        : MocoTrajectory(time,
                  conVars.count("states") ? conVars.at("states").first
                                          : std::vector<std::string>(),
                  conVars.count("controls") ? conVars.at("controls").first
                                            : std::vector<std::string>(),
                  conVars.count("multipliers") ? conVars.at("multipliers").first
                                               : std::vector<std::string>(),
                  conVars.count("derivatives") ? conVars.at("derivatives").first
                                               : std::vector<std::string>(),
                  parameters.first,
                  conVars.count("states") ? conVars.at("states").second
                                          : SimTK::Matrix(),
                  conVars.count("controls") ? conVars.at("controls").second
                                            : SimTK::Matrix(),
                  conVars.count("multipliers")
                          ? conVars.at("multipliers").second
                          : SimTK::Matrix(),
                  conVars.count("derivatives")
                          ? conVars.at("derivatives").second
                          : SimTK::Matrix(),
                  parameters.second) {}

void MocoTrajectory::setTime(const SimTK::Vector& time) {
    ensureUnsealed();
    OPENSIM_THROW_IF(time.size() != m_time.size(), Exception,
            "Expected {} times but got {}.", m_time.size(), time.size());
    m_time = time;
}

void MocoTrajectory::setState(
        const std::string& name, const SimTK::Vector& trajectory) {
    ensureUnsealed();
    OPENSIM_THROW_IF(trajectory.size() != m_states.nrow(), Exception,
            "For state {}, expected {} elements but got {}.", name,
            m_states.nrow(), trajectory.size());

    auto it = std::find(m_state_names.cbegin(), m_state_names.cend(), name);
    OPENSIM_THROW_IF(it == m_state_names.cend(), Exception,
            "Cannot find state named {}.", name);
    int index = (int)std::distance(m_state_names.cbegin(), it);
    m_states.updCol(index) = trajectory;
}

void MocoTrajectory::setControl(
        const std::string& name, const SimTK::Vector& trajectory) {
    ensureUnsealed();
    OPENSIM_THROW_IF(trajectory.size() != m_controls.nrow(), Exception,
            "For control {}, expected {} elements but got {}.", name,
            m_controls.nrow(), trajectory.size());

    auto it = std::find(m_control_names.cbegin(), m_control_names.cend(), name);
    OPENSIM_THROW_IF(it == m_control_names.cend(), Exception,
            "Cannot find control named {}.", name);
    int index = (int)std::distance(m_control_names.cbegin(), it);
    m_controls.updCol(index) = trajectory;
}

void MocoTrajectory::setMultiplier(
        const std::string& name, const SimTK::Vector& trajectory) {
    ensureUnsealed();
    OPENSIM_THROW_IF(trajectory.size() != m_multipliers.nrow(), Exception,
            "For multiplier {}, expected {} elements but got {}.", name,
            m_multipliers.nrow(), trajectory.size());

    auto it = std::find(
            m_multiplier_names.cbegin(), m_multiplier_names.cend(), name);
    OPENSIM_THROW_IF(it == m_multiplier_names.cend(), Exception,
            "Cannot find multiplier named {}.", name);
    int index = (int)std::distance(m_multiplier_names.cbegin(), it);
    m_multipliers.updCol(index) = trajectory;
}

void MocoTrajectory::setDerivative(
        const std::string& name, const SimTK::Vector& trajectory) {
    ensureUnsealed();
    OPENSIM_THROW_IF(trajectory.size() != m_derivatives.nrow(), Exception,
            "For derivative {}, expected {} elements but got {}.", name,
            m_derivatives.nrow(), trajectory.size());

    auto it = std::find(
            m_derivative_names.cbegin(), m_derivative_names.cend(), name);
    OPENSIM_THROW_IF(it == m_derivative_names.cend(), Exception,
            "Cannot find derivative named {}.", name);
    int index = (int)std::distance(m_derivative_names.cbegin(), it);
    m_derivatives.updCol(index) = trajectory;

}

void MocoTrajectory::setSlack(
        const std::string& name, const SimTK::Vector& trajectory) {
    ensureUnsealed();

    OPENSIM_THROW_IF(trajectory.size() != m_slacks.nrow(), Exception,
            "For slack {}, expected {} elements but got {}.", name,
            m_slacks.nrow(), trajectory.size());

    auto it = std::find(m_slack_names.cbegin(), m_slack_names.cend(), name);
    OPENSIM_THROW_IF(it == m_slack_names.cend(), Exception,
            "Cannot find slack named {}.", name);
    int index = (int)std::distance(m_slack_names.cbegin(), it);
    m_slacks.updCol(index) = trajectory;
}

void MocoTrajectory::appendSlack(
        const std::string& name, const SimTK::Vector& trajectory) {
    ensureUnsealed();

    OPENSIM_THROW_IF(m_time.nrow() == 0, Exception,
            "The time vector must be set before adding slack variables.");
    OPENSIM_THROW_IF(trajectory.size() != m_time.nrow(), Exception,
            "Attempted to add slack {} of length {}, but it is incompatible "
            "with the time vector, which has length {}.",
            name, trajectory.size(), m_time.nrow());

    m_slack_names.push_back(name);
    m_slacks.resizeKeep(m_time.nrow(), m_slacks.ncol() + 1);
    m_slacks.updCol(m_slacks.ncol() - 1) = trajectory;
}

void MocoTrajectory::setParameter(
        const std::string& name, const SimTK::Real& value) {
    ensureUnsealed();

    auto it = std::find(
            m_parameter_names.cbegin(), m_parameter_names.cend(), name);
    OPENSIM_THROW_IF(it == m_parameter_names.cend(), Exception,
            "Cannot find parameter named {}.", name);
    int index = (int)std::distance(m_parameter_names.cbegin(), it);
    m_parameters.updElt(0, index) = value;
}

void MocoTrajectory::setStatesTrajectory(const TimeSeriesTable& states,
        bool allowMissingColumns, bool allowExtraColumns) {
    ensureUnsealed();

    int numTimesTable = (int)states.getNumRows();
    OPENSIM_THROW_IF(numTimesTable < 2, Exception,
            "Cannot interpolate if number of times in table is 0 or 1.");

    const auto& labels = states.getColumnLabels();

    if (!allowMissingColumns) {
        for (const auto& trajectory_state : m_state_names) {
            OPENSIM_THROW_IF(find(labels, trajectory_state) == labels.end(),
                    Exception,
                    "Expected table to contain column '{}'; consider setting "
                    "allowMissingColumns to true.",
                    trajectory_state);
        }
    }

    std::vector<std::string> labelsToUse;
    for (const auto& label : labels) {
        if (find(m_state_names, label) != m_state_names.end()) {
            labelsToUse.push_back(label);
        } else {
            if (!allowExtraColumns) {
                OPENSIM_THROW(Exception,
                        "Column '{}' is not a state in the trajectory; "
                        "consider setting allowExtraColumns to true.",
                        label);
            }
        }
    }

    GCVSplineSet splines(states, labelsToUse, std::min(numTimesTable - 1, 5));

    SimTK::Vector curTime(1, SimTK::NaN);
    for (const auto& label : labelsToUse) {
        auto it = find(m_state_names, label);
        int istate = (int)std::distance(m_state_names.cbegin(), it);
        for (int itime = 0; itime < m_time.size(); ++itime) {
            curTime[0] = m_time[itime];
            m_states(itime, istate) = splines.get(label).calcValue(curTime);
        }
    }
}

void MocoTrajectory::insertStatesTrajectory(
        const TimeSeriesTable& subsetOfStates, bool overwrite) {
    ensureUnsealed();

    const auto origStateNames = m_state_names;
    const auto& labelsToInsert = subsetOfStates.getColumnLabels();
    for (const auto& label : labelsToInsert) {
        auto it = find(m_state_names, label);
        if (it == m_state_names.cend()) { m_state_names.push_back(label); }
    }

    m_states.resizeKeep(getNumTimes(), (int)m_state_names.size());

    const int numTimesTable = (int)subsetOfStates.getNumRows();

    GCVSplineSet splines(subsetOfStates, {}, std::min(numTimesTable - 1, 5));
    SimTK::Vector curTime(1, SimTK::NaN);
    for (const auto& label : labelsToInsert) {
        if (find(origStateNames, label) == origStateNames.cend() || overwrite) {
            auto it = find(m_state_names, label);
            int istate = (int)std::distance(m_state_names.cbegin(), it);
            for (int itime = 0; itime < m_time.size(); ++itime) {
                curTime[0] = m_time[itime];
                m_states(itime, istate) = splines.get(label).calcValue(curTime);
            }
        }
    }
}

void MocoTrajectory::insertControlsTrajectory(
        const TimeSeriesTable& subsetOfControls, bool overwrite) {
    ensureUnsealed();

    const auto origControlNames = m_control_names;
    const auto& labelsToInsert = subsetOfControls.getColumnLabels();
    for (const auto& label : labelsToInsert) {
        auto it = find(m_control_names, label);
        if (it == m_control_names.cend()) { m_control_names.push_back(label); }
    }

    m_controls.resizeKeep(getNumTimes(), (int)m_control_names.size());

    const int numTimesTable = (int)subsetOfControls.getNumRows();

    GCVSplineSet splines(subsetOfControls, {}, std::min(numTimesTable - 1, 5));
    SimTK::Vector curTime(1, SimTK::NaN);
    for (const auto& label : labelsToInsert) {
        if (find(origControlNames, label) == origControlNames.cend()
                || overwrite) {
            auto it = find(m_control_names, label);
            int istate = (int)std::distance(m_control_names.cbegin(), it);
            for (int itime = 0; itime < m_time.size(); ++itime) {
                curTime[0] = m_time[itime];
                m_controls(itime, istate) =
                    splines.get(label).calcValue(curTime);
            }
        }
    }
}

void MocoTrajectory::generateSpeedsFromValues() {
    auto valuesTable = exportToValuesTable();
    int numValues = (int)valuesTable.getNumColumns();
    OPENSIM_THROW_IF(!numValues, Exception,
        "Tried to compute speeds from coordinate values, but no values "
        "exist in the trajectory.");
    m_states.resize(getNumTimes(), 2*numValues);

    // Spline the values trajectory.
    GCVSplineSet splines(valuesTable, {}, std::min(getNumTimes() - 1, 5));

    std::vector<std::string> stateNames;
    const std::vector<std::string>& valueNames =
            valuesTable.getColumnLabels();
    std::vector<std::string> speedNames;
    for (int ivalue = 0; ivalue < numValues; ++ivalue) {
        std::string name(valueNames[ivalue]);
        stateNames.push_back(name);
        auto leafpos = name.find("/value");
        name.replace(leafpos, name.size(), "/speed");
        speedNames.push_back(name);
    }

    SimTK::Vector currTime(1, SimTK::NaN);
    for (int ivalue = 0; ivalue < numValues; ++ivalue) {
        const auto& name = valueNames[ivalue];
        // Compute the derivative from the splined value and assign to this
        // speed memory.
        for (int itime = 0; itime < m_time.size(); ++itime) {
            currTime[0] = m_time[itime];
            m_states(itime, ivalue) = splines.get(name).calcValue(currTime);
            m_states(itime, ivalue + numValues) =
                    splines.get(name).calcDerivative({0}, currTime);
        }
    }

    // Assign state names.
    m_state_names = stateNames;
}

void MocoTrajectory::generateAccelerationsFromValues() {
    auto valuesTable = exportToValuesTable();
    int numValues = (int)valuesTable.getNumColumns();
    OPENSIM_THROW_IF(!numValues, Exception,
        "Tried to compute accelerations from coordinate values, but no values "
        "exist in the trajectory.");
    int numDerivativesWithoutAccelerations =
            getNumDerivativesWithoutAccelerations();
    SimTK::Matrix derivativesWithoutAccelerations =
            getDerivativesWithoutAccelerationsTrajectory();
    m_derivatives.resize(getNumTimes(),
                         numValues + numDerivativesWithoutAccelerations);

    // Spline the values trajectory.
    GCVSplineSet splines(valuesTable, {}, std::min(getNumTimes() - 1, 5));

    const auto& valueNames = valuesTable.getColumnLabels();
    std::vector<std::string> derivativeNamesWithoutAccelerations =
            getDerivativeNamesWithoutAccelerations();
    std::vector<std::string> derivativeNames;
    for (int ivalue = 0; ivalue < numValues; ++ivalue) {
        std::string name(valueNames[ivalue]);
        auto leafpos = name.find("/value");
        name.replace(leafpos, name.size(), "/accel");
        derivativeNames.push_back(name);
    }
    derivativeNames.insert(derivativeNames.end(),
                           derivativeNamesWithoutAccelerations.begin(),
                           derivativeNamesWithoutAccelerations.end());

    SimTK::Vector currTime(1, SimTK::NaN);
    for (int ivalue = 0; ivalue < numValues; ++ivalue) {
        const auto& name = valueNames[ivalue];
        // Compute the derivative from the splined value and assign to this
        // speed memory.
        for (int itime = 0; itime < m_time.size(); ++itime) {
            currTime[0] = m_time[itime];
            m_derivatives(itime, ivalue) =
                    splines.get(name).calcDerivative({0, 0}, currTime);
        }
    }

    // Fill back in any non-acceleration derivative values.
    for (int idv = 0; idv < numDerivativesWithoutAccelerations; ++idv) {
        for (int itime = 0; itime < m_time.size(); ++itime) {
            currTime[0] = m_time[itime];
            m_derivatives(itime, idv + numValues) =
                    derivativesWithoutAccelerations(itime, idv);
        }
    }

    // Assign derivative names.
    m_derivative_names = derivativeNames;
}

void MocoTrajectory::generateAccelerationsFromSpeeds() {
    auto speedsTable = exportToSpeedsTable();
    int numSpeeds = (int)speedsTable.getNumColumns();
    OPENSIM_THROW_IF(!numSpeeds, Exception,
        "Tried to compute accelerations from coordinate speeds, but no speeds "
        "exist in the trajectory.");
    int numDerivativesWithoutAccelerations =
            getNumDerivativesWithoutAccelerations();
    SimTK::Matrix derivativesWithoutAccelerations =
            getDerivativesWithoutAccelerationsTrajectory();
    m_derivatives.resize(getNumTimes(),
                         numSpeeds + numDerivativesWithoutAccelerations);

    // Spline the values trajectory.
    GCVSplineSet splines(speedsTable, {}, std::min(getNumTimes() - 1, 5));

    const auto& speedNames = speedsTable.getColumnLabels();
    std::vector<std::string> derivativeNamesWithoutAccelerations =
            getDerivativeNamesWithoutAccelerations();
    std::vector<std::string> derivativeNames;
    for (int ispeed = 0; ispeed < numSpeeds; ++ispeed) {
        std::string name(speedNames[ispeed]);
        auto leafpos = name.find("/speed");
        name.replace(leafpos, name.size(), "/accel");
        derivativeNames.push_back(name);
    }
    derivativeNames.insert(derivativeNames.end(),
                           derivativeNamesWithoutAccelerations.begin(),
                           derivativeNamesWithoutAccelerations.end());

    SimTK::Vector currTime(1, SimTK::NaN);
    for (int ivalue = 0; ivalue < numSpeeds; ++ivalue) {
        const auto& name = speedNames[ivalue];
        // Compute the derivative from the splined value and assign to this
        // speed memory.
        for (int itime = 0; itime < m_time.size(); ++itime) {
            currTime[0] = m_time[itime];
            m_derivatives(itime, ivalue) =
                    splines.get(name).calcDerivative({0}, currTime);
        }
    }

    // Fill back in any non-acceleration derivative values.
    for (int idv = 0; idv < numDerivativesWithoutAccelerations; ++idv) {
        for (int itime = 0; itime < m_time.size(); ++itime) {
            currTime[0] = m_time[itime];
            m_derivatives(itime, idv + numSpeeds) =
                    derivativesWithoutAccelerations(itime, idv);
        }
    }

    // Assign derivative names.
    m_derivative_names = derivativeNames;
}

void MocoTrajectory::trimToIndices(int newStartIndex, int newFinalIndex) {
    OPENSIM_THROW_IF(newFinalIndex < newStartIndex, Exception,
            fmt::format("Expected newFinalIndex to be greater than "
                        "newStartIndex, but received {} and {} for "
                        "newStartIndex and newFinalIndex, respectively.",
                        newStartIndex, newFinalIndex));
    OPENSIM_THROW_IF(newStartIndex < 0, Exception,
            fmt::format("Expected newStartIndex to be greater than or equal to"
                        "0, but received {}.", newStartIndex));
    OPENSIM_THROW_IF(newFinalIndex > getNumTimes()-1, Exception,
            fmt::format("Expected newFinalIndex to be less than or equal to"
                        "the current final index {}, but received {}.",
                        getNumTimes()-1, newFinalIndex));

    const int newLength = newFinalIndex - newStartIndex + 1;

    const SimTK::Matrix statesBlock =
            m_states(newStartIndex, 0, newLength, m_states.ncol());
    m_states = statesBlock;

    const SimTK::Matrix controlsBlock =
            m_controls(newStartIndex, 0, newLength, m_controls.ncol());
    m_controls = controlsBlock;

    const SimTK::Matrix multipliersBlock =
            m_multipliers(newStartIndex, 0, newLength, m_multipliers.ncol());
    m_multipliers = multipliersBlock;

    const SimTK::Matrix derivativesBlock =
            m_derivatives(newStartIndex, 0, newLength, m_derivatives.ncol());
    m_derivatives = derivativesBlock;

    const SimTK::Matrix slacksBlock =
            m_slacks(newStartIndex, 0, newLength, m_slacks.ncol());
    m_slacks = slacksBlock;

    SimTK::Vector newTime(newLength, 0.0);
    for (int i = 0; i < newLength; ++i) {
        newTime[i] = m_time[i + newStartIndex];
    }

    m_time = newTime;
}

double MocoTrajectory::getInitialTime() const {
    ensureUnsealed();
    OPENSIM_THROW_IF(m_time.size() == 0, Exception, "Time vector is empty.");
    return m_time[0];
}

double MocoTrajectory::getFinalTime() const {
    ensureUnsealed();
    OPENSIM_THROW_IF(m_time.size() == 0, Exception, "Time vector is empty.");
    return m_time[m_time.size() - 1];
}

SimTK::VectorView MocoTrajectory::getState(const std::string& name) const {
    ensureUnsealed();
    auto it = std::find(m_state_names.cbegin(), m_state_names.cend(), name);
    OPENSIM_THROW_IF(it == m_state_names.cend(), Exception,
            "Cannot find state named {}.", name);
    int index = (int)std::distance(m_state_names.cbegin(), it);
    return m_states.col(index);
}
SimTK::VectorView MocoTrajectory::getControl(const std::string& name) const {
    ensureUnsealed();
    auto it = std::find(m_control_names.cbegin(), m_control_names.cend(), name);
    OPENSIM_THROW_IF(it == m_control_names.cend(), Exception,
            "Cannot find control named {}.", name);
    int index = (int)std::distance(m_control_names.cbegin(), it);
    return m_controls.col(index);
}
SimTK::VectorView MocoTrajectory::getMultiplier(const std::string& name) const {
    ensureUnsealed();
    auto it = std::find(
            m_multiplier_names.cbegin(), m_multiplier_names.cend(), name);
    OPENSIM_THROW_IF(it == m_multiplier_names.cend(), Exception,
            "Cannot find multiplier named {}.", name);
    int index = (int)std::distance(m_multiplier_names.cbegin(), it);
    return m_multipliers.col(index);
}
SimTK::VectorView MocoTrajectory::getDerivative(const std::string& name) const {
    ensureUnsealed();
    auto it = std::find(
            m_derivative_names.cbegin(), m_derivative_names.cend(), name);
    OPENSIM_THROW_IF(it == m_derivative_names.cend(), Exception,
            "Cannot find derivative named {}.", name);
    int index = (int)std::distance(m_derivative_names.cbegin(), it);
    return m_derivatives.col(index);
}
SimTK::VectorView MocoTrajectory::getSlack(const std::string& name) const {
    ensureUnsealed();
    auto it = std::find(m_slack_names.cbegin(), m_slack_names.cend(), name);
    OPENSIM_THROW_IF(it == m_slack_names.cend(), Exception,
            "Cannot find slack named {}.", name);
    int index = (int)std::distance(m_slack_names.cbegin(), it);
    return m_slacks.col(index);
}
const SimTK::Real& MocoTrajectory::getParameter(const std::string& name) const {
    ensureUnsealed();
    auto it = std::find(
            m_parameter_names.cbegin(), m_parameter_names.cend(), name);
    OPENSIM_THROW_IF(it == m_parameter_names.cend(), Exception,
            "Cannot find parameter named {}.", name);
    int index = (int)std::distance(m_parameter_names.cbegin(), it);
    return m_parameters.getElt(0, index);
}

double MocoTrajectory::resampleWithNumTimes(int numTimes) {
    ensureUnsealed();
    SimTK::Vector newTime = createVectorLinspace(
            numTimes, m_time[0], m_time[m_time.size() - 1]);
    resample(newTime);
    return newTime[1] - newTime[0];
}
double MocoTrajectory::resampleWithInterval(double desiredTimeInterval) {
    ensureUnsealed();
    // As a guide, solve for num_times in this equation, and convert that to
    // an integer:
    // time_interval = duration / (num_times - 1)
    const auto& duration = m_time[m_time.size() - 1] - m_time[0];
    const int actualNumTimes = (int)ceil(duration / desiredTimeInterval) + 1;
    resampleWithNumTimes(actualNumTimes);
    return duration / ((double)actualNumTimes - 1);
}
double MocoTrajectory::resampleWithFrequency(double desiredFrequency) {
    ensureUnsealed();
    // frequency = num_times / duration, so
    // num_times = ceil(duration * frequency);
    const auto& duration = m_time[m_time.size() - 1] - m_time[0];
    const int actualNumTimes = (int)ceil(duration * desiredFrequency);
    resampleWithNumTimes(actualNumTimes);
    return (double)actualNumTimes / duration;
}
void MocoTrajectory::resample(SimTK::Vector time) {
    ensureUnsealed();
    OPENSIM_THROW_IF(m_time.size() < 2, Exception,
            "Cannot resample if number of times is 0 or 1.");
    OPENSIM_THROW_IF(time[0] < m_time[0], Exception,
            "New initial time ({}) cannot be less than existing initial time "
            "({})",
            time[0], m_time[0]);
    OPENSIM_THROW_IF(time[time.size() - 1] > m_time[m_time.size() - 1],
            Exception,
            "New final time ({}) cannot be less than existing final time ({})",
            time[time.size() - 1], m_time[m_time.size() - 1]);
    for (int itime = 1; itime < time.size(); ++itime) {
        OPENSIM_THROW_IF(time[itime] < time[itime - 1], Exception,
                "New times must be non-decreasing, but time[{}] < time[{}] "
                "({} < {}).",
                itime, itime - 1, time[itime], time[itime - 1]);
    }

    int numStates = (int)m_state_names.size();
    int numControls = (int)m_control_names.size();
    int numMultipliers = (int)m_multiplier_names.size();
    int numDerivatives = (int)m_derivative_names.size();
    int numSlacks = (int)m_slack_names.size();

    // This interpolate step removes any NaN values in the slack variables. It
    // does not resize the slacks trajectory.
    for (int icol = 0; icol < m_slacks.ncol(); ++icol) {
        m_slacks.updCol(icol) =
                interpolate(m_time, m_slacks.col(icol), m_time, true);
    }

    const TimeSeriesTable table = convertToTable();
    const GCVSplineSet splines(table, {}, std::min(m_time.size() - 1, 5));

    m_time = std::move(time);
    const int numTimes = m_time.size();
    m_states.resize(numTimes, numStates);
    m_controls.resize(numTimes, numControls);
    m_multipliers.resize(numTimes, numMultipliers);
    m_derivatives.resize(numTimes, numDerivatives);
    m_slacks.resize(numTimes, numSlacks);
    if (m_time[numTimes - 1] == m_time[0]) {
        // If, for example, all times are 0.0, then we cannot use the spline,
        // which requires strictly increasing time.
        int icol;
        for (icol = 0; icol < numStates; ++icol)
            m_states.updCol(icol).setTo(
                    table.getDependentColumnAtIndex(icol).getElt(0, 0));
        for (int icontr = 0; icontr < numControls; ++icontr, ++icol)
            m_controls.updCol(icontr).setTo(
                    table.getDependentColumnAtIndex(icol).getElt(0, 0));
        for (int imult = 0; imult < numMultipliers; ++imult, ++icol)
            m_multipliers.updCol(imult).setTo(
                    table.getDependentColumnAtIndex(icol).getElt(0, 0));
        for (int ideriv = 0; ideriv < numDerivatives; ++ideriv, ++icol)
            m_derivatives.updCol(ideriv).setTo(
                    table.getDependentColumnAtIndex(icol).getElt(0, 0));
        for (int islack = 0; islack < numSlacks; ++islack, ++icol)
            m_slacks.updCol(islack).setTo(
                    table.getDependentColumnAtIndex(icol).getElt(0, 0));

    } else {
        SimTK::Vector curTime(1);
        for (int itime = 0; itime < m_time.size(); ++itime) {
            curTime[0] = m_time[itime];
            int icol;
            for (icol = 0; icol < numStates; ++icol)
                m_states(itime, icol) = splines[icol].calcValue(curTime);
            for (int icontr = 0; icontr < numControls; ++icontr, ++icol)
                m_controls(itime, icontr) = splines[icol].calcValue(curTime);
            for (int imult = 0; imult < numMultipliers; ++imult, ++icol)
                m_multipliers(itime, imult) = splines[icol].calcValue(curTime);
            for (int ideriv = 0; ideriv < numDerivatives; ++ideriv, ++icol)
                m_derivatives(itime, ideriv) = splines[icol].calcValue(curTime);
            for (int islack = 0; islack < numSlacks; ++islack, ++icol)
                m_slacks(itime, islack) = splines[icol].calcValue(curTime);
        }
    }
}

MocoTrajectory::MocoTrajectory(const std::string& filepath) {
    TimeSeriesTable table(filepath);
    const auto& metadata = table.getTableMetaData();
    // TODO: bug with file adapters.
    // auto numStates = metadata.getValueForKey("num_states").getValue<int>();
    // auto numControls =
    // metadata.getValueForKey("num_controls").getValue<int>(); auto
    // numMultipliers =
    //    metadata.getValueForKey("num_multipliers").getValue<int>();
    // auto numParameters =
    //    metadata.getValueForKey("num_parameters").getValue<int>();
    int numStates;
    SimTK::convertStringTo(
            metadata.getValueForKey("num_states").getValue<std::string>(),
            numStates);
    int numControls;
    SimTK::convertStringTo(
            metadata.getValueForKey("num_controls").getValue<std::string>(),
            numControls);
    int numMultipliers;
    SimTK::convertStringTo(
            metadata.getValueForKey("num_multipliers").getValue<std::string>(),
            numMultipliers);
    int numDerivatives;
    SimTK::convertStringTo(
            metadata.getValueForKey("num_derivatives").getValue<std::string>(),
            numDerivatives);
    int numSlacks;
    SimTK::convertStringTo(
            metadata.getValueForKey("num_slacks").getValue<std::string>(),
            numSlacks);
    int numParameters;
    SimTK::convertStringTo(
            metadata.getValueForKey("num_parameters").getValue<std::string>(),
            numParameters);
    OPENSIM_THROW_IF(numStates < 0, Exception, "Invalid num_states.");
    OPENSIM_THROW_IF(numControls < 0, Exception, "Invalid num_controls.");
    OPENSIM_THROW_IF(numMultipliers < 0, Exception, "Invalid num_multipliers.");
    OPENSIM_THROW_IF(numDerivatives < 0, Exception, "Invalid num_derivatives.");
    OPENSIM_THROW_IF(numSlacks < 0, Exception, "Invalid num_slacks.");
    OPENSIM_THROW_IF(numParameters < 0, Exception, "Invalid num_parameters.");

    const auto& labels = table.getColumnLabels();
    int offset = 0;
    m_state_names.insert(m_state_names.end(), labels.begin() + offset,
            labels.begin() + offset + numStates);
    offset += numStates;
    m_control_names.insert(m_control_names.end(), labels.begin() + offset,
            labels.begin() + offset + numControls);
    offset += numControls;
    m_multiplier_names.insert(m_multiplier_names.end(), labels.begin() + offset,
            labels.begin() + offset + numMultipliers);
    offset += numMultipliers;
    m_derivative_names.insert(m_derivative_names.end(), labels.begin() + offset,
            labels.begin() + offset + numDerivatives);
    offset += numDerivatives;
    m_slack_names.insert(m_slack_names.end(), labels.begin() + offset,
            labels.begin() + offset + numSlacks);
    offset += numSlacks;
    m_parameter_names.insert(
            m_parameter_names.end(), labels.begin() + offset, labels.end());

    OPENSIM_THROW_IF(numStates + numControls + numMultipliers + numDerivatives +
                                     numSlacks + numParameters !=
                             (int)table.getNumColumns(),
            Exception,
            "Expected num_states + num_controls + num_multipliers + "
            "num_derivatives + num_slacks + num_parameters = "
            "number of columns, but "
            "num_states={}, num_controls={}, "
            "num_multipliers={}, num_derivatives={}, num_slacks={}, "
            "num_parameters={}, number of columns={}.",
            numStates, numControls, numMultipliers, numDerivatives, numSlacks,
            numParameters, table.getNumColumns());

    const auto& time = table.getIndependentColumn();
    m_time = SimTK::Vector((int)time.size(), time.data());

    if (numStates) {
        m_states = table.getMatrixBlock(0, 0, table.getNumRows(), numStates);
    } else {
        m_states.resize((int)table.getNumRows(), 0);
    }
    if (numControls) {
        m_controls = table.getMatrixBlock(
                0, numStates, table.getNumRows(), numControls);
    } else {
        m_controls.resize((int)table.getNumRows(), 0);
    }
    if (numMultipliers) {
        m_multipliers = table.getMatrixBlock(
                0, numStates + numControls, table.getNumRows(), numMultipliers);
    } else {
        m_multipliers.resize((int)table.getNumRows(), 0);
    }
    if (numDerivatives) {
        m_derivatives = table.getMatrixBlock(0,
                numStates + numControls + numMultipliers, table.getNumRows(),
                numDerivatives);
    } else {
        m_derivatives.resize((int)table.getNumRows(), 0);
    }
    if (numSlacks) {
        m_slacks = table.getMatrixBlock(0,
                numStates + numControls + numMultipliers + numDerivatives,
                table.getNumRows(), numSlacks);
    } else {
        m_slacks.resize((int)table.getNumRows(), 0);
    }
    if (numParameters) {
        m_parameters = table.getMatrixBlock(0,
                                    numStates + numControls + numMultipliers +
                                            numDerivatives + numSlacks,
                                    1, numParameters)
                               .getAsRowVectorBase();
    }
}

void MocoTrajectory::write(const std::string& filepath) const {
    ensureUnsealed();
    STOFileAdapter::write(convertToTable(), filepath);
}

TimeSeriesTable MocoTrajectory::convertToTable() const {
    ensureUnsealed();
    std::vector<double> time(&m_time[0], &m_time[0] + m_time.size());

    // Concatenate the state, control, multiplier, and parameter names in a
    // single vector.
    std::vector<std::string> labels = m_state_names;
    labels.insert(labels.end(), m_control_names.begin(), m_control_names.end());
    labels.insert(
            labels.end(), m_multiplier_names.begin(), m_multiplier_names.end());
    labels.insert(
            labels.end(), m_derivative_names.begin(), m_derivative_names.end());
    labels.insert(labels.end(), m_slack_names.begin(), m_slack_names.end());
    labels.insert(
            labels.end(), m_parameter_names.begin(), m_parameter_names.end());
    int numTimes = (int)m_time.size();
    int numStates = (int)m_state_names.size();
    int numControls = (int)m_control_names.size();
    int numMultipliers = (int)m_multiplier_names.size();
    int numDerivatives = (int)m_derivative_names.size();
    int numSlacks = (int)m_slack_names.size();
    int numParameters = (int)m_parameter_names.size();

    SimTK::Matrix data(numTimes, (int)labels.size());
    int startCol = 0;
    if (numStates) {
        data.updBlock(0, startCol, numTimes, numStates) = m_states;
        startCol += numStates;
    }
    if (numControls) {
        data.updBlock(0, startCol, numTimes, numControls) = m_controls;
        startCol += numControls;
    }
    if (numMultipliers) {
        data.updBlock(0, startCol, numTimes, numMultipliers) = m_multipliers;
        startCol += numMultipliers;
    }
    if (numDerivatives) {
        data.updBlock(0, startCol, numTimes, numDerivatives) = m_derivatives;
        startCol += numDerivatives;
    }
    if (numSlacks) {
        data.updBlock(0, startCol, numTimes, numSlacks) = m_slacks;
        startCol += numSlacks;
    }
    if (numParameters) {
        // First row of table contains parameter values.
        data.updBlock(0, startCol, 1, numParameters) = m_parameters;
        // Remaining rows of table contain NaNs in parameter columns.
        SimTK::Matrix parameter_nan_rows(
                numTimes - 1, (int)m_parameter_names.size());
        parameter_nan_rows.setToNaN();
        data.updBlock(1, startCol, numTimes - 1, numParameters) =
                parameter_nan_rows;
    }
    TimeSeriesTable table;
    try {
        table = TimeSeriesTable(time, data, labels);
    } catch (const TimestampGreaterThanEqualToNext&) {
        // TimeSeriesTable requires monotonically increasing time, but this
        // might not be true for trajectories. Create the table with complying
        // times then hack in to set times back to what the trajectory contains.
        std::vector<double> tempTime(time.size());
        for (int i = 0; i < (int)tempTime.size(); ++i)
            tempTime[i] = -1000.0 + i;
        table = TimeSeriesTable(tempTime, data, labels);
        const_cast<std::vector<double>&>(table.getIndependentColumn()) = time;
    }
    // TODO table.updTableMetaData().setValueForKey("header", m_name);
    // table.updTableMetaData().setValueForKey("num_states", numStates);
    // table.updTableMetaData().setValueForKey("num_controls", numControls);
    // table.updTableMetaData().setValueForKey("num_multipliers",
    // numMultipliers); table.updTableMetaData().setValueForKey("num_slacks",
    // numSlacks); table.updTableMetaData().setValueForKey("num_parameters",
    // numParameters);
    table.updTableMetaData().setValueForKey(
            "num_states", std::to_string(numStates));
    table.updTableMetaData().setValueForKey(
            "num_controls", std::to_string(numControls));
    table.updTableMetaData().setValueForKey(
            "num_multipliers", std::to_string(numMultipliers));
    table.updTableMetaData().setValueForKey(
            "num_derivatives", std::to_string(numDerivatives));
    table.updTableMetaData().setValueForKey(
            "num_slacks", std::to_string(numSlacks));
    table.updTableMetaData().setValueForKey(
            "num_parameters", std::to_string(numParameters));
    table.updTableMetaData().setValueForKey(
            "inDegrees", std::string("no"));
    convertToTableImpl(table);
    return table;
}

TimeSeriesTable MocoTrajectory::exportToStatesTable() const {
    ensureUnsealed();
    TimeSeriesTable states(
            std::vector<double>(&m_time[0], &m_time[0] + m_time.size()),
            m_states, m_state_names);
    states.addTableMetaData("inDegrees", std::string("no"));
    return states;
}

TimeSeriesTable MocoTrajectory::exportToControlsTable() const {
    ensureUnsealed();
    return {std::vector<double>(&m_time[0], &m_time[0] + m_time.size()),
            m_controls, m_control_names};
}

TimeSeriesTable MocoTrajectory::exportToMultipliersTable() const {
    ensureUnsealed();
    return {std::vector<double>(&m_time[0], &m_time[0] + m_time.size()),
            m_multipliers, m_multiplier_names};
}

TimeSeriesTable MocoTrajectory::exportToDerivativesTable() const {
    ensureUnsealed();
    return {std::vector<double>(&m_time[0], &m_time[0] + m_time.size()),
            m_derivatives, m_derivative_names};
}

TimeSeriesTable MocoTrajectory::exportToValuesTable() const {
    ensureUnsealed();
    return {std::vector<double>(&m_time[0], &m_time[0] + m_time.size()),
            getValuesTrajectory(), getValueNames()};
}

TimeSeriesTable MocoTrajectory::exportToSpeedsTable() const {
    ensureUnsealed();
    return {std::vector<double>(&m_time[0], &m_time[0] + m_time.size()),
            getSpeedsTrajectory(), getSpeedNames()};
}

TimeSeriesTable MocoTrajectory::exportToAccelerationsTable() const {
    ensureUnsealed();
    return {std::vector<double>(&m_time[0], &m_time[0] + m_time.size()),
            getAccelerationsTrajectory(), getAccelerationNames()};
}

TimeSeriesTable
MocoTrajectory::exportToDerivativesWithoutAccelerationsTable() const {
    ensureUnsealed();
    return {std::vector<double>(&m_time[0], &m_time[0] + m_time.size()),
            getDerivativesWithoutAccelerationsTrajectory(),
            getDerivativeNamesWithoutAccelerations()};
}

StatesTrajectory MocoTrajectory::exportToStatesTrajectory(
        const MocoProblem& problem) const {
    ensureUnsealed();
    // TODO update when we support multiple phases.
    const auto& model = problem.getPhase(0).getModelProcessor().process();
    return exportToStatesTrajectory(model);
}

StatesTrajectory MocoTrajectory::exportToStatesTrajectory(
        const Model& model) const {
    ensureUnsealed();
    TimeSeriesTable states = exportToStatesTable();
    // TODO update when we support multiple phases.
    return StatesTrajectory::createFromStatesTable(model, states, true);
}

namespace {
template <typename T>
void randomizeMatrix(bool add, const SimTK::Random& randGen, T& mat) {
    for (int i = 0; i < mat.nrow(); ++i) {
        for (int j = 0; j < mat.ncol(); ++j) {
            auto& elt = mat.updElt(i, j);
            const double rand = randGen.getValue();
            if (add) {
                elt += rand;
            } else {
                elt = rand;
            }
        }
    }
}
} // namespace

void MocoTrajectory::randomize(bool add, const SimTK::Random& randGen) {
    ensureUnsealed();
    randomizeMatrix(add, randGen, m_states);
    randomizeMatrix(add, randGen, m_controls);
    randomizeMatrix(add, randGen, m_multipliers);
    randomizeMatrix(add, randGen, m_derivatives);
    randomizeMatrix(add, randGen, m_slacks);
    randomizeMatrix(add, randGen, m_parameters);
}

/*static*/ MocoTrajectory MocoTrajectory::createFromStatesControlsTables(
        const MocoProblemRep& /*problem*/,
        const TimeSeriesTable& statesTrajectory,
        const TimeSeriesTable& controlsTrajectory) {
    const int statesNumRows = (int)statesTrajectory.getNumRows();
    const int controlsNumRows = (int)controlsTrajectory.getNumRows();
    OPENSIM_THROW_IF(statesNumRows != controlsNumRows, Exception,
            "Expected statesTrajectory ({} rows) and controlsTrajectory ({} "
            "rows) to have the same number of rows.",
            statesNumRows, controlsNumRows);
    // TODO interpolate instead of creating this error.
    for (int i = 0; i < statesNumRows; ++i) {
        const auto& statesTime = statesTrajectory.getIndependentColumn()[i];
        const auto& controlsTime = controlsTrajectory.getIndependentColumn()[i];
        OPENSIM_THROW_IF(statesTime != controlsTime, Exception,
                "Expected time columns of statesTrajectory and "
                "controlsTrajectory to match, but they differ at i "
                "= {} (states time: {}; controls time: {}).",
                i, statesTime, controlsTime);
    }

    // TODO Support controlsTrajectory being empty.

    const auto& statesTimes = statesTrajectory.getIndependentColumn();
    // The "true" means to not copy the data.
    SimTK::Vector time((int)statesTimes.size(), statesTimes.data(), true);

    // TODO MocoProblem should be able to produce a MocoTrajectory template;
    // it's what knows the state, control, and parameter names.
    return MocoTrajectory(time, statesTrajectory.getColumnLabels(),
            controlsTrajectory.getColumnLabels(), {}, // TODO (multiplier_names)
            {},                                       // TODO (parameter_names)
            statesTrajectory.getMatrix(), controlsTrajectory.getMatrix(),
            SimTK::Matrix(0, 0),  // TODO (multipliersTrajectory)
            SimTK::RowVector(0)); // TODO (parameters)
}

bool MocoTrajectory::isCompatible(const MocoProblemRep& mp,
        bool requireAccelerations, bool throwOnError) const {
    ensureUnsealed();
    // Slack variables might be solver dependent, so we can't include them in
    // the compatibility check.

    auto compare = [&throwOnError](
            std::string varType, std::vector<std::string> trajNames,
            std::vector<std::string> probNames,
            std::string message = "") {
        std::sort(trajNames.begin(), trajNames.end());
        std::sort(probNames.begin(), probNames.end());
        if (trajNames == probNames) return true;
        if (!throwOnError && !Logger::shouldLog(Logger::Level::Debug)) {
            return false;
        }

        int sum = (int)trajNames.size() + (int)probNames.size();

        std::stringstream ss;

        // http://www.cplusplus.com/reference/algorithm/set_difference/
        {
            std::vector<std::string> inTrajNotProb(sum);
            auto inTrajNotProbEnd = std::set_difference(trajNames.begin(),
                    trajNames.end(), probNames.begin(), probNames.end(),
                    inTrajNotProb.begin());
            inTrajNotProb.resize(inTrajNotProbEnd - inTrajNotProb.begin());
            ss << "The trajectory and provided problem are not compatible. ";
            if (!inTrajNotProb.empty()) {
                ss << "The following " << varType
                   << " are in the trajectory but not the problem:\n";
                for (const auto& name : inTrajNotProb) {
                    ss << "  " << name << "\n";
                }
            }
        }

        {
            std::vector<std::string> inProbNotTraj(sum);
            auto inProbNotTrajEnd = std::set_difference(probNames.begin(),
                    probNames.end(), trajNames.begin(), trajNames.end(),
                    inProbNotTraj.begin());
            inProbNotTraj.resize(inProbNotTrajEnd - inProbNotTraj.begin());
            if (!inProbNotTraj.empty()) {
                ss << "The following " << varType
                   << " are in the problem but not the trajectory:\n";
                for (const auto& name : inProbNotTraj) {
                    ss << "  " << name << "\n";
                }
            }
        }
        ss << message << "\n";

        if (throwOnError) {
            throw Exception(__FILE__, __LINE__,
                    "MocoTrajectory::isCompatible()", ss.str());
        }
        log_debug(ss.str());
        return false;
    };

    auto mpsn = mp.createStateInfoNames();

    if (!compare("state(s)", m_state_names, mpsn)) return false;
    if (!compare("control(s)", m_control_names, mp.createControlInfoNames()))
        return false;
    if (!compare("multiplier(s)", m_multiplier_names,
                mp.createMultiplierInfoNames()))
        return false;
    if (!compare("parameter(s)", m_parameter_names, mp.createParameterNames()))
        return false;

    std::vector<std::string> mpdn; // Component derivative names only.
    const auto& implicitComponentRefs = mp.getImplicitComponentReferencePtrs();
    for (const auto& compRef : implicitComponentRefs) {
        const auto& derivName =
                compRef.second->getAbsolutePathString() + "/" + compRef.first;
        mpdn.push_back(derivName);
    }
    if (requireAccelerations) {
        // Create the expected names for the derivative variables.
        for (auto name : mpsn) {
            auto leafpos = name.find("value");
            if (leafpos != std::string::npos) {
                name.replace(leafpos, name.size(), "accel");
                mpdn.push_back(name);
            }
        }
    }

    return compare("derivative(s)", m_derivative_names, mpdn);
}

bool MocoTrajectory::isNumericallyEqual(
        const MocoTrajectory& other, double tol) const {
    ensureUnsealed();

    return m_state_names == other.m_state_names &&
           m_control_names == other.m_control_names &&
           m_multiplier_names == other.m_multiplier_names &&
           m_derivative_names == other.m_derivative_names &&
           // TODO include slack variables?
           // m_slack_names == other.m_slack_names &&
           m_parameter_names == other.m_parameter_names &&
           SimTK::Test::numericallyEqual(m_time, other.m_time, 1, tol) &&
           SimTK::Test::numericallyEqual(m_states, other.m_states, 1, tol) &&
           SimTK::Test::numericallyEqual(
                   m_controls, other.m_controls, 1, tol) &&
           SimTK::Test::numericallyEqual(
                   m_multipliers, other.m_multipliers, 1, tol) &&
           SimTK::Test::numericallyEqual(
                   m_derivatives, other.m_derivatives, 1, tol)
           // TODO include slack variables?
           //&& SimTK::Test::numericallyEqual(m_slacks, other.m_slacks, 1, tol)
           && SimTK::Test::numericallyEqual(
                      m_parameters, other.m_parameters, 1, tol);
}

namespace {
using VecStr = std::vector<std::string>;

// Check that two different vectors of strings have the same contents.
bool sameContents(VecStr v1, VecStr v2) {
    std::sort(v1.begin(), v1.end());
    std::sort(v2.begin(), v2.end());
    return v1 == v2;
}

// Check that `a` is a subset of both `b` and `c`.
void checkContains(std::string type, VecStr a, VecStr b, VecStr c) {
    // set_difference requires sorted containers.
    std::sort(a.begin(), a.end());
    std::sort(b.begin(), b.end());
    std::sort(c.begin(), c.end());
    std::vector<std::string> diff;
    std::set_difference(
            a.begin(), a.end(), b.begin(), b.end(), std::back_inserter(diff));
    if (!diff.empty()) {
        std::string msg = "Expected this trajectory's " + type +
                          " names to "
                          "contain the following:";
        for (const auto& elem : diff) msg += "\n  " + elem;
        OPENSIM_THROW(Exception, msg);
    }
    diff.clear();
    std::set_difference(
            a.begin(), a.end(), c.begin(), c.end(), std::back_inserter(diff));
    if (!diff.empty()) {
        std::string msg = "Expected the other trajectory's " + type +
                          " names to contain the following:";
        for (const auto& elem : diff) msg += "\n  " + elem;
        OPENSIM_THROW(Exception, msg);
    }
}

} // anonymous namespace

double MocoTrajectory::compareContinuousVariablesRMSInternal(
        const MocoTrajectory& other, std::vector<std::string> stateNames,
        std::vector<std::string> controlNames,
        std::vector<std::string> multiplierNames,
        std::vector<std::string> derivativeNames) const {
    ensureUnsealed();

    // Process state, control, multiplier, and derivative names.
    // ---------------------------------------------------------
    if (stateNames.empty()) {
        OPENSIM_THROW_IF(!sameContents(m_state_names, other.m_state_names),
                Exception,
                "Expected both trajectories to have the same state names; consider "
                "specifying the states to compare.");
        stateNames = m_state_names;
    } else if (stateNames.size() == 1 && stateNames[0] == "none") {
        stateNames.clear();
    } else {
        checkContains("state", stateNames, m_state_names, other.m_state_names);
    }
    if (controlNames.empty()) {
        OPENSIM_THROW_IF(!sameContents(m_control_names, other.m_control_names),
                Exception,
                "Expected both trajectories to have the same control names; "
                "consider specifying the controls to compare.");
        controlNames = m_control_names;
    } else if (controlNames.size() == 1 && controlNames[0] == "none") {
        controlNames.clear();
    } else {
        std::sort(controlNames.begin(), controlNames.end());
        checkContains("control", controlNames, m_control_names,
                other.m_control_names);
    }
    if (multiplierNames.empty()) {
        OPENSIM_THROW_IF(
                !sameContents(m_multiplier_names, other.m_multiplier_names),
                Exception,
                "Expected both trajectories to have the same multiplier names; "
                "consider specifying the multipliers to compare.");
        multiplierNames = m_multiplier_names;
    } else if (multiplierNames.size() == 1 && multiplierNames[0] == "none") {
        multiplierNames.clear();
    } else {
        checkContains("multiplier", multiplierNames, m_multiplier_names,
                other.m_multiplier_names);
    }
    if (derivativeNames.empty()) {
        OPENSIM_THROW_IF(
                !sameContents(m_derivative_names, other.m_derivative_names),
                Exception,
                "Expected both trajectories to have the same derivative names; "
                "consider specifying the derivatives to compare.");
        derivativeNames = m_derivative_names;
    } else if (derivativeNames.size() == 1 && derivativeNames[0] == "none") {
        derivativeNames.clear();
    } else {
        checkContains("derivative", derivativeNames, m_derivative_names,
                other.m_derivative_names);
    }
    const int numColumns = int(stateNames.size() + controlNames.size() +
                               multiplierNames.size() + derivativeNames.size());
    if (numColumns == 0) return 0;

    std::vector<double> selfTime =
            std::vector<double>(&m_time[0], &m_time[0] + m_time.size());
    std::vector<double> otherTime = std::vector<double>(
            &other.m_time[0], &other.m_time[0] + other.m_time.size());

    const auto initialTime = std::min(selfTime.front(), otherTime.front());
    const auto finalTime = std::max(selfTime.back(), otherTime.back());
    const auto numTimes = std::max(getNumTimes(), other.getNumTimes());
    // Times to use for integrating over time.
    auto integTime = createVectorLinspace(numTimes, initialTime, finalTime);
    const auto timeInterval = integTime[1] - integTime[0];

    auto integralSumSquaredError =
            [&selfTime, &otherTime, &numTimes, &integTime, &timeInterval](
                    const VecStr& namesToUse, const SimTK::Matrix& selfData,
                    const VecStr& selfNames, const SimTK::Matrix& otherData,
                    const VecStr& otherNames) -> double {
        if (namesToUse.empty()) return 0;

        TimeSeriesTable selfTable(selfTime, selfData, selfNames);
        TimeSeriesTable otherTable(otherTime, otherData, otherNames);

        GCVSplineSet self(selfTable, namesToUse,
                std::min((int)selfTable.getNumRows() - 1, 5));
        GCVSplineSet other(otherTable, namesToUse,
                std::min((int)otherTable.getNumRows() - 1, 5));

        SimTK::Vector sumSquaredError(numTimes, 0.0);
        for (int itime = 0; itime < numTimes; ++itime) {
            const auto& curTime = integTime[itime];
            SimTK::Vector curTimeVec(1, curTime);
            bool selfInRange =
                    self.getMinX() <= curTime && curTime <= self.getMaxX();
            bool otherInRange =
                    other.getMinX() <= curTime && curTime <= other.getMaxX();
            for (int iname = 0; iname < (int)namesToUse.size(); ++iname) {
                double selfValue =
                        selfInRange ? self.get(iname).calcValue(curTimeVec) : 0;
                double otherValue =
                        otherInRange ? other.get(iname).calcValue(curTimeVec)
                                     : 0;
                sumSquaredError[itime] += SimTK::square(selfValue - otherValue);
            }
        }
        // Trapezoidal rule for uniform grid:
        // dt / 2 (f_0 + 2f_1 + 2f_2 + 2f_3 + ... + 2f_{N-1} + f_N)
        assert(numTimes > 2);
        return timeInterval / 2.0 *
               (sumSquaredError.sum() + sumSquaredError(1, numTimes - 2).sum());
    };

    const auto stateISS = integralSumSquaredError(stateNames, m_states,
            m_state_names, other.m_states, other.m_state_names);
    const auto controlISS = integralSumSquaredError(controlNames, m_controls,
            m_control_names, other.m_controls, other.m_control_names);
    const auto multiplierISS = integralSumSquaredError(multiplierNames,
            m_multipliers, m_multiplier_names, other.m_multipliers,
            other.m_multiplier_names);
    const auto derivativeISS = integralSumSquaredError(derivativeNames,
            m_derivatives, m_derivative_names, other.m_derivatives,
            other.m_derivative_names);

    // sqrt(1/(T*N) * integral_t (sum_is error_is^2 + sum_ic error_ic^2
    //                                          + sum_im error_im^2)
    // `is`: index for states; `ic`: index for controls;
    // `im`: index for multipliers.
    const double ISS = stateISS + controlISS + multiplierISS + derivativeISS;
    return sqrt(ISS / (finalTime - initialTime) / numColumns);
}

double MocoTrajectory::compareContinuousVariablesRMS(
        const MocoTrajectory& other,
        std::map<std::string, std::vector<std::string>> cols) const {
    ensureUnsealed();
    for (auto kv : cols) {
        OPENSIM_THROW_IF(find(m_allowedKeys, kv.first) == m_allowedKeys.cend(),
                Exception, "Key '{}' is not allowed.", kv.first);
    }
    if (cols.size() == 0) {
        return compareContinuousVariablesRMSInternal(other);
    }
    static const std::vector<std::string> none{"none"};
    return compareContinuousVariablesRMSInternal(other,
            cols.count("states") ? cols.at("states") : none,
            cols.count("controls") ? cols.at("controls") : none,
            cols.count("multipliers") ? cols.at("multipliers") : none,
            cols.count("derivatives") ? cols.at("derivatives") : none);
}

double MocoTrajectory::compareContinuousVariablesRMSPattern(
        const MocoTrajectory& other, std::string columnType,
        std::string pattern) const {
    ensureUnsealed();
    const std::vector<std::string>* names;
    if (columnType == "states") {
        names = &m_state_names;
    } else if (columnType == "controls") {
        names = &m_control_names;
    } else if (columnType == "multipliers") {
        names = &m_multiplier_names;
    } else if (columnType == "derivatives") {
        names = &m_derivative_names;
    } else {
        OPENSIM_THROW(
                Exception, "Column type '{}' is not allowed.", columnType);
    }
    std::vector<std::string> namesToUse;
    std::regex regex(pattern);
    for (const auto& name : *names) {
        if (std::regex_match(name, regex)) {
            namesToUse.push_back(name);
        }
    }
    return compareContinuousVariablesRMS(other, {{columnType, namesToUse}});
}

double MocoTrajectory::compareParametersRMS(const MocoTrajectory& other,
        std::vector<std::string> parameterNames) const {
    ensureUnsealed();

    // Process parameter names.
    // ------------------------
    if (parameterNames.empty()) {
        OPENSIM_THROW_IF(
                !sameContents(m_parameter_names, other.m_parameter_names),
                Exception,
                "Expected both trajectories to have the same parameter names; "
                "consider "
                "specifying the parameters to compare.");
        parameterNames = m_parameter_names;
    } else {
        checkContains("parameter", parameterNames, m_parameter_names,
                other.m_parameter_names);
    }

    double sumSquaredError = 0;
    for (auto& name : parameterNames) {
        const SimTK::Real& selfValue = this->getParameter(name);
        const SimTK::Real& otherValue = other.getParameter(name);
        sumSquaredError += SimTK::square(selfValue - otherValue);
    }

    return sqrt(sumSquaredError / parameterNames.size());
}

void MocoTrajectory::ensureUnsealed() const {
    OPENSIM_THROW_IF(m_sealed, MocoTrajectoryIsSealed);
}

std::vector<std::string> MocoSolution::getObjectiveTermNames() const {
    ensureUnsealed();
    std::vector<std::string> names;
    for (const auto& entry : m_objectiveBreakdown) {
        names.push_back(entry.first);
    }
    return names;
}

double MocoSolution::getObjectiveTerm(const std::string& name) const {
    ensureUnsealed();
    for (const auto& entry : m_objectiveBreakdown) {
        if (entry.first == name) {
            return entry.second;
        }
    }
    OPENSIM_THROW(Exception, "Objective term '{}' not found.", name);
}

double MocoSolution::getObjectiveTermByIndex(int index) const {
    ensureUnsealed();
    OPENSIM_THROW_IF(
            index < 0, Exception, "Expected index to be non-negative.");
    OPENSIM_THROW_IF(index >= (int)m_objectiveBreakdown.size(), Exception,
            "Expected index ({}) to be less than the number of "
            "objective terms ({}).",
            index, m_objectiveBreakdown.size());
    return m_objectiveBreakdown[index].second;
}

void MocoSolution::printObjectiveBreakdown() const {
    ensureUnsealed();
    if (m_objectiveBreakdown.empty()) {
        log_cout("No terms or no breakdown available");
        return;
    }
    for (const auto& entry : m_objectiveBreakdown) {
        log_cout("{}: {}", entry.first, entry.second);
    }
}

void MocoSolution::convertToTableImpl(TimeSeriesTable& table) const {
    std::string success = m_success ? "true" : "false";
    table.updTableMetaData().setValueForKey("success", success);
    table.updTableMetaData().setValueForKey("status", m_status);
    table.updTableMetaData().setValueForKey(
            "objective", std::to_string(m_objective));
    table.updTableMetaData().setValueForKey(
            "num_iterations", std::to_string(m_numIterations));
    table.updTableMetaData().setValueForKey(
            "solver_duration", std::to_string(m_solverDuration));
    for (const auto& entry : m_objectiveBreakdown) {
        table.updTableMetaData().setValueForKey(
                "objective_" + entry.first, std::to_string(entry.second));

    }
}
