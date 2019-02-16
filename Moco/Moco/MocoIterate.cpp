/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoIterate.cpp                                              *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
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
#include "MocoIterate.h"

#include "MocoProblem.h"
#include "MocoUtilities.h"

#include <OpenSim/Common/FileAdapter.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

MocoIterate::MocoIterate(const SimTK::Vector& time,
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
                format("Expected states to have %i rows but it has %i.",
                        time.size(), m_states.nrow()));
    }
    if (m_controls.ncol()) {
        OPENSIM_THROW_IF(time.size() != m_controls.nrow(), Exception,
                format("Expected controls to have %i rows but it has %i.",
                        time.size(), m_controls.nrow()));
    }
    if (m_multipliers.ncol()) {
        OPENSIM_THROW_IF(time.size() != m_multipliers.nrow(), Exception,
                format("Expected multipliers to have %i rows but it has %i.",
                        time.size(), m_multipliers.nrow()));
    }
    OPENSIM_THROW_IF((int)m_parameter_names.size() != m_parameters.nelt(),
            Exception, "Inconsistent number of parameters.");
}

MocoIterate::MocoIterate(const SimTK::Vector& time,
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
        : MocoIterate(time, state_names, control_names, multiplier_names,
                  parameter_names, statesTrajectory, controlsTrajectory,
                  multipliersTrajectory, parameters) {
    m_derivative_names = derivative_names;
    m_derivatives = derivativesTrajectory;
    OPENSIM_THROW_IF((int)m_derivative_names.size() != m_derivatives.ncol(),
            Exception, "Inconsistent number of derivatives.");
    if (m_derivatives.ncol()) {
        OPENSIM_THROW_IF((int)time.size() != m_derivatives.nrow(), Exception,
                "Inconsistent number of times in derivatives trajectory.");
    }
}

void MocoIterate::setTime(const SimTK::Vector& time) {
    ensureUnsealed();
    OPENSIM_THROW_IF(time.size() != m_time.size(), Exception, format(
            "Expected %i times but got %i.", m_time.size(), time.size()));
    m_time = time;
}

void MocoIterate::setState(
        const std::string& name, const SimTK::Vector& trajectory) {
    ensureUnsealed();
    OPENSIM_THROW_IF(trajectory.size() != m_states.nrow(), Exception, format(
            "For state %s, expected %i elements but got %i.", name,
            m_states.nrow(), trajectory.size()));

    auto it = std::find(m_state_names.cbegin(), m_state_names.cend(), name);
    OPENSIM_THROW_IF(it == m_state_names.cend(), Exception,
            format("Cannot find state named %s.", name));
    int index = (int)std::distance(m_state_names.cbegin(), it);
    m_states.updCol(index) = trajectory;
}

void MocoIterate::setControl(
        const std::string& name, const SimTK::Vector& trajectory) {
    ensureUnsealed();
    OPENSIM_THROW_IF(trajectory.size() != m_controls.nrow(), Exception,
            format("For control %s, expected %i elements but got %i.",
                    name, m_controls.nrow(), trajectory.size()));

    auto it = std::find(m_control_names.cbegin(), m_control_names.cend(), name);
    OPENSIM_THROW_IF(it == m_control_names.cend(), Exception,
            format("Cannot find control named %s.", name));
    int index = (int)std::distance(m_control_names.cbegin(), it);
    m_controls.updCol(index) = trajectory;
}

void MocoIterate::setMultiplier(
        const std::string& name, const SimTK::Vector& trajectory) {
    ensureUnsealed();
    OPENSIM_THROW_IF(trajectory.size() != m_multipliers.nrow(), Exception,
            format("For multiplier %s, expected %i elements but got %i.",
            name, m_multipliers.nrow(), trajectory.size()));

    auto it = std::find(
            m_multiplier_names.cbegin(), m_multiplier_names.cend(), name);
    OPENSIM_THROW_IF(it == m_multiplier_names.cend(), Exception,
            format("Cannot find multiplier named %s.", name));
    int index = (int)std::distance(m_multiplier_names.cbegin(), it);
    m_multipliers.updCol(index) = trajectory;
}

void MocoIterate::setSlack(
        const std::string& name, const SimTK::Vector& trajectory) {
    ensureUnsealed();

    OPENSIM_THROW_IF(trajectory.size() != m_slacks.nrow(), Exception,
        format("For slack %s, expected %i elements but got %i.",
                name, m_slacks.nrow(), trajectory.size()));

    auto it = std::find(m_slack_names.cbegin(), m_slack_names.cend(), name);
    OPENSIM_THROW_IF(it == m_slack_names.cend(), Exception,
        format("Cannot find slack named %s.", name));
    int index = (int)std::distance(m_slack_names.cbegin(), it);
    m_slacks.updCol(index) = trajectory;
}

void MocoIterate::appendSlack(
        const std::string& name, const SimTK::Vector& trajectory) {
    ensureUnsealed();

    OPENSIM_THROW_IF(m_time.nrow() == 0, Exception,
        "The time vector must be set before adding slack variables.");
    OPENSIM_THROW_IF(trajectory.size() != m_time.nrow(), Exception,
            format("Attempted to add slack %s of length %i, but it is "
                   "incompatible with the time vector, which has length %i.",
                    name, trajectory.size(), m_time.nrow()));

    m_slack_names.push_back(name);
    m_slacks.resizeKeep(m_time.nrow(), m_slacks.ncol() + 1);
    m_slacks.updCol(m_slacks.ncol() - 1) = trajectory;
}

void MocoIterate::setParameter(
        const std::string& name, const SimTK::Real& value) {
    ensureUnsealed();

    auto it = std::find(
            m_parameter_names.cbegin(), m_parameter_names.cend(), name);
    OPENSIM_THROW_IF(it == m_parameter_names.cend(), Exception,
            format("Cannot find parameter named %s.", name));
    int index = (int)std::distance(m_parameter_names.cbegin(), it);
    m_parameters.updElt(0, index) = value;
}

void MocoIterate::setStatesTrajectory(const TimeSeriesTable& states,
        bool allowMissingColumns, bool allowExtraColumns) {
    ensureUnsealed();

    int numTimesTable = (int)states.getNumRows();
    OPENSIM_THROW_IF(numTimesTable < 2, Exception,
            "Cannot interpolate if number of times in table is 0 or 1.");

    const auto& labels = states.getColumnLabels();

    auto find = [](const std::vector<std::string>& v, const std::string& elem) {
        return std::find(v.cbegin(), v.cend(), elem);
    };

    if (!allowMissingColumns) {
       for (const auto& iterate_state : m_state_names) {
           OPENSIM_THROW_IF(find(labels, iterate_state) == labels.end(),
                   Exception,
                   format("Expected table to contain column '%s'; consider "
                          "setting allowMissingColumns to true.",
                          iterate_state));
       }
    }

    std::vector<std::string> labelsToUse;
    for (const auto& label : labels) {
        if (std::find(m_state_names.begin(), m_state_names.end(), label) !=
                m_state_names.end()) {
            labelsToUse.push_back(label);
        } else {
            if (!allowExtraColumns) {
                OPENSIM_THROW(Exception,
                        format("Column '%s' is not a state in the "
                        "iterate; consider setting allowExtraColumns to "
                        "true.", label));
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

double MocoIterate::getInitialTime() const {
    ensureUnsealed();
    OPENSIM_THROW_IF(m_time.size() == 0, Exception, "Time vector is empty.");
    return m_time[0];
}

double MocoIterate::getFinalTime() const {
    ensureUnsealed();
    OPENSIM_THROW_IF(m_time.size() == 0, Exception, "Time vector is empty.");
    return m_time[m_time.size() - 1];
}

SimTK::VectorView MocoIterate::getState(const std::string& name) const {
    ensureUnsealed();
    auto it = std::find(m_state_names.cbegin(), m_state_names.cend(), name);
    OPENSIM_THROW_IF(it == m_state_names.cend(), Exception,
            format("Cannot find state named %s.", name));
    int index = (int)std::distance(m_state_names.cbegin(), it);
    return m_states.col(index);
}
SimTK::VectorView MocoIterate::getControl(const std::string& name) const {
    ensureUnsealed();
    auto it = std::find(m_control_names.cbegin(), m_control_names.cend(), name);
    OPENSIM_THROW_IF(it == m_control_names.cend(), Exception,
            format("Cannot find control named %s.", name));
    int index = (int)std::distance(m_control_names.cbegin(), it);
    return m_controls.col(index);
}
SimTK::VectorView MocoIterate::getMultiplier(const std::string& name) const {
    ensureUnsealed();
    auto it = std::find(m_multiplier_names.cbegin(), m_multiplier_names.cend(),
            name);
    OPENSIM_THROW_IF(it == m_multiplier_names.cend(), Exception, 
            format("Cannot find multiplier named %s.", name));
    int index = (int)std::distance(m_multiplier_names.cbegin(), it);
    return m_multipliers.col(index);
}
SimTK::VectorView MocoIterate::getSlack(const std::string& name) const {
    ensureUnsealed();
    auto it = std::find(m_slack_names.cbegin(), m_slack_names.cend(), name);
    OPENSIM_THROW_IF(it == m_slack_names.cend(), Exception,
        format("Cannot find slack named %s.", name));
    int index = (int)std::distance(m_slack_names.cbegin(), it);
    return m_slacks.col(index);
}
const SimTK::Real& MocoIterate::getParameter(const std::string& name) const {
    ensureUnsealed();
    auto it = std::find(
            m_parameter_names.cbegin(), m_parameter_names.cend(), name);
    OPENSIM_THROW_IF(it == m_parameter_names.cend(), Exception,
            format("Cannot find parameter named %s.", name));
    int index = (int)std::distance(m_parameter_names.cbegin(), it);
    return m_parameters.getElt(0, index);
}

double MocoIterate::resampleWithNumTimes(int numTimes) {
    ensureUnsealed();
    SimTK::Vector newTime = createVectorLinspace(
            numTimes, m_time[0], m_time[m_time.size() - 1]);
    resample(newTime);
    return newTime[1] - newTime[0];
}
double MocoIterate::resampleWithInterval(double desiredTimeInterval) {
    ensureUnsealed();
    // As a guide, solve for num_times in this equation, and convert that to
    // an integer:
    // time_interval = duration / (num_times - 1)
    const auto& duration = m_time[m_time.size() - 1] - m_time[0];
    const int actualNumTimes = (int)ceil(duration / desiredTimeInterval) + 1;
    resampleWithNumTimes(actualNumTimes);
    return duration / ((double)actualNumTimes - 1);
}
double MocoIterate::resampleWithFrequency(double desiredFrequency) {
    ensureUnsealed();
    // frequency = num_times / duration, so
    // num_times = ceil(duration * frequency);
    const auto& duration = m_time[m_time.size() - 1] - m_time[0];
    const int actualNumTimes = (int)ceil(duration * desiredFrequency);
    resampleWithNumTimes(actualNumTimes);
    return (double)actualNumTimes / duration;
}
void MocoIterate::resample(SimTK::Vector time) {
    ensureUnsealed();
    OPENSIM_THROW_IF(m_time.size() < 2, Exception,
            "Cannot resample if number of times is 0 or 1.");
    OPENSIM_THROW_IF(time[0] < m_time[0], Exception,
            format("New initial time (%f) cannot be less than existing initial "
                   "time (%f)",
                    time[0], m_time[0]));
    OPENSIM_THROW_IF(time[time.size() - 1] > m_time[m_time.size() - 1],
            Exception,
            format("New final time (%f) cannot be less than existing final "
                   "time (%f)",
                    time[time.size() - 1], m_time[m_time.size() - 1]));
    for (int itime = 1; itime < time.size(); ++itime) {
        OPENSIM_THROW_IF(time[itime] < time[itime - 1], Exception,
                format("New times must be non-decreasing, but "
                       "time[%i] < time[%i] (%f < %f).",
                        itime, itime - 1, time[itime], time[itime - 1]));
    }

    int numStates = (int)m_state_names.size();
    int numControls = (int)m_control_names.size();
    int numMultipliers = (int)m_multiplier_names.size();
    int numDerivatives = (int)m_derivative_names.size();
    int numSlacks = (int)m_slack_names.size();

    TimeSeriesTable table = convertToTable();
    GCVSplineSet splines(table, {}, std::min(m_time.size() - 1, 5));

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
            m_states.updCol(icol) = table.getDependentColumnAtIndex(icol)[0];
        for (int icontr = 0; icontr < numControls; ++icontr, ++icol)
            m_controls.updCol(icontr) =
                    table.getDependentColumnAtIndex(icol)[0];
        for (int imult = 0; imult < numMultipliers; ++imult, ++icol)
            m_multipliers.updCol(imult) =
                    table.getDependentColumnAtIndex(icol)[0];
        for (int ideriv = 0; ideriv < numDerivatives; ++ideriv, ++icol)
            m_derivatives.updCol(ideriv) =
                    table.getDependentColumnAtIndex(icol)[0];
        for (int islack = 0; islack < numDerivatives; ++islack, ++icol)
            m_slacks.updCol(islack) = table.getDependentColumnAtIndex(icol)[0];

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

MocoIterate::MocoIterate(const std::string& filepath) {
    FileAdapter::OutputTables tables = FileAdapter::readFile(filepath);

    // There should only be one table.
    OPENSIM_THROW_IF(tables.size() != 1, Exception,
            format("Expected MocoIterate file '%s' to contain 1 table, but it "
                   "contains %i tables.", filepath, tables.size()));

    // Get the first table.
    auto* table = dynamic_cast<TimeSeriesTable*>(tables.begin()->second.get());
    OPENSIM_THROW_IF(!table, Exception,
            "Expected MocoIterate file to contain a (scalar) "
            "TimeSeriesTable, but it contains a different type of table.");

    const auto& metadata = table->getTableMetaData();
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

    const auto& labels = table->getColumnLabels();
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
                             (int)table->getNumColumns(),
            Exception,
            format("Expected num_states + num_controls + num_multipliers + "
                   "num_derivatives + num_slacks + num_parameters = "
                   "number of columns, but "
                   "num_states=%i, num_controls=%i, "
                   "num_multipliers=%i, num_derivatives=%i, num_slacks=%i, "
                   "num_parameters=%i, number of columns=%i.",
                    numStates, numControls,
                    numMultipliers, numDerivatives, numSlacks,
                    numParameters, table->getNumColumns()));

    const auto& time = table->getIndependentColumn();
    m_time = SimTK::Vector((int)time.size(), time.data());

    if (numStates) {
        m_states = table->getMatrixBlock(0, 0, table->getNumRows(), numStates);
    }
    if (numControls) {
        m_controls = table->getMatrixBlock(
                0, numStates, table->getNumRows(), numControls);
    }
    if (numMultipliers) {
        m_multipliers = table->getMatrixBlock(0, numStates + numControls,
                table->getNumRows(), numMultipliers);
    }
    if (numDerivatives) {
        m_derivatives = table->getMatrixBlock(0,
                numStates + numControls + numMultipliers, table->getNumRows(),
                numDerivatives);
    }
    if (numSlacks) {
        m_slacks = table->getMatrixBlock(0,
                numStates + numControls + numMultipliers + numDerivatives,
                table->getNumRows(), numSlacks);
    }
    if (numParameters) {
        m_parameters = table->getMatrixBlock(0,
                                    numStates + numControls + numMultipliers +
                                            numDerivatives + numSlacks,
                                    1, numParameters)
                               .getAsRowVectorBase();
    }
}

void MocoIterate::write(const std::string& filepath) const {
    ensureUnsealed();
    TimeSeriesTable table0 = convertToTable();
    DataAdapter::InputTables tables = {{"table", &table0}};
    FileAdapter::writeFile(tables, filepath);
}

TimeSeriesTable MocoIterate::convertToTable() const {
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
        // TimeSeriesTable requires monotonically increasing time, but
        // this might not be true for iterates. Create the table with complying
        // times then hack in to set times back to what the iterate contains.
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
    return table;
}

Storage MocoIterate::exportToStatesStorage() const {
    ensureUnsealed();
    return convertTableToStorage(exportToStatesTable());
}

TimeSeriesTable MocoIterate::exportToStatesTable() const {
    ensureUnsealed();
    return {std::vector<double>(&m_time[0], &m_time[0] + m_time.size()),
            m_states, m_state_names};
}

StatesTrajectory MocoIterate::exportToStatesTrajectory(
        const MocoProblem& problem) const {
    ensureUnsealed();
    Storage storage = exportToStatesStorage();
    // TODO update when we support multiple phases.
    const auto& model = problem.getPhase(0).getModel();
    return StatesTrajectory::createFromStatesStorage(model, storage);
}

/*static*/ MocoIterate MocoIterate::createFromStatesControlsTables(
        const MocoProblemRep& /*problem*/,
        const TimeSeriesTable& statesTrajectory,
        const TimeSeriesTable& controlsTrajectory) {
    const int statesNumRows = (int)statesTrajectory.getNumRows();
    const int controlsNumRows = (int)controlsTrajectory.getNumRows();
    OPENSIM_THROW_IF(statesNumRows != controlsNumRows, Exception,
            format("Expected statesTrajectory (%i rows) and controlsTrajectory "
                   "(%i rows) to have the same number of rows.",
                   statesNumRows, controlsNumRows));
    // TODO interpolate instead of creating this error.
    for (int i = 0; i < statesNumRows; ++i) {
        const auto& statesTime = statesTrajectory.getIndependentColumn()[i];
        const auto& controlsTime = controlsTrajectory.getIndependentColumn()[i];
        OPENSIM_THROW_IF(statesTime != controlsTime, Exception,
                format("Expected time columns of statesTrajectory and "
                       "controlsTrajectory to match, but they differ at i = %i "
                       "(states time: %g; controls time: %g).",
                       i, statesTime, controlsTime));
    }

    // TODO Support controlsTrajectory being empty.

    const auto& statesTimes = statesTrajectory.getIndependentColumn();
    // The "true" means to not copy the data.
    SimTK::Vector time((int)statesTimes.size(), statesTimes.data(), true);

    // TODO MocoProblem should be able to produce a MocoIterate template; it's
    // what knows the state, control, and parameter names.
    return MocoIterate(time, statesTrajectory.getColumnLabels(),
            controlsTrajectory.getColumnLabels(), {}, // TODO (multiplier_names)
            {},                                       // TODO (parameter_names)
            statesTrajectory.getMatrix(), controlsTrajectory.getMatrix(),
            SimTK::Matrix(0, 0),  // TODO (multipliersTrajectory)
            SimTK::RowVector(0)); // TODO (parameters)
}

bool MocoIterate::isCompatible(
        const MocoProblemRep& mp, bool throwOnError) const {
    ensureUnsealed();
    // Slack variables might be solver dependent, so we can't check for
    // compatibility on the problem.

    auto mpsn = mp.createStateInfoNames();
    std::sort(mpsn.begin(), mpsn.end());
    auto mpcn = mp.createControlInfoNames();
    std::sort(mpcn.begin(), mpcn.end());
    auto mpmn = mp.createMultiplierInfoNames();
    std::sort(mpmn.begin(), mpmn.end());
    auto mppn = mp.createParameterNames();
    std::sort(mppn.begin(), mppn.end());

    auto sn(m_state_names);
    std::sort(sn.begin(), sn.end());

    auto cn(m_control_names);
    std::sort(cn.begin(), cn.end());

    auto mn(m_multiplier_names);
    std::sort(mn.begin(), mn.end());

    auto dn(m_derivative_names);
    std::sort(dn.begin(), dn.end());

    auto pn(m_parameter_names);
    std::sort(pn.begin(), pn.end());

    // Create the expected names for the derivative variable names.
    std::vector<std::string> mpdn;
    for (auto name : mpsn) {
        auto leafpos = name.find("value");
        if (leafpos != std::string::npos) {
            name.replace(leafpos, name.size(), "accel");
            mpdn.push_back(name);
        }
    }
    std::sort(mpdn.begin(), mpdn.end());

    bool compatible = mpsn == sn && mpcn == cn && mpmn == mn &&
                      // It's okay to not have any derivatives (for solving the
                      // problem with an explicit dynamics mode).
                      (dn.empty() || mpdn == dn) && mppn == pn;

    // TODO more detailed error message specifying exactly what's different.
    OPENSIM_THROW_IF(!compatible && throwOnError, Exception,
            "Iterate and provided problem are not compatible.");

    return compatible;
}

bool MocoIterate::isNumericallyEqual(
        const MocoIterate& other, double tol) const {
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

using VecStr = std::vector<std::string>;

// Check that two different vectors of strings have the same contents.
bool sameContents(VecStr v1, VecStr v2) {
    std::sort(v1.begin(), v1.end());
    std::sort(v2.begin(), v2.end());
    return v1 == v2;
}

// Check that two different vectors of string ("b", "c") contain the same subset
// vector of strings ("a").
void checkContains(std::string type, VecStr a, VecStr b, VecStr c) {
    // set_difference requires sorted containers.
    std::sort(a.begin(), a.end());
    std::sort(b.begin(), b.end());
    std::sort(c.begin(), c.end());
    std::vector<std::string> diff;
    std::set_difference(
            a.begin(), a.end(), b.begin(), b.end(), std::back_inserter(diff));
    if (!diff.empty()) {
        std::string msg = "Expected this iterate's " + type +
                          " names to "
                          "contain the following:";
        for (const auto& elem : diff) msg += "\n  " + elem;
        OPENSIM_THROW(Exception, msg);
    }
    diff.clear();
    std::set_difference(
            a.begin(), a.end(), c.begin(), c.end(), std::back_inserter(diff));
    if (!diff.empty()) {
        std::string msg = "Expected the other iterate's " + type +
                          " names to contain the following:";
        for (const auto& elem : diff) msg += "\n  " + elem;
        OPENSIM_THROW(Exception, msg);
    }
}

double MocoIterate::compareContinuousVariablesRMS(const MocoIterate& other,
        std::vector<std::string> stateNames,
        std::vector<std::string> controlNames,
        std::vector<std::string> multiplierNames,
        std::vector<std::string> derivativeNames) const {
    ensureUnsealed();

    // Process state, control, multiplier, and derivative names.
    // ---------------------------------------------------------
    if (stateNames.empty()) {
        OPENSIM_THROW_IF(!sameContents(m_state_names, other.m_state_names),
                Exception,
                "Expected both iterates to have the same state names; consider "
                "specifying the states to compare.");
        stateNames = m_state_names;
    } else if (stateNames.size() == 1 && stateNames[0] == "none") {
        stateNames.clear();
    } else {
        // Will hold elements of stateNames that are not in m_state_names, etc.
        checkContains("state", stateNames, m_state_names, other.m_state_names);
    }
    if (controlNames.empty()) {
        OPENSIM_THROW_IF(!sameContents(m_control_names, other.m_control_names),
                Exception,
                "Expected both iterates to have the same control names; "
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
                "Expected both iterates to have the same multiplier names; "
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
                "Expected both iterates to have the same derivative names; "
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

        GCVSplineSet self(selfTable, namesToUse);
        GCVSplineSet other(otherTable, namesToUse);

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

double MocoIterate::compareParametersRMS(const MocoIterate& other,
        std::vector<std::string> parameterNames) const {
    ensureUnsealed();

    // Process parameter names.
    // ------------------------
    if (parameterNames.empty()) {
        OPENSIM_THROW_IF(
                !sameContents(m_parameter_names, other.m_parameter_names),
                Exception,
                "Expected both iterates to have the same parameter names; "
                "consider "
                "specifying the parameters to compare.");
        parameterNames = m_parameter_names;
    } else {
        // Will hold elements of parameterNames that are not in
        // m_parameter_names, etc.
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

void MocoIterate::ensureUnsealed() const {
    OPENSIM_THROW_IF(m_sealed, MocoIterateIsSealed);
}
