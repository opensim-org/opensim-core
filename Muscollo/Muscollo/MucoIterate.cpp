/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoIterate.cpp                                          *
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
#include "MucoIterate.h"
#include "MucoProblem.h"
#include "MuscolloUtilities.h"

#include <OpenSim/Common/FileAdapter.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

MucoIterate::MucoIterate(const SimTK::Vector& time,
        std::vector<std::string> state_names,
        std::vector<std::string> control_names,
        const SimTK::Matrix& statesTrajectory,
        const SimTK::Matrix& controlsTrajectory) :
        m_time(time), m_state_names(std::move(state_names)),
        m_control_names(std::move(control_names)),
        m_states(statesTrajectory),
        m_controls(controlsTrajectory) {
    OPENSIM_THROW_IF((int)m_state_names.size() != m_states.ncol(),
            Exception, "Inconsistent number of states.");
    OPENSIM_THROW_IF((int)m_control_names.size() != m_controls.ncol(),
            Exception, "Inconsistent number of controls.");
    OPENSIM_THROW_IF(time.size() != m_states.nrow() ||
            time.size() != m_controls.nrow(), Exception,
            "Inconsistent number of times.");
}

void MucoIterate::setTime(const SimTK::Vector& time) {
    ensureUnsealed();
    OPENSIM_THROW_IF(time.size() != m_time.size(), Exception,
            "Expected " + std::to_string(m_time.size()) +
            " times but got " + std::to_string(time.size()) + ".");
    m_time = time;
}

void MucoIterate::setState(const std::string& name,
        const SimTK::Vector& trajectory) {
    ensureUnsealed();
    OPENSIM_THROW_IF(trajectory.size() != m_states.nrow(), Exception,
            "For state " + name + ", expected " +
            std::to_string(m_states.nrow()) +
            " elements but got " + std::to_string(trajectory.size()) + ".");

    auto it = std::find(m_state_names.cbegin(), m_state_names.cend(), name);
    OPENSIM_THROW_IF(it == m_state_names.cend(), Exception,
            "Cannot find state named " + name + ".");
    int index = (int)std::distance(m_state_names.cbegin(), it);
    m_states.updCol(index) = trajectory;
}

void MucoIterate::setControl(const std::string& name,
        const SimTK::Vector& trajectory) {
    ensureUnsealed();
    OPENSIM_THROW_IF(trajectory.size() != m_controls.nrow(), Exception,
            "For control " + name + ", expected " +
            std::to_string(m_controls.nrow()) +
            " elements but got " + std::to_string(trajectory.size()) + ".");

    auto it = std::find(m_control_names.cbegin(), m_control_names.cend(),
            name);
    OPENSIM_THROW_IF(it == m_control_names.cend(), Exception,
            "Cannot find control named " + name + ".");
    int index = (int)std::distance(m_control_names.cbegin(), it);
    m_controls.updCol(index) = trajectory;
}

void MucoIterate::setStatesTrajectory(const TimeSeriesTable& states,
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
                   Exception, "Expected table to contain column '" +
                   iterate_state + "'; consider setting "
                   "allowMissingColumns to true.");
       }
    }

    std::vector<std::string> labelsToUse;
    if (!allowExtraColumns) {
        for (const auto& label : labels) {
            if (find(m_state_names, label) != labels.end()) {
                labelsToUse.push_back(label);
            } else {
                OPENSIM_THROW(Exception,
                        "Column '" + label + "' is not a state in the "
                        "iterate; consider setting allowExtraColumns to true.");
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


SimTK::VectorView MucoIterate::getState(const std::string& name) const {
    ensureUnsealed();
    auto it = std::find(m_state_names.cbegin(), m_state_names.cend(), name);
    OPENSIM_THROW_IF(it == m_state_names.cend(), Exception,
            "Cannot find state named " + name + ".");
    int index = (int)std::distance(m_state_names.cbegin(), it);
    return m_states.col(index);
}
SimTK::VectorView MucoIterate::getControl(const std::string& name) const {
    ensureUnsealed();
    auto it = std::find(m_control_names.cbegin(), m_control_names.cend(),
            name);
    OPENSIM_THROW_IF(it == m_control_names.cend(), Exception,
            "Cannot find control named " + name + ".");
    int index = (int)std::distance(m_control_names.cbegin(), it);
    return m_controls.col(index);
}

double MucoIterate::resampleWithNumTimes(int numTimes) {
    ensureUnsealed();
    int numStates = (int)m_state_names.size();
    int numControls = (int)m_control_names.size();
    TimeSeriesTable table = convertToTable();
    OPENSIM_THROW_IF(m_time.size() < 2, Exception,
            "Cannot resample if number of times is 0 or 1.");
    GCVSplineSet splines(table, {}, std::min(m_time.size() - 1, 5));
    m_time = createVectorLinspace(numTimes, m_time[0], m_time[m_time.size()-1]);
    m_states.resize(numTimes, numStates);
    m_controls.resize(numTimes, numControls);
    SimTK::Vector time(1);
    for (int itime = 0; itime < m_time.size(); ++itime) {
        time[0] = m_time[itime];
        int icol;
        for (icol = 0; icol < numStates; ++icol)
            m_states(itime, icol) = splines[icol].calcValue(time);
        for (int icontr = 0; icontr < numControls; ++icontr, ++icol)
            m_controls(itime, icontr) = splines[icol].calcValue(time);
    }
    return m_time[1] - m_time[0];
}
double MucoIterate::resampleWithInterval(double desiredTimeInterval) {
    ensureUnsealed();
    // As a guide, solve for num_times in this equation, and convert that to
    // an integer:
    // time_interval = duration / (num_times - 1)
    const auto& duration = m_time[m_time.size() - 1] - m_time[0];
    const int actualNumTimes = (int)ceil(duration / desiredTimeInterval) + 1;
    resampleWithNumTimes(actualNumTimes);
    return duration / ((double)actualNumTimes - 1);
}
double MucoIterate::resampleWithFrequency(double desiredFrequency) {
    ensureUnsealed();
    // frequency = num_times / duration, so
    // num_times = ceil(duration * frequency);
    const auto& duration = m_time[m_time.size() - 1] - m_time[0];
    const int actualNumTimes = (int)ceil(duration * desiredFrequency);
    resampleWithNumTimes(actualNumTimes);
    return (double)actualNumTimes / duration;
}

MucoIterate::MucoIterate(const std::string& filepath) {
    FileAdapter::OutputTables tables = FileAdapter::readFile(filepath);

    // There should only be one table.
    OPENSIM_THROW_IF(tables.size() != 1, Exception,
            "Expected MucoIterate file '" + filepath +
            "' to contain 1 table, but it contains " +
            std::to_string(tables.size()) + " tables.");

    // Get the first table.
    auto* table = dynamic_cast<TimeSeriesTable*>(tables.begin()->second.get());
    OPENSIM_THROW_IF(!table, Exception,
            "Expected MucoIterate file to contain a (scalar) "
            "TimeSeriesTable, but it contains a different type of table.");

    const auto& metadata = table->getTableMetaData();
    // TODO: bug with file adapters.
    //auto numStates = metadata.getValueForKey("num_states").getValue<int>();
    //auto numControls = metadata.getValueForKey("num_controls").getValue<int>();
    int numStates;
    SimTK::convertStringTo(
            metadata.getValueForKey("num_states").getValue<std::string>(),
            numStates);
    int numControls;
    SimTK::convertStringTo(
            metadata.getValueForKey("num_controls").getValue<std::string>(),
            numControls);
    OPENSIM_THROW_IF(numStates < 0, Exception, "Invalid num_states.");
    OPENSIM_THROW_IF(numControls < 0, Exception, "Invalid num_controls.");

    const auto& labels = table->getColumnLabels();
    m_state_names.insert(m_state_names.end(),
            labels.begin(), labels.begin() + numStates);
    m_control_names.insert(m_control_names.end(),
            labels.begin() + numStates, labels.end());

    OPENSIM_THROW_IF(numStates + numControls != (int)table->getNumColumns(),
            Exception,
            "Expected num_states + num_controls = number of columns, but "
            "num_states=" + std::to_string(numStates) + ", "
            "num_controls=" + std::to_string(numControls) + ", "
            "number of columns=" + std::to_string(table->getNumColumns()));

    const auto& time = table->getIndependentColumn();
    m_time = SimTK::Vector((int)time.size(), time.data());

    m_states = table->getMatrixBlock(0, 0, table->getNumRows(), numStates);
    m_controls = table->getMatrixBlock(0, numStates,
            table->getNumRows(), numControls);
}

void MucoIterate::write(const std::string& filepath) const {
    ensureUnsealed();
    TimeSeriesTable table0 = convertToTable();
    DataAdapter::InputTables tables = {{"table", &table0}};
    FileAdapter::writeFile(tables, filepath);
}

TimeSeriesTable MucoIterate::convertToTable() const {
    ensureUnsealed();
    std::vector<double> time(&m_time[0], &m_time[0] + m_time.size());

    // Concatenate the state and control names in a single vector.
    std::vector<std::string> labels = m_state_names;
    labels.insert(labels.end(),
            m_control_names.begin(), m_control_names.end());
    int numTimes = (int)m_time.size();
    int numStates = (int)m_state_names.size();
    int numControls = (int)m_control_names.size();

    SimTK::Matrix data(numTimes, (int)labels.size());
    data.updBlock(0, 0, numTimes, numStates) = m_states;
    data.updBlock(0, numStates, numTimes, numControls) = m_controls;
    TimeSeriesTable table(time, data, labels);

    //table.updTableMetaData().setValueForKey("num_states", numStates);
    //table.updTableMetaData().setValueForKey("num_controls", numControls);
    table.updTableMetaData().setValueForKey("num_states",
            std::to_string(numStates));
    table.updTableMetaData().setValueForKey("num_controls",
            std::to_string(numControls));
    return table;
}

Storage MucoIterate::exportToStatesStorage() const {
    ensureUnsealed();
    TimeSeriesTable table(
        std::vector<double>(&m_time[0], &m_time[0] + m_time.size()),
        m_states, m_state_names);
    return convertTableToStorage(table);
}

StatesTrajectory MucoIterate::exportToStatesTrajectory(
        const MucoProblem& problem) const {
    ensureUnsealed();
    Storage storage = exportToStatesStorage();
    // TODO update when we support multiple phases.
    const auto& model = problem.getPhase(0).getModel();
    return StatesTrajectory::createFromStatesStorage(model, storage);
}

bool MucoIterate::isCompatible(const MucoProblem& mp, bool throwOnError) const {
    ensureUnsealed();

    auto mpsn = mp.getPhase().createStateInfoNames();
    std::sort(mpsn.begin(), mpsn.end());
    auto mpcn = mp.getPhase().createControlInfoNames();
    std::sort(mpcn.begin(), mpcn.end());

    auto sn(m_state_names);
    std::sort(sn.begin(), sn.end());

    auto cn(m_control_names);
    std::sort(cn.begin(), cn.end());

    bool compatible = mpsn == sn && mpcn == cn;

    // TODO more detailed error message specifying exactly what's different.
    OPENSIM_THROW_IF(!compatible && throwOnError, Exception,
            "Iterate and provided problem are not compatible.");

    return compatible;
}

bool MucoIterate::isNumericallyEqual(const MucoIterate& other) const {
    ensureUnsealed();
    return m_state_names == other.m_state_names &&
            m_control_names == other.m_control_names &&
            SimTK::Test::numericallyEqual(m_time, other.m_time, 1) &&
            SimTK::Test::numericallyEqual(m_states, other.m_states, 1) &&
            SimTK::Test::numericallyEqual(m_controls, other.m_controls, 1);
}

double MucoIterate::compareRMS(const MucoIterate& other,
        std::vector<std::string> stateNames,
        std::vector<std::string> controlNames) const {
    ensureUnsealed();

    // Process state and control names.
    // --------------------------------
    using VecStr = std::vector<std::string>;
    auto sameContents = [](VecStr v1, VecStr v2) {
        std::sort(v1.begin(), v1.end());
        std::sort(v2.begin(), v2.end());
        return v1 == v2;
    };
    // Check that b and c contain a.
    auto checkContains = [](std::string type, VecStr a, VecStr b, VecStr c) {
        // set_difference requires sorted containers.
        std::sort(a.begin(), a.end());
        std::sort(b.begin(), b.end());
        std::sort(c.begin(), c.end());
        std::vector<std::string> diff;
        std::set_difference(a.begin(), a.end(), b.begin(), b.end(),
                std::back_inserter(diff));
        if (!diff.empty()) {
            std::string msg = "Expected this iterate's " + type + " names to "
                    "contain the following:";
            for (const auto& elem : diff) msg += "\n  " + elem;
            OPENSIM_THROW(Exception, msg);
        }
        diff.clear();
        std::set_difference(a.begin(), a.end(), c.begin(), c.end(),
                std::back_inserter(diff));
        if (!diff.empty()) {
            std::string msg = "Expected the other iterate's " + type +
                    " names to contain the following:";
            for (const auto& elem : diff) msg += "\n  " + elem;
            OPENSIM_THROW(Exception, msg);
        }
    };
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


    std::vector<double> selfTime =
            std::vector<double>(&m_time[0], &m_time[0] + m_time.size());
    std::vector<double> otherTime = std::vector<double>(&other.m_time[0],
            &other.m_time[0] + other.m_time.size());

    const auto initialTime = std::min(selfTime.front(), otherTime.front());
    const auto finalTime = std::max(selfTime.back(), otherTime.back());
    const auto numTimes = std::max(getNumTimes(), other.getNumTimes());
    // Times to use for integrating over time.
    auto integTime = createVectorLinspace(numTimes, initialTime, finalTime);
    const auto timeInterval = integTime[1] - integTime[0];

    auto integralSumSquaredError = [&selfTime, &otherTime,
            &numTimes, &integTime, &timeInterval](
            const VecStr& namesToUse,
            const SimTK::Matrix& selfData, const VecStr& selfNames,
            const SimTK::Matrix& otherData, const VecStr& otherNames) -> double
    {
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
                double selfValue = selfInRange ?
                                   self.get(iname).calcValue(curTimeVec) : 0;
                double otherValue = otherInRange ?
                                    other.get(iname).calcValue(curTimeVec) : 0;
                sumSquaredError[itime] += SimTK::square(selfValue - otherValue);
            }
        }
        // Trapezoidal rule for uniform grid:
        // dt / 2 (f_0 + 2f_1 + 2f_2 + 2f_3 + ... + 2f_{N-1} + f_N)
        assert(numTimes > 2);
        return timeInterval / 2.0 * (sumSquaredError.sum() +
                sumSquaredError(1, numTimes - 2).sum());
    };

    const auto stateISS = integralSumSquaredError(stateNames,
            m_states, m_state_names, other.m_states, other.m_state_names);
    const auto controlISS = integralSumSquaredError(controlNames, m_controls,
            m_control_names, other.m_controls, other.m_control_names);

    // sqrt(1/T * integral_t (sum_is error_is^2 + sum_ic error_ic^2)
    // `is`: index for states; `ic`: index for controls.
    return sqrt((stateISS + controlISS) / (finalTime - initialTime));
}

void MucoIterate::ensureUnsealed() const {
    OPENSIM_THROW_IF(m_sealed, Exception,
            "This iterate is sealed, to force you to acknowledge the "
            "solver failed; call unseal() to gain access.");
}













