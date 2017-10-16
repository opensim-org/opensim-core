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
#include "MuscolloUtilities.h"

#include <OpenSim/Common/STOFileAdapter.h>
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
        m_controls(controlsTrajectory) {}

void MucoIterate::setTime(const SimTK::Vector& time) {
    OPENSIM_THROW_IF(time.size() != m_time.size(), Exception,
            "Expected " + std::to_string(m_time.size()) +
            " times but got " + std::to_string(time.size()) + ".");
    m_time = time;
}

void MucoIterate::setState(const std::string& name,
        const SimTK::Vector& trajectory) {
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

void MucoIterate::write(const std::string& filepath) const {
    std::vector<double> time(&m_time[0], &m_time[0] + m_time.size());

    // Concatenate the state and control names in a single vector.
    std::vector<std::string> labels = m_state_names;
    labels.insert(labels.end(),
            m_control_names.begin(), m_control_names.end());

    int numTimes = (int)m_time.size();
    int numStates = (int)m_state_names.size();
    int numControls = (int)m_control_names.size();
    SimTK::Matrix data(numTimes, (int)labels.size());
    for (int itime = 0; itime < numTimes; ++itime) {
        int icol;
        for (icol = 0; icol < numStates; ++icol) {
            data(itime, icol) = m_states(itime, icol);
        }
        for (int icontr = 0; icontr < numControls; ++icontr, ++icol) {
            data(itime, icol) = m_controls(itime, icontr);
        }
    }
    TimeSeriesTable table0(time, data, labels);
    OpenSim::DataAdapter::InputTables tables = {{"table", &table0}};
    FileAdapter::writeFile(tables, filepath);
}

Storage MucoIterate::exportToStatesStorage() const {
    TimeSeriesTable table(
    std::vector<double>(&m_time[0], &m_time[0] + m_time.size()),
    m_states, m_state_names);
    return convertTableToStorage(table);
}

StatesTrajectory MucoIterate::exportToStatesTrajectory(const Model& model)
        const {
    Storage storage = exportToStatesStorage();
    return StatesTrajectory::createFromStatesStorage(model, storage);
}
















