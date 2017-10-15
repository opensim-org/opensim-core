#ifndef MUSCOLLO_MUCOITERATE_H
#define MUSCOLLO_MUCOITERATE_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoIterate.h                                           *
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

#include <SimTKcommon/internal/BigMatrix.h>

namespace OpenSim {

class MucoIterate {
public:
    MucoIterate(const SimTK::Vector& time,
            std::vector<std::string> state_names,
            std::vector<std::string> control_names,
            const SimTK::Matrix& statesTrajectory,
            const SimTK::Matrix& controlsTrajectory);

    MucoIterate* clone() const { return new MucoIterate(*this); }
    /// Resize the time vector and the time dimension of the states and
    /// controls trajectories.
    /// This may erase any data that was previously stored.
    // TODO change this to interpolate.
    void setNumTimes(int numTimes) {
        m_time.resize(numTimes);
        m_states.resize(numTimes, m_states.ncol());
        m_controls.resize(numTimes, m_controls.ncol());
    }
    void setTime(const SimTK::Vector& time);
    void setState(const std::string& name, const SimTK::Vector& trajectory);
    void setControl(const std::string& name, const SimTK::Vector& trajectory);

    /// These variants support use of an initializer list. Example:
    /// @code{.cpp}
    /// iterate.setTime({0, 0.5, 1.0});
    /// @endcode
    void setTime_std(const std::vector<double>& time) {
        setTime(SimTK::Vector((int)time.size(), time.data()));
    }
    /// This variant supports use of an initializer list.
    void setState_std(const std::string& name,
            const std::vector<double>& trajectory) {
        setState(name,
                SimTK::Vector((int)trajectory.size(), trajectory.data()));
    }
    /// This variant supports use of an initializer list.
    void setControl_std(const std::string& name,
            const std::vector<double>& trajectory) {
        setControl(name,
                SimTK::Vector((int)trajectory.size(), trajectory.data()));
    }

    const SimTK::Vector& getTime() const
    {   return m_time; }
    // TODO inconsistent plural "state names" vs "states trajectory"
    const std::vector<std::string>& getStateNames() const
    {   return m_state_names; }
    const std::vector<std::string>& getControlNames() const
    {   return m_control_names; }
    const SimTK::Matrix& getStatesTrajectory() const
    {   return m_states; }
    const SimTK::Matrix& getControlsTrajectory() const
    {   return m_controls; }

    void write(const std::string& filepath) const;
private:
    SimTK::Vector m_time;
    std::vector<std::string> m_state_names;
    std::vector<std::string> m_control_names;
    SimTK::Matrix m_states;
    SimTK::Matrix m_controls;
};

class MucoSolution : public MucoIterate {
    using MucoIterate::MucoIterate;
};

} // namespace OpenSim

#endif // MUSCOLLO_MUCOITERATE_H
