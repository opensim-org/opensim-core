#ifndef MOCO_ACCELERATIONMOTION_H
#define MOCO_ACCELERATIONMOTION_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: AccelerationMotion.h                                         *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
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

#include <OpenSim/Simulation/Model/ModelComponent.h>

#include <simbody/internal/Motion.h>

namespace OpenSim {

class AccelerationMotion : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(AccelerationMotion, ModelComponent);
public:
    AccelerationMotion(std::string name) { setName(std::move(name)); }
    void setUDot(SimTK::State& state, const SimTK::Vector& fullUDot) const;
    const SimTK::Vector& getUDot(const SimTK::State& state,
            SimTK::MobilizedBodyIndex mobodIdx) const;
    void setEnabled(SimTK::State& state, bool enabled) const;
protected:
private:
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void extendRealizeTopology(SimTK::State& state) const override;
    mutable SimTK::ResetOnCopy<std::vector<SimTK::DiscreteVariableIndex>>
            m_dvIndices;
    mutable SimTK::ResetOnCopy<std::vector<SimTK::Motion>> m_motions;
};

} // namespace OpenSim

#endif // MOCO_ACCELERATIONMOTION_H
