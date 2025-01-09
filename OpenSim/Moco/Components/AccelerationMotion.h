#ifndef OPENSIM_ACCELERATIONMOTION_H
#define OPENSIM_ACCELERATIONMOTION_H
/* -------------------------------------------------------------------------- *
 * OpenSim: AccelerationMotion.h                                              *
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

#include <OpenSim/Moco/osimMocoDLL.h>

namespace OpenSim {

/** This class is a thin wrapper to Simbody's SimTK::Motion for prescribing
the acceleration of all degrees of freedom (UDot), and is used when
enforcing dynamics using implicit differential equations (UDot is supplied
by the solver, not by Simbody). This component adds discrete variables
for holding onto the user-supplied UDot and passing it onto the
SimTK::Motion. Then, SimbodyMatterSubsystem::findMotionForces() provides the
"implicit" differential equation residual (akin to
SimbodyMatterSubsystem::calcResidualForce()).
By default, the prescribed motions are disabled; see
setEnabled().
This prescribed motion does *not* add constraints to the system; rather,
this class removes degrees of freedom.
This class is not intended for use outside of Moco.
The wrapper to OpenSim is necessary so that the discrete variables appear in
the State whenever initSystem() is called on a model. */
class OSIMMOCO_API AccelerationMotion : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(AccelerationMotion, ModelComponent);
public:
    AccelerationMotion() = default;
    AccelerationMotion(std::string name) { setName(std::move(name)); }
    /// Set the UDot vector. The vector must have size SimTK::State::getNU().
    void setUDot(SimTK::State& state, const SimTK::Vector& fullUDot) const;
    /// Get the subset of UDots for the requested MobilizedBody.
    const SimTK::Vector& getUDot(const SimTK::State& state,
            SimTK::MobilizedBodyIndex mobodIdx) const;
    /// Use this to set whether the prescribed acceleration motion is used or
    /// not.
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

#endif // OPENSIM_ACCELERATIONMOTION_H
