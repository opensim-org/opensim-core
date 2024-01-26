#ifndef OPENSIM_CONTROLALLOCATOR_H
#define OPENSIM_CONTROLALLOCATOR_H
/* -------------------------------------------------------------------------- *
 * OpenSim: ControlAllocator.h                                                *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2024 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
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
#include <OpenSim/Moco/osimMocoDLL.h>

namespace OpenSim {

class OSIMMOCO_API ControlAllocator : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(ControlAllocator, ModelComponent);

public:
//=============================================================================
// OUTPUTS
//=============================================================================
    OpenSim_DECLARE_LIST_OUTPUT(controls, double, getControlForOutputChannel,
            SimTK::Stage::Dynamics);

//=============================================================================
// METHODS
//=============================================================================

    // CONSTRUCTION AND DESTRUCTION
    ControlAllocator();
    ~ControlAllocator() noexcept override;

    ControlAllocator(const ControlAllocator&);
    ControlAllocator(ControlAllocator&&);

    ControlAllocator& operator=(const ControlAllocator&);
    ControlAllocator& operator=(ControlAllocator&&);

    // GET AND SET
    /**
     * Add a control to the control allocator.
     */
    void addControl(const std::string& controlName);

    /**
     * Set the allocator controls stored in the discrete variable.
     *
     * The `controls` vector must be the same size as the number of controls
     * added to the allocator by `addControl()`.
     */
    void setControls(SimTK::State& s, const SimTK::Vector& controls) const;

    /**
     * Get a writable reference to the allocator controls stored in the discrete
     * variable.
     */
    SimTK::Vector& updControls(SimTK::State& s) const;

    /**
     * Get a const reference to the allocator controls stored in the discrete
     * variable.
     */
    const SimTK::Vector& getControls(const SimTK::State& s) const;

    /**
     * Get the control value for the requested output channel.
     */
    double getControlForOutputChannel(
            const SimTK::State& s, const std::string& channel) const;

    /**
     * Get the names of the controls in the order they were added to the
     * allocator.
     */
    std::vector<std::string> getControlNamesInOrder() const;

protected:
    // MODEL COMPONENT INTERFACE
    void extendRealizeTopology(SimTK::State& state) const override;
    void extendFinalizeFromProperties() override;

private:
    std::unordered_map<std::string, int> m_controlIndexMap;
    mutable SimTK::DiscreteVariableIndex m_discreteVarIndex;
};

} // namespace OpenSim

#endif // OPENSIM_CONTROLALLOCATOR_H
