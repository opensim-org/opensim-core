#ifndef OPENSIM_DISCRETECONTROLLER_H
#define OPENSIM_DISCRETECONTROLLER_H
/* -------------------------------------------------------------------------- *
 * OpenSim: DiscreteController.h                                              *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2020 Stanford University and the Authors                     *
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

#include <OpenSim/Moco/osimMocoDLL.h>

#include <OpenSim/Simulation/Control/Controller.h>

namespace OpenSim {

/** This component is used internally by Moco for passing a solver's control
variables to a Model. */
class DiscreteController : public Controller {
    OpenSim_DECLARE_CONCRETE_OBJECT(DiscreteController, Controller);
public:
    DiscreteController() = default;
    void setDiscreteControls(SimTK::State& s,
            const SimTK::Vector& controls) const;
    SimTK::Vector& updDiscreteControls(SimTK::State& s) const;
    const SimTK::Vector& getDiscreteControls(const SimTK::State& s) const;
    void computeControls(
            const SimTK::State& s, SimTK::Vector& controls) const override;
protected:
    void extendRealizeTopology(SimTK::State&) const override;
    mutable SimTK::DiscreteVariableIndex m_discreteVarIndex;

};

} // namespace OpenSim


#endif // OPENSIM_DISCRETECONTROLLER_H
