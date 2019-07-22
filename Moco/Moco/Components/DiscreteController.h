#ifndef MOCO_DISCRETECONTROLLER_H
#define MOCO_DISCRETECONTROLLER_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: DiscreteController.h                                         *
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

#include "../osimMocoDLL.h"

#include <OpenSim/Simulation/Control/Controller.h>

namespace OpenSim {

// TODO does not connect to actuators? no need? do any other functions in Controller depend on
// having actuators?
class DiscreteController : public Controller {
    OpenSim_DECLARE_CONCRETE_OBJECT(DiscreteController, Controller);
public:
    DiscreteController() = default;
    void setDiscreteControls(SimTK::State& s,
            const SimTK::Vector& controls) const;
    SimTK::Vector& updDiscreteControls(SimTK::State& s) const;
    void computeControls(
            const SimTK::State& s, SimTK::Vector& controls) const override;
protected:
    void extendRealizeTopology(SimTK::State&) const override;
    mutable SimTK::DiscreteVariableIndex m_discreteVarIndex;

};

} // namespace OpenSim


#endif // MOCO_DISCRETECONTROLLER_H
