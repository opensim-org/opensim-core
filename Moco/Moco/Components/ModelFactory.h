#ifndef MOCO_MODELFACTORY_H
#define MOCO_MODELFACTORY_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: ModelFactory.h                                               *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2018 Stanford University and the Authors                     *
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

#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim {

/// This class provides utilities for creating OpenSim models.
class OSIMMOCO_API ModelFactory {
public:
    /// @name Create a model
    /// @{

    /// Create a pendulum with the provided number of links.
    /// For each link, there is a body `/bodyset/b#` (where `#` is the link
    /// index starting at 0), a PinJoint `/jointset/j#` with coordinate
    /// `/jointset/j#/q#`, a CoordinateActuator `/tau#`, a Marker
    /// `/markerset/marker#` at the origin of the link's body, and a
    /// PhysicalOffsetFrame `/b#center` at the center of the link.
    static Model createNLinkPendulum(int numLinks);
    /// This is a convenience for `createNLinkPendulum(1)`.
    static Model createPendulum() { return createNLinkPendulum(1); }
    /// This is a convenience for `createNLinkPendulum(2)`.
    static Model createDoublePendulum() { return createNLinkPendulum(2); }

    /// @}

    /// @name Modify a Model
    /// @{

    /// Add CoordinateActuator%s for each unconstrained coordinate (e.g.,
    /// !Coordinate::isConstrained()) in the model, using the provided optimal
    /// force. Increasing the optimal force decreases the required control
    /// signal to generate a given actuation level. The actuators are added to
    /// the model's ForceSet and are named "reserve_<coordinate-path>" with
    /// forward slashes converted to underscores.
    static void createReserveActuators(Model& model, double optimalForce);

    /// @}
};

} // namespace OpenSim

#endif // MOCO_MODELFACTORY_H
