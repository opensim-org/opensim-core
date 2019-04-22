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
    static Model createNLinkPendulum(int numLinks);
    static Model createPendulum() {
        return createNLinkPendulum(1);
    }
    static Model createDoublePendulum() {
        return createNLinkPendulum(2);
    }
/// Replace muscles in a model with a PathActuator of the same GeometryPath,
/// optimal force, and min/max control defaults.
/// @note This only replaces muscles within the model's ForceSet.
    static void replaceMusclesWithPathActuators(Model& model);

/// Remove muscles from the model.
/// @note This only removes muscles within the model's ForceSet.
    static void removeMuscles(Model& model);

/// Replace a joint in the model with a WeldJoint.
/// @note This assumes the joint is in the JointSet and that the joint's
///       connectees are PhysicalOffsetFrames.
    static void replaceJointWithWeldJoint(
            Model& model, const std::string& jointName);

};

} // namespace OpenSim

#endif // MOCO_MODELFACTORY_H
