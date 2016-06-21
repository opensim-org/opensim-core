/* -------------------------------------------------------------------------- *
 *                        OpenSim:  helperMethods.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
 * Author(s): Chris Dembia, Shrinidhi K. Lakshmikanth, Ajay Seth,             *
 *            Thomas Uchida                                                   *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/* Helper methods to take care of some mundane tasks. All methods in this file
have already been implemented. */

#include <OpenSim/OpenSim.h>

namespace OpenSim {

// Configures a PathActuator so that it wraps over a WrapObject attached to the
// specified Body. This footwork is necessary because of a bug in GeometryPath.
void addPathWrapHelper(ModelComponent& model,
    const std::string& pathActuatorName, const std::string& wrapObjectName,
    const std::string& bodyName)
{
    auto& pathActuator = model.updComponent<PathActuator>(pathActuatorName);
    auto& body         = model.updComponent<Body>(bodyName);
    auto& wrapObject   = body.upd_WrapObjectSet().get(wrapObjectName);
    pathActuator.connect(model);
    body.connect(model);
    pathActuator.updGeometryPath().addPathWrap(wrapObject);
}

} // end of namespace OpenSim
