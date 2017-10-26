#ifndef SIMBODY_UTILS_H_
#define SIMBODY_UTILS_H_
/* -------------------------------------------------------------------------- *
 *                         SimbodyUtils.h                                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Jack Middleton, Ayman Habib                                     *
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

#include "Simbody.h"

namespace OpenSim {

class Body;
class Model;
using SimTK::Vector_;
using SimTK::State;
using SimTK::SpatialVec;

//==============================================================================
//                                 SimbodyUtils
//==============================================================================
/** Class intended to serve as a facade to simbody functionality that is not exposed 
through the normal SWIG binding process, primarily because the overhead and clutter 
resulting from wrapping these classes outweigh the benefits of exposing methods

Most methods of this class are static and implementated by delegating the call 
to the appropriate SimTK/Simbody classes

@author Ayman Habib
**/

class SimbodyUtils {
public:
    const SimTK::Vector_<SimTK::SpatialVec>&  getGravityForce_getBodyForces(const Model& model, const SimTK::State& state) const {
        return model.getGravityForce().getBodyForces(state);
    };
};

} // namespace OpenSim

#endif // SIMBODY_UTILS_H_

