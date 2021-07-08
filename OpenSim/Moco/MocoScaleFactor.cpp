/* -------------------------------------------------------------------------- *
 * OpenSim: MocoScaleFactor.h                                                 *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2021 Stanford University and the Authors                     *
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

#include "MocoScaleFactor.h"

using namespace OpenSim;

MocoScaleFactor::MocoScaleFactor() {
    constructProperties();
}

MocoScaleFactor::MocoScaleFactor(const std::string& name,
        const MocoBounds& bounds) : MocoScaleFactor() {
    setName(name);
    set_bounds(bounds.getAsArray());
}

void MocoScaleFactor::constructProperties() {
    constructProperty_bounds();
    constructProperty_scale_factor(1.0);
}