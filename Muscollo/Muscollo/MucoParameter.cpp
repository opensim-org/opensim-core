/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoParameter.cpp                                          *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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
#include "MucoParameter.h"

using namespace OpenSim;

MucoParameter::MucoParameter() {
    constructProperties();
    if (getName().empty()) setName("parameter");
}

MucoParameter::MucoParameter(const std::string& name,
    const std::string& modelProperty,
    const std::string& modelComponent,
    const MucoBounds& bounds) : MucoParameter() {
    setName(name);
    set_bounds(bounds.getAsArray());
    set_model_property(modelProperty);
    set_model_component(modelComponent);
}

void MucoParameter::constructProperties() {
    constructProperty_bounds();
    constructProperty_model_property("");
    constructProperty_model_component("");
}