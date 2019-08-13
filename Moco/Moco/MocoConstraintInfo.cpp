/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoConstraintInfo.h                                         *
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

#include "MocoConstraintInfo.h"

using namespace OpenSim;

MocoConstraintInfo::MocoConstraintInfo() {
    constructProperties();
    if (getName().empty()) setName("constraint");
}

std::vector<std::string> MocoConstraintInfo::getConstraintLabels() const {
    std::vector<std::string> labels(getNumEquations());
    for (int i = 0; i < getNumEquations(); ++i) {
        if (getProperty_suffixes().empty()) {
            labels[i] = getName() + "_" + std::to_string(i);
        }
        else {
            labels[i] = getName() + "_" + get_suffixes(i);
        }
    }
    return labels;
}

void MocoConstraintInfo::printDescription(std::ostream& stream) const {
    stream << getName() << ". " << getConcreteClassName() <<
            ". number of scalar equations: " << getNumEquations();

    const std::vector<MocoBounds> bounds = getBounds();
    stream << ". bounds: ";
    for (int i = 0; i < (int)bounds.size(); ++i) {
        bounds[i].printDescription(stream);
        if (i < (int)bounds.size() - 1)
            stream << ", ";
    }
    stream << std::endl;
}

void MocoConstraintInfo::constructProperties() {
    constructProperty_bounds();
    constructProperty_suffixes();
}
