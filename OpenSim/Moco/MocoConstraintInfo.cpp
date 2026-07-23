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

void MocoConstraintInfo::printDescription() const {
    log_info("  {}. {}. number of scalar equations: {}. bounds: {}",
        getName(),
        getConcreteClassName(),
        getNumEquations(),
        detail::join(getBounds(), ", ")
    );
}

void MocoConstraintInfo::constructProperties() {
    constructProperty_bounds();
    constructProperty_suffixes();
}
