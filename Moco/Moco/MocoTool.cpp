/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoTool.cpp                                                 *
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
#include "MocoTool.h"

using namespace OpenSim;

void MocoTool::constructProperties() {
    constructProperty_initial_time();
    constructProperty_final_time();
    constructProperty_mesh_interval(0.02);
    constructProperty_model(ModelProcessor());
}

void MocoTool::updateTimeInfo(const std::string& dataLabel,
        const double& dataInitial, const double& dataFinal,
        TimeInfo& info) const {
    double initial;
    double final;
    if (!getProperty_initial_time().empty()) {
        OPENSIM_THROW_IF_FRMOBJ(get_initial_time() < dataInitial, Exception,
                format("Provided initial time of %g is less than what is "
                       "available from %s data, which starts at %g.",
                        get_initial_time(), dataLabel, dataInitial));
        initial = get_initial_time();
    } else {
        initial = std::max(info.initial, dataInitial);
    }
    if (!getProperty_final_time().empty()) {
        OPENSIM_THROW_IF_FRMOBJ(get_final_time() > dataFinal, Exception,
                format("Provided final time of %g is greater than what "
                       "is available from %s data, which ends at %g.",
                        get_final_time(), dataLabel, dataFinal));
        final = get_final_time();
    } else {
        final = std::min(final, dataFinal);
    }

    OPENSIM_THROW_IF_FRMOBJ(final < initial, Exception,
            format("Initial time of %g is greater than final time of %g.",
                    initial, final));

    info.initial = initial;
    info.final = final;

    // We do not want to end up with a larger mesh interval than requested.
    info.numMeshPoints =
            (int)std::ceil((info.final - info.initial) / get_mesh_interval()) +
            1;
}
