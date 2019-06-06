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

MocoTool::TimeInfo MocoTool::calcInitialAndFinalTimes(
        const std::vector<double>& time0, const std::vector<double>& time1,
        const double& meshInterval) const {

    TimeInfo out;
    double initialTimeFromData = time0.front();
    double finalTimeFromData = time0.back();
    if (time1.size()) {
        initialTimeFromData = std::max(initialTimeFromData, time1.front());
        finalTimeFromData = std::min(finalTimeFromData, time1.back());
    }
    if (!getProperty_initial_time().empty()) {
        OPENSIM_THROW_IF_FRMOBJ(get_initial_time() < initialTimeFromData,
                Exception,
                format("Provided initial time of %g is less than what is "
                       "available from data, %g.",
                        get_initial_time(), initialTimeFromData));
        out.initialTime = get_initial_time();
    } else {
        out.initialTime = initialTimeFromData;
    }
    if (!getProperty_final_time().empty()) {
        OPENSIM_THROW_IF_FRMOBJ(get_final_time() > finalTimeFromData, Exception,
                format("Provided final time of %g is greater than what "
                       "is available from data, %g.",
                        get_final_time(), finalTimeFromData));
        out.finalTime = get_final_time();
    } else {
        out.finalTime = finalTimeFromData;
    }
    OPENSIM_THROW_IF_FRMOBJ(out.finalTime < out.initialTime, Exception,
            format("Initial time of %g is greater than final time of %g.",
                    out.initialTime, out.finalTime));

    // We do not want to end up with a lower mesh frequency than requested.
    out.numMeshPoints =
            (int)std::ceil((out.finalTime - out.initialTime) / (meshInterval)) +
                    1;
    return out;
}
