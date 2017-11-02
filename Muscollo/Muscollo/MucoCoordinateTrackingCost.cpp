/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoCoordinateTrackingCost.cpp                           *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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

#include "MucoCoordinateTrackingCost.h"
#include "MuscolloUtilities.h"
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

void MucoCoordinateTrackingCost::initializeImpl() const {
    m_refsplines = GCVSplineSet(m_table);
    auto allSysYIndices = createSystemYIndexMap(getModel());
    m_sysYIndices.resize(m_refsplines.getSize());
    for (int iref = 0; iref < m_refsplines.getSize(); ++iref) {
        const auto& refName = m_refsplines[iref].getName();
        OPENSIM_THROW_IF(allSysYIndices.count(refName) == 0, Exception,
                "State '" + refName + "' unrecognized.");
        m_sysYIndices[iref] = allSysYIndices[refName];
    }
}

void MucoCoordinateTrackingCost::calcIntegralCostImpl(/*int meshIndex,*/
        const SimTK::State& state, double& integrand) const {
    const auto& time = state.getTime();

    SimTK::Vector timeVec(1, time);

    // TODO cache the reference coordinate values at the mesh points, rather
    // than evaluating the spline.
    for (int iref = 0; iref < m_refsplines.getSize(); ++iref) {
        const auto& modelValue = state.getY()[m_sysYIndices[iref]];
        const auto& refValue = m_refsplines[iref].calcValue(timeVec);
        integrand += pow(modelValue - refValue, 2);
    }
}
