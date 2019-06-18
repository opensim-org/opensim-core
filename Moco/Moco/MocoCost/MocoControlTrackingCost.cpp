/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoControlTrackingCost.h                                    *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
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

#include "MocoControlTrackingCost.h"
#include <OpenSim/Simulation/Model/Model.h>
#include "../MocoUtilities.h"

using namespace OpenSim;

void MocoControlTrackingCost::initializeOnModelImpl(const Model& model) const {

    // TODO: set relativeToDirectory properly.
    TimeSeriesTable tableToUse = get_reference().process();

    // Check that there are no redundant columns in the reference data.
    checkRedundantLabels(tableToUse.getColumnLabels());

    // Convert data table to spline set.
    auto allSplines = GCVSplineSet(tableToUse);

    // Get a map between control names and their indices in the model. This also
    // checks that the model controls are in the correct order.
    auto allControlIndices = createSystemControlIndexMap(model);

    // Throw exception if a weight is specified for a nonexistent control.
    for (int i = 0; i < get_control_weights().getSize(); ++i) {
        const auto& weightName = get_control_weights().get(i).getName();
        if (allControlIndices.count(weightName) == 0) {
            OPENSIM_THROW_FRMOBJ(Exception,
                "Weight provided with name '" + weightName + "' but this is "
                "not a recognized control.");
        }
    }

    // Populate member variables needed to compute the cost. Unless the property
    // allow_unused_references is set to true, an exception is thrown for
    // names in the references that don't correspond to a control variable.
    for (int iref = 0; iref < allSplines.getSize(); ++iref) {
        const auto& refName = allSplines[iref].getName();
        if (allControlIndices.count(refName) == 0) {
            if (get_allow_unused_references()) {
                continue;
            }
            OPENSIM_THROW_FRMOBJ(Exception,
                "Control reference '" + refName + "' unrecognized.");
        }

        m_control_indices.push_back(allControlIndices[refName]);
        double refWeight = 1.0;
        if (get_control_weights().contains(refName)) {
            refWeight = get_control_weights().get(refName).getWeight();
        }
        m_control_weights.push_back(refWeight);
        m_ref_splines.cloneAndAppend(allSplines[iref]);
    }
}

void MocoControlTrackingCost::calcIntegralCostImpl(const SimTK::State& state,
    double& integrand) const {

    const auto& time = state.getTime();
    SimTK::Vector timeVec(1, time);
    const auto& controls = getModel().getControls(state);

    // TODO cache the reference coordinate values at the mesh points, 
    // rather than evaluating the spline.
    integrand = 0;
    for (int iref = 0; iref < m_ref_splines.getSize(); ++iref) {
        const auto& modelValue = controls[m_control_indices[iref]];
        const auto& refValue = m_ref_splines[iref].calcValue(timeVec);
        integrand += m_control_weights[iref] * pow(modelValue - refValue, 2);
    }
}
