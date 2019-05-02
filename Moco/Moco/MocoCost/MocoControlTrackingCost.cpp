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

    TimeSeriesTable tableToUse;

    if (get_reference_file() != "") {
        // Should not be able to supply both.
        assert(m_table.getNumColumns() == 0);

        auto tablesFromFile = FileAdapter::readFile(get_reference_file());
        // There should only be one table.
        OPENSIM_THROW_IF_FRMOBJ(tablesFromFile.size() != 1, Exception,
            format("Expected reference file '%s' to contain 1 table, but "
                "it contains %i tables.",
                get_reference_file(), tablesFromFile.size()));
        // Get the first table.
        auto* firstTable =
            dynamic_cast<TimeSeriesTable*>(
                tablesFromFile.begin()->second.get());
        OPENSIM_THROW_IF_FRMOBJ(!firstTable, Exception,
            "Expected reference file to contain a (scalar) "
            "TimeSeriesTable, but it contains a different type of table.");
        tableToUse = *firstTable;
    }
    else if (m_table.getNumColumns() != 0) {
        tableToUse = m_table;
    }
    else {
        OPENSIM_THROW_FRMOBJ(Exception,
            "Expected user to either provide a reference"
            " file or to programmatically provide a reference table, but "
            " the user supplied neither.");
    }

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
        if (!get_allow_unused_references()) {
            OPENSIM_THROW_IF_FRMOBJ(allControlIndices.count(refName) == 0,
                Exception, "Control reference '" + refName + "' unrecognized.");
        }

        m_controlIndices.push_back(allControlIndices[refName]);
        double refWeight = 1.0;
        if (get_control_weights().contains(refName)) {
            refWeight = get_control_weights().get(refName).getWeight();
        }
        m_control_weights.push_back(refWeight);
        m_refsplines.cloneAndAppend(allSplines[iref]);
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
    for (int iref = 0; iref < m_refsplines.getSize(); ++iref) {
        const auto& modelValue = controls[m_controlIndices[iref]];
        const auto& refValue = m_refsplines[iref].calcValue(timeVec);
        integrand += m_control_weights[iref] * pow(modelValue - refValue, 2);
    }
}