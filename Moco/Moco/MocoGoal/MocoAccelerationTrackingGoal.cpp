/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoAccelerationTrackingGoal.cpp                             *
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

#include "MocoAccelerationTrackingGoal.h"

#include "../MocoUtilities.h"
#include <algorithm>

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/StatesTrajectory.h>

using namespace OpenSim;
using SimTK::Vec3;

void MocoAccelerationTrackingGoal::initializeOnModelImpl(
        const Model& model) const {
    // Get the reference data.
    TimeSeriesTableVec3 accelerationTable;
    std::vector<std::string> pathsToUse;
    if (m_acceleration_table.getNumColumns() != 0 ||   // acceleration table or
            get_acceleration_reference_file() != "") { // reference file provided
        TimeSeriesTableVec3 accelerationTableToUse;
        // Should not be able to supply any two simultaneously.
        if (get_acceleration_reference_file() != "") { // acceleration ref file
            assert(m_acceleration_table.getNumColumns() == 0);
            accelerationTableToUse =
                    readTableFromFileT<Vec3>(get_acceleration_reference_file());

        } else { // acceleration table
            assert(get_acceleration_reference_file() == "");
            accelerationTableToUse = m_acceleration_table;
        }

        // If the frame_paths property is empty, use all frame paths specified
        // in the table's column labels. Otherwise, select only the columns
        // from the tabel that correspond with paths in frame_paths.
        if (!getProperty_frame_paths().empty()) {
            pathsToUse = accelerationTableToUse.getColumnLabels();
            accelerationTable = accelerationTableToUse;
        } else {
            accelerationTable = TimeSeriesTableVec3(
                    accelerationTableToUse.getIndependentColumn());
            const auto& labels = accelerationTableToUse.getColumnLabels();
            for (int i = 0; i < getProperty_frame_paths().size(); ++i) {
                const auto& path = get_frame_paths(i);
                OPENSIM_THROW_IF_FRMOBJ(
                    std::find(labels.begin(), labels.end(), path) == 
                        labels.end(),
                    Exception,
                    format("Expected frame_paths to match at least one of the "
                       "column labels in the acceleration reference, but frame "
                       "path '%s' not found in the reference labels.", path));
                pathsToUse.push_back(path);
                accelerationTable.appendColumn(path, 
                    accelerationTableToUse.getDependentColumn(path));
            }
        }

    }

    // Check that there are no redundant columns in the reference data.
    checkRedundantLabels(accelerationTable.getColumnLabels());

    // Cache the model frames and acceleration weights based on the order of the
    // acceleration table.
    for (int i = 0; i < (int)pathsToUse.size(); ++i) {
        const auto& path = pathsToUse[i];
        const auto& frame = model.getComponent<Frame>(path);
        m_model_frames.emplace_back(&frame);

        double weight = 1.0;
        if (get_acceleration_weights().contains(path)) {
            weight = get_acceleration_weights().get(path).getWeight();
        }
        m_acceleration_weights.push_back(weight);
    }

    // Create a new scalar-valued TimeSeriesTable using the time index from the
    // acceleration table argument. We'll populate this table with the
    // acceleration values we need when calculating the integral tracking cost,
    // namely the frame acceleration vector elements.
    TimeSeriesTable flatTable(accelerationTable.getIndependentColumn());

    // This matrix has the input table number of columns times three to hold all
    // acceleration elements.
    SimTK::Matrix mat((int)accelerationTable.getNumRows(),
                      3*(int)accelerationTable.getNumColumns());
    // Column labels are necessary for creating the GCVSplineSet from the table,
    // so we'll label each column using the frame path and the acceleration 
    // vector element (e.g. "<frame-path>/acceleration_a0" for the first 
    // acceleration vector element).
    std::vector<std::string> colLabels;
    for (int irow = 0; irow < (int)accelerationTable.getNumRows(); ++irow) {
        const auto row = accelerationTable.getRowAtIndex(irow);

        // Get acceleration vector elements.
        int icol = 0;
        for (int ielt = 0; ielt < row.size(); ++ielt) {
            const auto& label = accelerationTable.getColumnLabel(ielt);
            const auto& p = row[ielt];
            for (int ip = 0; ip < p.size(); ++ip) {
                mat.updElt(irow, icol++) = p[ip];
                if (!irow) {
                    colLabels.push_back(format("%s/acceleration_a%i", label, 
                        ip));
                }
            }
        }
    }

    flatTable.updMatrix() = mat;
    flatTable.setColumnLabels(colLabels);

    m_ref_splines = GCVSplineSet(flatTable);

    setNumIntegralsAndOutputs(1, 1);
}

void MocoAccelerationTrackingGoal::calcIntegrandImpl(const SimTK::State& state, 
        double& integrand) const {
    const auto& time = state.getTime();
    getModel().realizeAcceleration(state);
    SimTK::Vector timeVec(1, time);

    integrand = 0;
    for (int iframe = 0; iframe < (int)m_model_frames.size(); ++iframe) {
        const auto& a_model =
                m_model_frames[iframe]->getLinearAccelerationInGround(state);

        // Compute acceleration error.
        Vec3 a_ref(0.0);
        for (int ia = 0; ia < a_ref.size(); ++ia) {
            a_ref[ia] = m_ref_splines[3*iframe + ia].calcValue(timeVec);
        }
        Vec3 error = a_model - a_ref;

        // Add this frame's acceleration error to the integrand.
        const double& weight = m_acceleration_weights[iframe];
        integrand += weight * error.normSqr();
    }
}

void MocoAccelerationTrackingGoal::printDescriptionImpl(
        std::ostream& stream) const {
    stream << "        ";
    stream << "acceleration reference file: " 
           << get_acceleration_reference_file()
           << std::endl;
    for (int i = 0; i < getProperty_frame_paths().size(); i++) {
        stream << "        ";
        stream << "frame " << i << ": " << get_frame_paths(i) << std::endl;
    }
}