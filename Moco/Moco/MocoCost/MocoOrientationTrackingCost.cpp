/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoOrientationTrackingCost.h                                *
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

#include "MocoOrientationTrackingCost.h"

#include <algorithm>
#include "../MocoUtilities.h"
#include <OpenSim/Simulation/StatesTrajectory.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Frame.h>
#include <OpenSim/Common/TimeSeriesTable.h>

using namespace OpenSim;

void MocoOrientationTrackingCost::initializeOnModelImpl(const Model& model) 
        const {
    // Get the reference data.
    TimeSeriesTable_<Rotation> rotationTable;
    std::vector<std::string> pathsToUse;
    if (m_rotation_table.getNumColumns() != 0) { // rotation table provided
        // Should not be able to supply any two simultaneously.
        assert(get_reference_file() == "");
        assert(m_states_table.getNumColumns() == 0);

        // If the frame_paths property is empty, use all frame paths specified
        // in the table's column labels. Otherwise, select only the columns 
        // from the tabel that correspond with paths in frame_paths.
        if (!getProperty_frame_paths().empty()) {
            pathsToUse = m_rotation_table.getColumnLabels();
            rotationTable = m_rotation_table;
        } else {
            rotationTable = TimeSeriesTable_<Rotation>(
                m_rotation_table.getIndependentColumn());
            const auto& labels = m_rotation_table.getColumnLabels();
            for (int i = 0; i < getProperty_frame_paths().size(); ++i) {
                const auto& path = get_frame_paths(i);
                OPENSIM_THROW_IF_FRMOBJ(
                    std::find(labels.begin(), labels.end(), path) == 
                        labels.end(),
                    Exception,
                    format("Expected frame_paths to match at least one of the "
                        "column labels in the rotation reference, but frame "
                        "path '%s' not found in the reference labels.", path));
                pathsToUse.push_back(path);
                rotationTable.appendColumn(path,
                    m_rotation_table.getDependentColumn(path));
            }
        }
    } else { // states reference file or states reference provided
        TimeSeriesTable tableToUse;
        if (get_reference_file() != "") { // states reference file
            // Should not be able to supply any two simultaneously.
            assert(m_states_table.getNumColumns() == 0);
            assert(m_rotation_table.getNumColumns() == 0);

            auto tablesFromFile = FileAdapter::readFile(get_reference_file());
            // There should only be one table.
            OPENSIM_THROW_IF_FRMOBJ(tablesFromFile.size() != 1, Exception,
                format("Expected reference file '%s' to contain 1 table, but "
                    "it contains %i tables.", get_reference_file(), 
                    tablesFromFile.size()));
            // Get the first table.
            auto* firstTable =
                dynamic_cast<TimeSeriesTable*>(
                    tablesFromFile.begin()->second.get());
            OPENSIM_THROW_IF_FRMOBJ(!firstTable, Exception,
                "Expected reference file to contain a (scalar) "
                "TimeSeriesTable, but it contains a different type of table.");
            tableToUse = *firstTable;

        } else if (m_states_table.getNumColumns() != 0) { // states reference
            // Should not be able to supply any two simultaneously.
            assert(get_reference_file() == "");
            assert(m_rotation_table.getNumColumns() == 0);
            tableToUse = m_states_table;

        } else {
            OPENSIM_THROW_FRMOBJ(Exception,
                "Expected user to either provide a reference "
                "file or to programmatically provide a reference table, but "
                "the user supplied neither.");
        }

        // Create the StatesTrajectory.
        Storage sto = convertTableToStorage(tableToUse);
        auto statesTraj = StatesTrajectory::createFromStatesStorage(model, sto);

        // Use all paths provided in frame_paths.
        OPENSIM_THROW_IF_FRMOBJ(getProperty_frame_paths().empty(), Exception, 
            "Expected paths in the frame_paths property, but none were found.");
        for (int i = 0; i < getProperty_frame_paths().size(); ++i) {
            pathsToUse.push_back(get_frame_paths(i));
        }

        // Use the StatesTrajectory to create the table of rotation data to
        // be used in the cost.
        for (auto state : statesTraj) {
            model.getSystem().prescribe(state);
            model.realizePosition(state);
            std::vector<Rotation> rotations;
            for (const auto& path : pathsToUse) {
                Rotation rotation =
                    model.getComponent<Frame>(path).getOutputValue<Rotation>(
                        state, "rotation");
                rotations.push_back(rotation);
            }
            rotationTable.appendRow(state.getTime(), rotations);
        }
        rotationTable.setColumnLabels(pathsToUse);
    }

    // Cache the model frames and rotation weights based on the order of the 
    // rotation table.
    for (int i = 0; i < pathsToUse.size(); ++i) {
        const auto& path = pathsToUse[i];
        const auto& frame = model.getComponent<Frame>(path);
        m_model_frames.emplace_back(&frame);

        double weight = 1.0;
        if (get_rotation_weights().contains(path)) {
            weight = get_rotation_weights().get(path).getWeight();
        }
        m_rotation_weights.push_back(weight);
    }

    // Create a new scalar-valued TimeSeriesTable using the time index from the
    // rotation table argument. We'll populate this table with the rotation
    // values we need when calculating the integral tracking cost, namely a 
    // quaternion representation of the rotations.
    TimeSeriesTable flatTable(rotationTable.getIndependentColumn());

    // This matrix has the input table number of columns times four to hold all
    // four quaternion elements per rotation.
    SimTK::Matrix mat((int)rotationTable.getNumRows(), 
                    4*(int)rotationTable.getNumColumns());
    // Column labels are necessary for creating the GCVSplineSet from the table,
    // so we'll label each column using the frame path and the quaternion
    // element (e.g. "<frame-path>/quaternion_e0" for the first quaternion
    // element).
    std::vector<std::string> colLabels;
    for (int irow = 0; irow < rotationTable.getNumRows(); ++irow) {
        const auto row = rotationTable.getRowAtIndex(irow);

        // Get quaternion elements from rotations.
        int icol = 0;
        for (int ielt = 0; ielt < row.size(); ++ielt) {
            const auto& label = rotationTable.getColumnLabel(ielt);
            const auto& R = row[ielt];
            auto e = R.convertRotationToQuaternion();
            for (int ie = 0; ie < e.size(); ++ie) {
                mat.updElt(irow, icol++) = e[ie];
                if (!irow) {
                    colLabels.push_back(format("%s/quaternion_e%i", label, ie));
                }
            }
        }
    }
    flatTable.updMatrix() = mat;
    flatTable.setColumnLabels(colLabels);

    m_ref_splines = GCVSplineSet(flatTable);
}

void MocoOrientationTrackingCost::calcIntegralCostImpl(const SimTK::State& state,
        double& integrand) const {
    const auto& time = state.getTime();
    getModel().realizePosition(state);
    SimTK::Vector timeVec(1, time);

    integrand = 0;
    for (int iframe = 0; iframe < (int)m_model_frames.size(); ++iframe) {
        const auto& R_GD =
            m_model_frames[iframe]->getRotationInGround(state);

        // Use quaternion data at this time point to construct a Rotation object
        // from which we'll calcuation an angle-axis representation of the 
        // current orientation error.
        const SimTK::Quaternion e(
            m_ref_splines[4*iframe].calcValue(timeVec),
            m_ref_splines[4*iframe + 1].calcValue(timeVec),
            m_ref_splines[4*iframe + 2].calcValue(timeVec),
            m_ref_splines[4*iframe + 3].calcValue(timeVec));
        const Rotation R_GM(e);
        const Rotation R_DM = ~R_GD*R_GM;
        const SimTK::Vec4 aa_DM = R_DM.convertRotationToAngleAxis();
        
        // Add this frame's rotation error to the integrand.
        const double& weight = m_rotation_weights[iframe];
        integrand += weight * SimTK::square(aa_DM[0]);
    }
}

