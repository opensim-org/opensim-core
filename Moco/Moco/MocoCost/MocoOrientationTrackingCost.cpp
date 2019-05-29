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
    if (m_rotation_table.getNumColumns() != 0 ||   // rotation table or rotation
            get_rotation_reference_file() != "") { // reference file provided
        TimeSeriesTable_<Rotation> rotationTableToUse;
        if (get_rotation_reference_file() != "") { // rotation reference file
            // Should not be able to supply any two simultaneously.
            assert(get_states_reference_file() == "");
            assert(m_states_table.getNumColumns() == 0);
            assert(m_rotation_table.getNumColumns() == 0);
            rotationTableToUse = readTableFromFile<Rotation>(
                get_rotation_reference_file());

        } else { // rotation table
            // Should not be able to supply any two simultaneously.
            assert(get_states_reference_file() == "");
            assert(m_states_table.getNumColumns() == 0);
            assert(get_rotation_reference_file() == "");
            rotationTableToUse = m_rotation_table;
        }

        // If the frame_paths property is empty, use all frame paths specified
        // in the table's column labels. Otherwise, select only the columns 
        // from the tabel that correspond with paths in frame_paths.
        if (!getProperty_frame_paths().empty()) {
            pathsToUse = rotationTableToUse.getColumnLabels();
            rotationTable = rotationTableToUse;
        } else {
            rotationTable = TimeSeriesTable_<Rotation>(
                rotationTableToUse.getIndependentColumn());
            const auto& labels = rotationTableToUse.getColumnLabels();
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
                    rotationTableToUse.getDependentColumn(path));
            }
        }
        
    } else { // states reference file or states reference provided
        TimeSeriesTable statesTableToUse;
        if (get_states_reference_file() != "") { // states reference file
            // Should not be able to supply any two simultaneously.
            assert(m_states_table.getNumColumns() == 0);
            assert(get_rotation_reference_file() == "");
            assert(m_rotation_table.getNumColumns() == 0);
            statesTableToUse = readTableFromFile<double>(
                    get_states_reference_file());

        } else if (m_states_table.getNumColumns() != 0) { // states table
            // Should not be able to supply any two simultaneously.
            assert(get_states_reference_file() == "");
            assert(get_rotation_reference_file() == "");
            assert(m_rotation_table.getNumColumns() == 0);
            statesTableToUse = m_states_table;

        } else {
            OPENSIM_THROW_FRMOBJ(Exception,
                "Expected user to either provide a reference "
                "file or to programmatically provide a reference table, but "
                "the user supplied neither.");
        }

        // Check that the reference state names match the model state names.
        auto modelStateNames = model.getStateVariableNames();
        auto tableStateNames = statesTableToUse.getColumnLabels();
        for (int i = 0; i < modelStateNames.getSize(); ++i) {
            const auto& name = modelStateNames[i];
            OPENSIM_THROW_IF_FRMOBJ(std::count(tableStateNames.begin(), 
                    tableStateNames.end(), name) == 0, 
                Exception, format("Expected the reference state names to match "
                    "the model state names, but reference state %s not found "
                    "in the model.", name));
        }

        // Create the StatesTrajectory.
        Storage sto = convertTableToStorage(statesTableToUse);
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
            // This realization ignores any SimTK::Motions prescribed in the
            // model.
            model.realizePosition(state);
            std::vector<Rotation> rotations;
            for (const auto& path : pathsToUse) {
                Rotation rotation =
                    model.getComponent<Frame>(path).getRotationInGround(state);
                rotations.push_back(rotation);
            }
            rotationTable.appendRow(state.getTime(), rotations);
        }
        rotationTable.setColumnLabels(pathsToUse);
    }

    // Cache the model frames and rotation weights based on the order of the 
    // rotation table.
    for (int i = 0; i < (int)pathsToUse.size(); ++i) {
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
    for (int irow = 0; irow < (int)rotationTable.getNumRows(); ++irow) {
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

    // Rotation frame symbols: 
    //  G - ground
    //  D - data (reference)
    //  M - model
    integrand = 0;
    for (int iframe = 0; iframe < (int)m_model_frames.size(); ++iframe) {
        const auto& R_GM =
            m_model_frames[iframe]->getRotationInGround(state);

        // Construct a new quaternion object from the splined quaternion data.
        // This constructor normalizes the provided values to ensure that a 
        // valid unit quaternion is created. Other approaches, such as the 
        // Slerp algorithm (https://en.wikipedia.org/wiki/Slerp) may also be
        // valid. However, ensuring that the normalization step is included
        // seems to be sufficient for the purposes of this cost. 
        // https://keithmaggio.wordpress.com/2011/02/15/math-magician-lerp-slerp-and-nlerp/
        const SimTK::Quaternion e(
            m_ref_splines[4*iframe].calcValue(timeVec),
            m_ref_splines[4*iframe + 1].calcValue(timeVec),
            m_ref_splines[4*iframe + 2].calcValue(timeVec),
            m_ref_splines[4*iframe + 3].calcValue(timeVec));
        // Construct a Rotation object from which we'll calcuation an angle-axis 
        // representation of the current orientation error.
        const Rotation R_GD(e);
        const Rotation R_MD = ~R_GM*R_GD;
        const SimTK::Vec4 aa_MD = R_MD.convertRotationToAngleAxis();
        
        // Add this frame's rotation error to the integrand.
        const double& weight = m_rotation_weights[iframe];
        integrand += weight * SimTK::square(aa_MD[0]);
    }
}

