/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoOrientationTrackingGoal.h                                *
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

#include "MocoOrientationTrackingGoal.h"

#include <OpenSim/Moco/MocoUtilities.h>

#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Simulation/Model/Frame.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/StatesTrajectory.h>

using namespace OpenSim;

void MocoOrientationTrackingGoal::initializeOnModelImpl(const Model& model)
        const {
    // Get the reference data.
    TimeSeriesTable_<SimTK::Rotation_<double>> rotationTable;
    if (m_rotation_table.getNumColumns() != 0 ||   // rotation table or rotation
            get_rotation_reference_file() != "") { // reference file provided
        TimeSeriesTable_<SimTK::Rotation_<double>> rotationTableToUse;
        // Should not be able to supply any two simultaneously.
        assert(get_states_reference().empty());
        if (get_rotation_reference_file() != "") { // rotation reference file
            assert(m_rotation_table.getNumColumns() == 0);
            rotationTableToUse = TimeSeriesTable_<SimTK::Rotation_<double>>(
                    get_rotation_reference_file());

        } else { // rotation table
            assert(get_rotation_reference_file() == "");
            rotationTableToUse = m_rotation_table;
        }

        // If the frame_paths property is empty, use all frame paths specified
        // in the table's column labels. Otherwise, select only the columns 
        // from the table that correspond with paths in frame_paths.
        if (!getProperty_frame_paths().empty()) {
            m_frame_paths = rotationTableToUse.getColumnLabels();
            rotationTable = rotationTableToUse;
        } else {
            rotationTable = TimeSeriesTable_<SimTK::Rotation_<double>>(
                rotationTableToUse.getIndependentColumn());
            const auto& labels = rotationTableToUse.getColumnLabels();
            for (int i = 0; i < getProperty_frame_paths().size(); ++i) {
                const auto& path = get_frame_paths(i);
                OPENSIM_THROW_IF_FRMOBJ(std::find(labels.begin(), labels.end(),
                                                path) == labels.end(),
                        Exception,
                        "Expected frame_paths to match one of the "
                        "column labels in the rotation reference, but frame "
                        "path '{}' not found in the reference labels.",
                        path);
                m_frame_paths.push_back(path);
                rotationTable.appendColumn(path,
                    rotationTableToUse.getDependentColumn(path));
            }
        }

    } else { // states reference file or states reference provided
        assert(get_rotation_reference_file() == "");
        assert(m_rotation_table.getNumColumns() == 0);
        // TODO: set relativeToDirectory properly.
        TimeSeriesTable statesTableToUse =
                get_states_reference().processAndConvertToRadians(model);

        // Check that the reference state names match the model state names.
        checkLabelsMatchModelStates(model, statesTableToUse.getColumnLabels());

        // Create the StatesTrajectory.
        auto statesTraj = StatesTrajectory::createFromStatesTable(
                model, statesTableToUse);

        // Use all paths provided in frame_paths.
        OPENSIM_THROW_IF_FRMOBJ(getProperty_frame_paths().empty(), Exception,
            "Expected paths in the frame_paths property, but none were found.");
        for (int i = 0; i < getProperty_frame_paths().size(); ++i) {
            m_frame_paths.push_back(get_frame_paths(i));
        }

        // Use the StatesTrajectory to create the table of rotation data to
        // be used in the cost.
        for (auto state : statesTraj) {
            // This realization ignores any SimTK::Motions prescribed in the
            // model.
            model.realizePosition(state);
            std::vector<SimTK::Rotation_<double>> rotations;
            for (const auto& path : m_frame_paths) {
                SimTK::Rotation_<double> rotation =
                    model.getComponent<Frame>(path).getRotationInGround(state);
                rotations.push_back(rotation);
            }
            rotationTable.appendRow(state.getTime(), rotations);
        }
        rotationTable.setColumnLabels(m_frame_paths);
    }

    // Check that there are no redundant columns in the reference data.
    TableUtilities::checkNonUniqueLabels(rotationTable.getColumnLabels());

    // Cache the model frames and rotation weights based on the order of the 
    // rotation table.
    for (int i = 0; i < (int)m_frame_paths.size(); ++i) {
        const auto& path = m_frame_paths[i];
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
                    colLabels.push_back(
                            fmt::format("{}/quaternion_e{}", label, ie));
                }
            }
        }
    }
    flatTable.updMatrix() = mat;
    flatTable.setColumnLabels(colLabels);

    m_ref_splines = GCVSplineSet(flatTable);

    setRequirements(1, 1, SimTK::Stage::Position);
}

void MocoOrientationTrackingGoal::calcIntegrandImpl(
        const IntegrandInput& input, SimTK::Real& integrand) const {
    const auto& time = input.state.getTime();
    getModel().realizePosition(input.state);
    SimTK::Vector timeVec(1, time);

    // Rotation frame symbols: 
    //  G - ground
    //  D - data (reference)
    //  M - model
    integrand = 0;
    for (int iframe = 0; iframe < (int)m_model_frames.size(); ++iframe) {
        const auto& R_GM =
            m_model_frames[iframe]->getRotationInGround(input.state);

        // Construct a new quaternion object from the splined quaternion data.
        // This constructor normalizes the provided values to ensure that a 
        // valid unit quaternion is created. Other approaches, such as the 
        // Slerp algorithm (https://en.wikipedia.org/wiki/Slerp) may also be
        // valid. However, ensuring that the normalization step is included
        // seems to be sufficient for the purposes of this cost. 
        // https://keithmaggio.wordpress.com/2011/02/15/math-magician-lerp-slerp-and-nlerp/
        const SimTK::Quaternion_<double> e(
            m_ref_splines[4*iframe].calcValue(timeVec),
            m_ref_splines[4*iframe + 1].calcValue(timeVec),
            m_ref_splines[4*iframe + 2].calcValue(timeVec),
            m_ref_splines[4*iframe + 3].calcValue(timeVec));
        // Construct a Rotation object from which we'll calculate an angle-axis
        // representation of the current orientation error.
        const SimTK::Rotation_<double> R_GD(e);
        const SimTK::Rotation_<double> R_MD = ~R_GM*R_GD;
        const SimTK::Vec4 aa_MD = R_MD.convertRotationToAngleAxis();

        // Add this frame's rotation error to the integrand.
        const double& weight = m_rotation_weights[iframe];
        integrand += weight * SimTK::square(aa_MD[0]);
    }
}

void MocoOrientationTrackingGoal::printDescriptionImpl() const {
    log_cout("        rotation reference file: {}",
            get_rotation_reference_file());
    for (int i = 0; i < (int)m_frame_paths.size(); i++) {
        log_cout("        frame {}: {}, weight: {}",
                i, m_frame_paths[i], m_rotation_weights[i]);
    }
}

