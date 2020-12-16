/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoAngularVelocityTrackingGoal.cpp                          *
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

#include "MocoAngularVelocityTrackingGoal.h"

#include <OpenSim/Moco/MocoUtilities.h>

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/StatesTrajectory.h>

using namespace OpenSim;
using SimTK::Vec3;

void MocoAngularVelocityTrackingGoal::initializeOnModelImpl(
        const Model& model) const {
    // Get the reference data.
    TimeSeriesTableVec3 angularVelocityTable;
    if (m_angular_velocity_table.getNumColumns() != 0 || // ang. vel. table or
            get_angular_velocity_reference_file() !=
                    "") { // reference file provided
        TimeSeriesTableVec3 angularVelocityTableToUse;
        // Should not be able to supply any two simultaneously.
        assert(get_states_reference().empty());
        if (get_angular_velocity_reference_file() != "") { // ang. vel. ref file
            assert(m_angular_velocity_table.getNumColumns() == 0);
            angularVelocityTableToUse = TimeSeriesTableVec3(
                    get_angular_velocity_reference_file());

        } else { // ang. vel. table
            assert(get_angular_velocity_reference_file() == "");
            angularVelocityTableToUse = m_angular_velocity_table;
        }

        // If the frame_paths property is empty, use all frame paths specified
        // in the table's column labels. Otherwise, select only the columns
        // from the table that correspond with paths in frame_paths.
        if (!getProperty_frame_paths().empty()) {
            m_frame_paths = angularVelocityTableToUse.getColumnLabels();
            angularVelocityTable = angularVelocityTableToUse;
        } else {
            angularVelocityTable = TimeSeriesTableVec3(
                    angularVelocityTableToUse.getIndependentColumn());
            const auto& labels = angularVelocityTableToUse.getColumnLabels();
            for (int i = 0; i < getProperty_frame_paths().size(); ++i) {
                const auto& path = get_frame_paths(i);
                OPENSIM_THROW_IF_FRMOBJ(std::find(labels.begin(), labels.end(),
                                                path) == labels.end(),
                        Exception,
                        "Expected frame_paths to match one of the column "
                        "labels in the angular velocity reference, but frame "
                        "path '{}' not found in the reference labels.",
                        path);
                m_frame_paths.push_back(path);
                angularVelocityTable.appendColumn(path,
                        angularVelocityTableToUse.getDependentColumn(path));
            }
        }

    } else { // states reference file or states reference provided
        assert(get_angular_velocity_reference_file() != "");
        assert(m_angular_velocity_table.getNumColumns() == 0);
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
                "Expected paths in the frame_paths property, but none were "
                "found.");
        for (int i = 0; i < getProperty_frame_paths().size(); ++i) {
            m_frame_paths.push_back(get_frame_paths(i));
        }

        // Use the StatesTrajectory to create the table of angular velocity data
        // to be used in the cost.
        for (auto state : statesTraj) {
            // This realization ignores any SimTK::Motions prescribed in the
            // model.
            model.realizeVelocity(state);
            std::vector<Vec3> angularVelocities;
            for (const auto& path : m_frame_paths) {
                Vec3 angularVelocity =
                        model.getComponent<Frame>(path)
                                .getAngularVelocityInGround(state);
                angularVelocities.push_back(angularVelocity);
            }
            angularVelocityTable.appendRow(state.getTime(), angularVelocities);
        }
        angularVelocityTable.setColumnLabels(m_frame_paths);
    }

    // Check that there are no redundant columns in the reference data.
    TableUtilities::checkNonUniqueLabels(
            angularVelocityTable.getColumnLabels());

    // Cache the model frames and angular velocity weights based on the order of
    // the angular velocity table.
    for (int i = 0; i < (int)m_frame_paths.size(); ++i) {
        const auto& path = m_frame_paths[i];
        const auto& frame = model.getComponent<Frame>(path);
        m_model_frames.emplace_back(&frame);

        double weight = 1.0;
        if (get_angular_velocity_weights().contains(path)) {
            weight = get_angular_velocity_weights().get(path).getWeight();
        }
        m_angular_velocity_weights.push_back(weight);
    }

    m_ref_splines = GCVSplineSet(angularVelocityTable.flatten(
        {"/angular_velocity_x", "/angular_velocity_y", "/angular_velocity_z"}));

    setRequirements(1, 1, SimTK::Stage::Velocity);
}

void MocoAngularVelocityTrackingGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
    const auto& state = input.state;
    const auto& time = state.getTime();
    getModel().realizeVelocity(state);
    SimTK::Vector timeVec(1, time);

    integrand = 0;
    Vec3 angular_velocity_ref(0.0);
    for (int iframe = 0; iframe < (int)m_model_frames.size(); ++iframe) {
        const auto& angular_velocity_model =
                m_model_frames[iframe]->getAngularVelocityInGround(state);

        // Compute angular velocity error.
        for (int iw = 0; iw < angular_velocity_ref.size(); ++iw) {
            angular_velocity_ref[iw] =
                    m_ref_splines[3 * iframe + iw].calcValue(timeVec);
        }
        Vec3 error = angular_velocity_model - angular_velocity_ref;

        // Add this frame's angular velocity error to the integrand.
        const double& weight = m_angular_velocity_weights[iframe];
        integrand += weight * error.normSqr();
    }
}

void MocoAngularVelocityTrackingGoal::printDescriptionImpl() const {
    log_cout("        angular velocity reference file: {}",
            get_angular_velocity_reference_file());
    for (int i = 0; i < (int)m_frame_paths.size(); i++) {
        log_cout("        frame {}: {}, weight: {}", i, m_frame_paths[i],
                m_angular_velocity_weights[i]);
    }
}
