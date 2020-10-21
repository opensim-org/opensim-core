/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoTranslationTrackingGoal.h                                *
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

#include "MocoTranslationTrackingGoal.h"

#include <OpenSim/Moco/MocoUtilities.h>

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/StatesTrajectory.h>

using namespace OpenSim;
using SimTK::Vec3;

void MocoTranslationTrackingGoal::initializeOnModelImpl(const Model& model)
        const {
    // Get the reference data.
    TimeSeriesTableVec3 translationTable;
    if (m_translation_table.getNumColumns() != 0 ||   // translation table or 
            get_translation_reference_file() != "") { // reference file provided
        TimeSeriesTableVec3 translationTableToUse;
        // Should not be able to supply any two simultaneously.
        assert(get_states_reference().empty());
        if (get_translation_reference_file() != "") { // translation ref file
            assert(m_translation_table.getNumColumns() == 0);
            translationTableToUse = TimeSeriesTableVec3(
                    get_translation_reference_file());

        } else { // translation table
            assert(get_translation_reference_file() == "");
            translationTableToUse = m_translation_table;
        }

        // If the frame_paths property is empty, use all frame paths specified
        // in the table's column labels. Otherwise, select only the columns 
        // from the table that correspond with paths in frame_paths.
        if (!getProperty_frame_paths().empty()) {
            m_frame_paths = translationTableToUse.getColumnLabels();
            translationTable = translationTableToUse;
        } else {
            translationTable = TimeSeriesTableVec3(
                translationTableToUse.getIndependentColumn());
            const auto& labels = translationTableToUse.getColumnLabels();
            for (int i = 0; i < getProperty_frame_paths().size(); ++i) {
                const auto& path = get_frame_paths(i);
                OPENSIM_THROW_IF_FRMOBJ(std::find(labels.begin(), labels.end(),
                                                path) == labels.end(),
                        Exception,
                        fmt::format(
                                "Expected frame_paths to match one of the "
                                "column labels in the translation reference, "
                                "but frame path '{}' not found in the "
                                "reference labels.",
                                path));
                m_frame_paths.push_back(path);
                translationTable.appendColumn(path,
                    translationTableToUse.getDependentColumn(path));
            }
        }

    } else { // states reference file or states reference provided
        assert(get_translation_reference_file() != "");
        assert(m_translation_table.getNumColumns() == 0);
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

        // Use the StatesTrajectory to create the table of translation data to
        // be used in the cost.
        for (auto state : statesTraj) {
            // This realization ignores any SimTK::Motions prescribed in the
            // model.
            model.realizePosition(state);
            std::vector<Vec3> translations;
            for (const auto& path : m_frame_paths) {
                Vec3 translation =
                    model.getComponent<Frame>(path).getPositionInGround(state);
                translations.push_back(translation);
            }
            translationTable.appendRow(state.getTime(), translations);
        }
        translationTable.setColumnLabels(m_frame_paths);

    }

    // Check that there are no redundant columns in the reference data.
    TableUtilities::checkNonUniqueLabels(translationTable.getColumnLabels());

    // Cache the model frames and translation weights based on the order of the 
    // translation table.
    for (int i = 0; i < (int)m_frame_paths.size(); ++i) {
        const auto& path = m_frame_paths[i];
        const auto& frame = model.getComponent<Frame>(path);
        m_model_frames.emplace_back(&frame);

        double weight = 1.0;
        if (get_translation_weights().contains(path)) {
            weight = get_translation_weights().get(path).getWeight();
        }
        m_translation_weights.push_back(weight);
    }

    m_ref_splines = GCVSplineSet(translationTable.flatten(
        {"/position_x", "/position_y", "/position_z"}));

    setRequirements(1, 1, SimTK::Stage::Position);
}

void MocoTranslationTrackingGoal::calcIntegrandImpl(
        const IntegrandInput& input, SimTK::Real& integrand) const {
    const auto& time = input.state.getTime();
    getModel().realizePosition(input.state);
    SimTK::Vector timeVec(1, time);

    integrand = 0;
    Vec3 position_ref;
    for (int iframe = 0; iframe < (int)m_model_frames.size(); ++iframe) {
        const auto& position_model =
            m_model_frames[iframe]->getPositionInGround(input.state);

        // Compute position error.

        for (int ip = 0; ip < position_ref.size(); ++ip) {
            position_ref[ip] =
                    m_ref_splines[3*iframe + ip].calcValue(timeVec);
        }
        Vec3 error = position_model - position_ref;

        // Add this frame's position error to the integrand.
        const double& weight = m_translation_weights[iframe];
        integrand += weight * error.normSqr();
    }
}

void MocoTranslationTrackingGoal::printDescriptionImpl() const {
    log_cout("        translation reference file: {}",
             get_translation_reference_file());
    for (int i = 0; i < (int)m_frame_paths.size(); i++) {
        log_cout("        frame {}: {}, weight: {}", i, m_frame_paths[i],
                m_translation_weights[i]);
    }
}
