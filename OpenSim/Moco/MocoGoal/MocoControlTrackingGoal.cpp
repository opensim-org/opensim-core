/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoControlTrackingGoal.h                                    *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2020 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
 * Contributor(s): Christopher Dembia                                         *
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

#include "MocoControlTrackingGoal.h"

#include <OpenSim/Moco/MocoUtilities.h>

#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

MocoControlTrackingGoalReference::MocoControlTrackingGoalReference() {
    constructProperty_reference("");
}

MocoControlTrackingGoalReference::MocoControlTrackingGoalReference(
        std::string name, std::string reference)
        : MocoControlTrackingGoalReference() {
    setName(std::move(name));
    set_reference(std::move(reference));
}

void MocoControlTrackingGoal::addScaleFactor(const std::string &name,
        const std::string &control, const MocoBounds& bounds) {

    // Ensure that the specified control has reference data associated with it.
    if (getProperty_reference_labels().empty()) {
        const auto& labels = get_reference().process().getColumnLabels();
        bool foundLabel = false;
        for (const auto& label : labels) {
            if (control == label) {
                foundLabel = true;
            }
        }
        OPENSIM_THROW_IF_FRMOBJ(!foundLabel,  Exception,
                "No reference label provided for control '{}'.", control);
    } else {
        OPENSIM_THROW_IF_FRMOBJ(!hasReferenceLabel(control),  Exception,
                "No reference label provided for control '{}'.", control);
    }

    // Update the scale factor map so we can retrieve the correct MocoScaleFactor
    // for this control during initialization.
    m_scaleFactorMap[control] = name;

    // Append the scale factor to the MocoGoal.
    appendScaleFactor(MocoScaleFactor(name, bounds));
}

void MocoControlTrackingGoal::initializeOnModelImpl(const Model& model) const {

    // Get a map between control names and their indices in the model. This also
    // checks that the model controls are in the correct order.
    auto allControlIndices = createSystemControlIndexMap(model);

    // Throw exception if a weight is specified for a nonexistent control.
    for (int i = 0; i < get_control_weights().getSize(); ++i) {
        const auto& weightName = get_control_weights().get(i).getName();
        if (allControlIndices.count(weightName) == 0) {
            OPENSIM_THROW_FRMOBJ(Exception,
                    "Weight provided with name '{}' but this is "
                    "not a recognized control.",
                    weightName);
        }
    }

    // We will populate these variables differently depending on whether the
    // goal is in 'auto' or 'manual' mode.
    std::set<std::string> controlsToTrack;
    std::unordered_map<std::string, std::string> refLabels;

    // If 'manual' labeling mode: only track controls for which the user
    // provided a reference label.
    // Throw exception for controls that do not exist in the model.
    for (int i = 0; i < getProperty_reference_labels().size(); ++i) {
        const auto& controlName = get_reference_labels(i).getName();
        if (allControlIndices.count(controlName) == 0) {
            OPENSIM_THROW_FRMOBJ(Exception,
                    "Control '{}' does not exist in the model.", controlName);
        }
        OPENSIM_THROW_IF_FRMOBJ(controlsToTrack.count(controlName), Exception,
                "Expected control '{}' to be provided no more "
                "than once, but it is provided more than once.",
                controlName);
        controlsToTrack.insert(controlName);
        refLabels[controlName] = get_reference_labels(i).get_reference();
    }

    // TODO: set relativeToDirectory properly.
    TimeSeriesTable tableToUse = get_reference().process();

    // Check that there are no redundant columns in the reference data.
    TableUtilities::checkNonUniqueLabels(tableToUse.getColumnLabels());

    // Convert data table to spline set.
    auto allSplines = GCVSplineSet(tableToUse);

    if (!controlsToTrack.empty()) {
        // The goal is in 'manual' labeling mode; perform error-checking.
        for (const auto& control : controlsToTrack) {
            OPENSIM_THROW_IF_FRMOBJ(!allSplines.contains(refLabels[control]),
                    Exception,
                    "Expected reference to contain label '{}', "
                    "which was associated with control '{}', but "
                    "no such label exists.",
                    refLabels[control], control);
        }
    } else {
        // The goal is in 'auto' labeling mode;
        // populate controlsToTrack, refLabels.
        for (int iref = 0; iref < allSplines.getSize(); ++iref) {
            const auto& refLabel = allSplines[iref].getName();
            if (allControlIndices.count(refLabel) == 0) {
                if (get_allow_unused_references()) { continue; }
                OPENSIM_THROW_FRMOBJ(Exception,
                        "Reference contains column '{}' which does "
                        "not match the name of any control variables.",
                        refLabel);
            }
            // In this labeling mode, the refLabel is the control name.
            controlsToTrack.insert(refLabel);
            refLabels[refLabel] = refLabel;
        }
    }

    // Populate member variables needed to compute the cost.
    const auto& scaleFactors = getModel().getComponentList<MocoScaleFactor>();
    for (const auto& controlToTrack : controlsToTrack) {

        double weight = 1.0;
        if (get_control_weights().contains(controlToTrack)) {
            weight = get_control_weights().get(controlToTrack).getWeight();
        }
        if (weight == 0) {
            continue;
        }

        m_control_indices.push_back(allControlIndices[controlToTrack]);
        m_control_weights.push_back(weight);

        const auto& refLabel = refLabels[controlToTrack];
        // Make sure m_ref_splines contains the reference data for this control.
        if (!m_ref_splines.contains(refLabel)) {
            m_ref_splines.cloneAndAppend(allSplines.get(refLabel));
        }
        // Find the index of the reference spline associated with this control
        // (the same reference may be used for multiple controls).
        m_ref_indices.push_back(m_ref_splines.getIndex(refLabel));

        // For use in printDescriptionImpl().
        m_control_names.push_back(controlToTrack);
        m_ref_labels.push_back(refLabel);

        // Check to see if the model contains a MocoScaleFactor associated with
        // this control.
        bool foundScaleFactor = false;
        for (const auto& scaleFactor : scaleFactors) {
            if (m_scaleFactorMap[controlToTrack] == scaleFactor.getName()) {
                m_scaleFactorRefs.emplace_back(&scaleFactor);
                foundScaleFactor = true;
            }
        }
        // If we didn't find a MocoScaleFactor for this control, set the
        // reference pointer to null.
        if (!foundScaleFactor) {
            m_scaleFactorRefs.emplace_back(nullptr);
        }
    }
    setRequirements(1, 1, SimTK::Stage::Time);
}

void MocoControlTrackingGoal::calcIntegrandImpl(
        const IntegrandInput& input, SimTK::Real& integrand) const {

    const auto& time = input.time;
    SimTK::Vector timeVec(1, time);
    const auto& controls = input.controls;
    getModel().getMultibodySystem().realize(input.state, SimTK::Stage::Time);

    integrand = 0;
    for (int i = 0; i < (int)m_control_indices.size(); ++i) {
        const auto& modelValue = controls[m_control_indices[i]];
        const auto& refValue =
                m_ref_splines[m_ref_indices[i]].calcValue(timeVec);

        // If a scale factor exists for this control, retrieve its value.
        double scaleFactor = 1.0;
        if (m_scaleFactorRefs[i] != nullptr) {
            scaleFactor = m_scaleFactorRefs[i]->getScaleFactor();
        }

        // Compute the tracking error.
        double error = modelValue - (scaleFactor * refValue);

        // Compute the integrand.
        integrand += m_control_weights[i] * error * error;
    }
}

void MocoControlTrackingGoal::printDescriptionImpl() const {
    for (int i = 0; i < (int)m_control_names.size(); i++) {
        log_cout("        control: {}, reference label: {}, weight: {}",
                m_control_names[i], m_ref_labels[i], m_control_weights[i]);
    }
}

