/* -------------------------------------------------------------------------- *
 * OpenSim: MocoContactInitializer.cpp                                        *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2024 Stanford University and the Authors                     *
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

#include "MocoContactInitializer.h"

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Actuators/ModelOperators.h>
#include <OpenSim/Common/CommonUtilities.h>

#include <casadi/casadi.hpp>

using namespace OpenSim;

class ContactTrackingObjective : public casadi::Callback {
public:
    ContactTrackingObjective(Model model) {
        m_model = std::move(model);
        m_state = m_model.initSystem();
    }

    // CASADI::CALLBACK INTERFACE
    casadi_int get_n_in() override { return 2;}
    casadi_int get_n_out() override { return 1;}

    casadi::Sparsity get_sparsity_in(casadi_int i) override {
        if (i == 0) {
            return casadi::Sparsity::dense(m_state.getNQ(), 1);
        } else if (i == 1) {
            return casadi::Sparsity::dense(m_state.getNU(), 1);
        } else {
            return casadi::Sparsity::dense(0, 0);
        }
    }

    casadi::Sparsity get_sparsity_out(casadi_int i) override {
        if (i == 0) {
            return casadi::Sparsity::dense(1, 1);
        } else {
            return casadi::Sparsity::dense(0, 0);
        }
    }

private:
    Model m_model;
    mutable SimTK::State m_state;


    SimTK::Vector m_time;
    SimTK::RowVector m_forceRef;
    SimTK::RowVector m_statesRef;
    SimTK::RowVector m_previousSolution;

};




void MocoContactInitializer::constructProperties() {
    constructProperty_model(ModelProcessor(Model{}));
    constructProperty_external_loads();
    constructProperty_contact_groups();
    constructProperty_markers_reference(MarkersReference());
}


Model MocoContactInitializer::initializeModel() {

    // TODO probably remove.
    ModelProcessor modelProcessor = getModelProcessor();
    modelProcessor.append(ModOpRemoveMuscles());
    Model model = modelProcessor.process();
    model.initSystem();

    return model;
}

void MocoContactInitializer::initializeExternalLoads(const Model& model) {

    // Get the ExternalLoads object.
    std::unique_ptr<ExternalLoads> extLoadsFromFile;
    const ExternalLoads* extLoads;
    if (!getProperty_external_loads().empty()) {
        OPENSIM_THROW_IF_FRMOBJ(!getProperty_external_loads_file().empty(),
                Exception,
                "Expected either an ExternalLoads file or object, but both "
                "were provided.");
        extLoads = &get_external_loads();
    } else if (!get_external_loads_file().empty()) {
        extLoadsFromFile = OpenSim::make_unique<ExternalLoads>(
                get_external_loads_file(), true);
        extLoads = extLoadsFromFile.get();
    } else {
        OPENSIM_THROW_FRMOBJ(Exception, "No ExternalLoads provided.");
    }

    // Spline the data.
    const std::string dataFilePath =
            convertRelativeFilePathToAbsoluteFromXMLDocument(
                    extLoads->getDocumentFileName(),
                    extLoads->getDataFileName());
    TimeSeriesTable data(dataFilePath);
    GCVSplineSet allRefSplines(data);

    // Each ExternalForce has an applied_to_body property. For the ExternalForce
    // to be properly paired with a group of contact force components, the
    // contact force components must also apply forces to the same body. Here,
    // we find which of the two bodies in each contact force component matches
    // the ExternalForce body.
    for (int ig = 0; ig < getProperty_contact_groups().size(); ++ig) {
        const auto& group = get_contact_groups(ig);

        OPENSIM_THROW_IF_FRMOBJ(
                !extLoads->contains(group.get_external_force_name()), Exception,
                "External force '{}' not found.",
                group.get_external_force_name());
        const auto& extForce =
                extLoads->get(group.get_external_force_name());

        GroupInfo groupInfo;
        for (int ic = 0; ic < group.getProperty_contact_force_paths().size();
                ++ic) {
            const auto& path = group.get_contact_force_paths(ic);
            const auto& contactForce =
                    model.getComponent<Force>(path);

            // TODO
            // int recordOffset = findRecordOffset(group, contactForce,
            //         extForce.get_applied_to_body());

            // groupInfo.contacts.push_back(
            //         std::make_pair(&contactForce, recordOffset));
        }

        // Gather the relevant data splines for this contact group.
        // We assume that the "x", "y", and "z" columns could have been in any
        // order.
        const std::string& forceID = extForce.get_force_identifier();
        groupInfo.refSplines.cloneAndAppend(allRefSplines.get(forceID + "x"));
        groupInfo.refSplines.cloneAndAppend(allRefSplines.get(forceID + "y"));
        groupInfo.refSplines.cloneAndAppend(allRefSplines.get(forceID + "z"));

        // Check which frame the contact force data is expressed in.
        groupInfo.refExpressedInFrame = nullptr;
        if (extForce.get_force_expressed_in_body() != "ground") {
            const auto& forceExpressedInBody =
                    extForce.get_force_expressed_in_body();
            if (model.hasComponent<PhysicalFrame>(forceExpressedInBody)) {
                groupInfo.refExpressedInFrame =
                        &model.getComponent<PhysicalFrame>(
                                forceExpressedInBody);
            } else if (model.hasComponent<PhysicalFrame>(
                               "./bodyset/" + forceExpressedInBody)) {
                groupInfo.refExpressedInFrame =
                        &model.getComponent<PhysicalFrame>(
                                "./bodyset/" + forceExpressedInBody);
            } else {
                OPENSIM_THROW_FRMOBJ(Exception,
                        "Could not find '{}' in the model or the BodySet.",
                        forceExpressedInBody);
            }
        }

        // Compute normalization factors.
        std::vector<std::string> suffixes = {"x", "y", "z"};
        for (int i = 0; i < 3; ++i) {
            double factor = SimTK::max(
                data.getDependentColumn(forceID + suffixes[i]).abs());
            OPENSIM_THROW_IF_FRMOBJ(factor < SimTK::SignificantReal,
                    Exception,
                    "The property `normalize_tracking_error` was enabled, "
                    "but the peak magnitude of the ground contact force "
                    "data in the {}-direction is close to zero.",
                    suffixes[i]);
            groupInfo.scaleFactors[i] = factor;
        }
        

        m_groups.push_back(groupInfo);
    }
}