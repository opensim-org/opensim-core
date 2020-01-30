/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoContactTrackingGoal.cpp                                  *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2020 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
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

#include "MocoContactTrackingGoal.h"

using namespace OpenSim;

MocoContactTrackingGoalGroup::MocoContactTrackingGoalGroup() {
    constructProperties();
}

MocoContactTrackingGoalGroup::MocoContactTrackingGoalGroup(
        const std::vector<std::string>& contactForcePaths,
        const std::string& externalForceName) {
    constructProperties();
    for (const auto& path : contactForcePaths) {
        append_contact_force_paths(path);
    }
    set_external_force_name(externalForceName);
}

void MocoContactTrackingGoalGroup::constructProperties() {
    constructProperty_contact_force_paths();
    constructProperty_external_force_name("");
}


void MocoContactTrackingGoal::constructProperties() {
    constructProperty_contact_groups();
    constructProperty_external_loads();
    constructProperty_external_loads_file("");
    constructProperty_projection("none");
    constructProperty_projection_vector();
}

void MocoContactTrackingGoal::setExternalLoadsFile(
        const std::string& extLoadsFile) {
    updProperty_external_loads().clear();
    set_external_loads_file(extLoadsFile);
}

void MocoContactTrackingGoal::setExternalLoads(const ExternalLoads& extLoads) {
    updProperty_external_loads_file().clear();
    set_external_loads(extLoads);
}

void MocoContactTrackingGoal::initializeOnModelImpl(const Model& model) const {

    // Calculate the denominator.
    m_denominator = model.getTotalMass(model.getWorkingState());
    const double gravityAccelMagnitude = model.get_gravity().norm();
    if (gravityAccelMagnitude > SimTK::SignificantReal) {
        m_denominator *= gravityAccelMagnitude;
    }

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
    const std::string dataFilePath = getAbsolutePathnameFromXMLDocument(
            extLoads->getDocumentFileName(), extLoads->getDataFileName());
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
                !extLoads->contains(group.get_external_force_name()),
                Exception,
                format("External force '%s' not found.",
                        group.get_external_force_name()));
        const auto& extForce =
                extLoads->get(group.get_external_force_name());
        const std::string& appliedToBody = extForce.get_applied_to_body();

        GroupInfo groupInfo;
        for (int ic = 0; ic < group.getProperty_contact_force_paths().size();
                ++ic) {
            const auto& path = group.get_contact_force_paths(ic);
            const auto& contactForce =
                    model.getComponent<SmoothSphereHalfSpaceForce>(path);

            // First, assume we want the first 3 entries in
            // SmoothSphereHalfSpaceForce::getRecordValues(), which contain
            // forces on the sphere.
            int recordOffset = 0;
            const std::string& sphereBaseName =
                    contactForce.getConnectee<PhysicalFrame>("sphere_frame")
                            .findBaseFrame()
                            .getName();
            if (sphereBaseName != appliedToBody) {
                // The ExternalForce is not applied to the sphere's body.
                const std::string& halfSpaceBaseName =
                        contactForce
                                .getConnectee<PhysicalFrame>("half_space_frame")
                                .findBaseFrame()
                                .getName();
                OPENSIM_THROW_IF_FRMOBJ(halfSpaceBaseName != appliedToBody,
                        Exception,
                        format("Contact force '%s' has sphere base frame '%s' "
                               "and half space base frame '%s'; one of these "
                               "frames should match the applied_to_body "
                               "setting ('%s') of ExternalForce '%s'.",
                                contactForce.getAbsolutePathString(),
                                sphereBaseName, halfSpaceBaseName,
                                appliedToBody,
                                group.get_external_force_name()));
                // We want the forces applied to the half space, which are
                // entries 6, 7, 8 in
                // SmoothSphereHalfSpaceForce::getRecordValues().
                recordOffset = 6;
            }
            groupInfo.contacts.push_back(
                    std::make_pair(&contactForce, recordOffset));
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
                OPENSIM_THROW_FRMOBJ(
                        Exception, format("Could not find '%s' in the model or "
                                          "the BodySet.",
                                           forceExpressedInBody));
            }
        }

        m_groups.push_back(groupInfo);
    }

    // Should the contact force errors be projected onto a plane or vector?
    if (get_projection() == "vector") {
        m_projectionType = ProjectionType::Vector;
    } else if (get_projection() == "plane") {
        m_projectionType = ProjectionType::Plane;
    } else if (get_projection() != "none") {
        OPENSIM_THROW_FRMOBJ(
                Exception, format("Expected 'projection' to be 'none', "
                                  "'vector', or 'plane', but got '%s'.",
                                   get_projection()));
    }
    if (m_projectionType != ProjectionType::None) {
        OPENSIM_THROW_IF_FRMOBJ(getProperty_projection_vector().empty(),
                Exception, "Must provide a value for 'projection_vector'.");
        m_projectionVector = SimTK::UnitVec3(get_projection_vector());
    }

    setNumIntegralsAndOutputs(1, 1);
}

void MocoContactTrackingGoal::calcIntegrandImpl(
        const SimTK::State& state, double& integrand) const {
    const auto& time = state.getTime();
    getModel().realizeVelocity(state);
    SimTK::Vector timeVec(1, time);

    integrand = 0;
    SimTK::Vec3 force_ref;
    for (const auto& group : m_groups) {

        // Model force.
        SimTK::Vec3 force_model(0);
        for (const auto& entry : group.contacts) {
            Array<double> recordValues = entry.first->getRecordValues(state);
            const auto& recordOffset = entry.second;
            for (int im = 0; im < force_model.size(); ++im) {
                force_model[im] += recordValues[recordOffset + im];
            }
        }

        // Reference force.
        for (int ir = 0; ir < force_ref.size(); ++ir) {
            force_ref[ir] = group.refSplines[ir].calcValue(timeVec);
        }

        // Re-express the reference force.
        if (group.refExpressedInFrame) {
            group.refExpressedInFrame->expressVectorInGround(state, force_ref);
        }

        SimTK::Vec3 error3D = force_model - force_ref;

        // Project the error.
        SimTK::Vec3 error;
        if (m_projectionType == ProjectionType::None) {
            error = error3D;
        } else if (m_projectionType == ProjectionType::Vector) {
            error = SimTK::dot(error3D, m_projectionVector) *
                    m_projectionVector;
        } else {
            error = error3D - SimTK::dot(error3D, m_projectionVector) *
                                      m_projectionVector;
        }

        integrand += error.normSqr();
    }
}

void MocoContactTrackingGoal::printDescriptionImpl(std::ostream& stream) const {
    stream << "        ";
    stream << "projection type: " << get_projection() << std::endl;
    if (m_projectionType != ProjectionType::None) {
        stream << "        ";
        stream << "projection vector: " << get_projection_vector() << std::endl;
    }
    for (int ig = 0; ig < getProperty_contact_groups().size(); ++ig) {
        const auto& group = get_contact_groups(ig);
        stream << "        ";
        stream << "group " << ig
               << ": ExternalForce: " << group.get_external_force_name()
               << std::endl;
        stream << "            ";
        stream << "forces: " << std::endl;
        for (int ic = 0; ic < group.getProperty_contact_force_paths().size();
                ++ic) {
            stream << "                ";
            stream << group.get_contact_force_paths(ic) << std::endl;
        }
    }
}
