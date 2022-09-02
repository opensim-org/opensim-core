/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoContactImpulseTrackingGoal.cpp                           *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2022 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicos Haralabidis                                               *
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

#include "MocoContactImpulseTrackingGoal.h"
#include <OpenSim/Simulation/Model/SmoothSphereHalfSpaceForce.h>

using namespace OpenSim;
using RefPtrMSF = SimTK::ReferencePtr<const MocoScaleFactor>;

MocoContactImpulseTrackingGoalGroup::MocoContactImpulseTrackingGoalGroup() {
    constructProperties();
}

MocoContactImpulseTrackingGoalGroup::MocoContactImpulseTrackingGoalGroup(
        const std::vector<std::string>& contactForcePaths,
        const std::string& externalForceName) {
    constructProperties();
    for (const auto& path : contactForcePaths) {
        append_contact_force_paths(path);
    }
    set_external_force_name(externalForceName);
}

MocoContactImpulseTrackingGoalGroup::MocoContactImpulseTrackingGoalGroup(
        const std::vector<std::string>& contactForcePaths,
        const std::string& externalForceName,
        const std::vector<std::string>& altFramePaths) :
        MocoContactImpulseTrackingGoalGroup(contactForcePaths, externalForceName) {
    for (const auto& path : altFramePaths) {
        append_alternative_frame_paths(path);
    }
}

void MocoContactImpulseTrackingGoalGroup::constructProperties() {
    constructProperty_contact_force_paths();
    constructProperty_external_force_name("");
    constructProperty_alternative_frame_paths();
}


void MocoContactImpulseTrackingGoal::constructProperties() {
    constructProperty_contact_groups();
    constructProperty_external_loads();
    constructProperty_external_loads_file("");
    constructProperty_impulse_axis(-1);
}

void MocoContactImpulseTrackingGoal::setExternalLoadsFile(
    const std::string& extLoadsFile) {
    updProperty_external_loads().clear();
    set_external_loads_file(extLoadsFile);
}

void MocoContactImpulseTrackingGoal::setExternalLoads(const ExternalLoads& extLoads) {
    updProperty_external_loads_file().clear();
    set_external_loads(extLoads);
}

void MocoContactImpulseTrackingGoal::addScaleFactor(const std::string& name,
    const std::string& externalForceName,
    const MocoBounds& bounds) {

    // Check that the contact group associated with the provided external force
    // name exists.
    bool foundContactGroup = false;
    for (int ig = 0; ig < getProperty_contact_groups().size(); ++ig) {
        const auto& group = get_contact_groups(ig);
        if (externalForceName == group.get_external_force_name()) {
            foundContactGroup = true;
        }
    }
    OPENSIM_THROW_IF_FRMOBJ(!foundContactGroup, Exception,
        "Contact group associated with external force '{}' not found.",
        externalForceName);

    // Update the scale factor map so we can retrieve the correct MocoScaleFactor
    // for this contact group during initialization.
    m_scaleFactorMap[externalForceName] = name;

    // Append the scale factor to the MocoGoal.
    appendScaleFactor(MocoScaleFactor(name, bounds));
}

void MocoContactImpulseTrackingGoal::initializeOnModelImpl(const Model& model) const {

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
    }
    else if (!get_external_loads_file().empty()) {
        extLoadsFromFile = OpenSim::make_unique<ExternalLoads>(
            get_external_loads_file(), true);
        extLoads = extLoadsFromFile.get();
    }
    else {
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
    const auto& scaleFactors = getModel().getComponentList<MocoScaleFactor>();
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
                model.getComponent<SmoothSphereHalfSpaceForce>(path);

            int recordOffset = findRecordOffset(group, contactForce,
                extForce.get_applied_to_body());

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
            }
            else if (model.hasComponent<PhysicalFrame>(
                    "./bodyset/" + forceExpressedInBody)) {
                groupInfo.refExpressedInFrame =
                        &model.getComponent<PhysicalFrame>(
                            "./bodyset/" + forceExpressedInBody);
            }
            else {
                OPENSIM_THROW_FRMOBJ(Exception,
                    "Could not find '{}' in the model or the BodySet.",
                    forceExpressedInBody);
            }
        }

        m_groups.push_back(groupInfo);

        // Check to see if the model contains a MocoScaleFactor associated with
        // this contact group.
        RefPtrMSF thisScaleFactorRef;
        bool foundScaleFactor = false;
        for (const auto& scaleFactor : scaleFactors) {
            if (m_scaleFactorMap[group.get_external_force_name()] == scaleFactor.getName()) {
                thisScaleFactorRef = &scaleFactor;
                foundScaleFactor = true;
            }
        }
        // If we didn't find a MocoScaleFactor for this force direction, set
        // the reference pointer to null.
        if (!foundScaleFactor) {
            thisScaleFactorRef = nullptr;
        }
        m_scaleFactorRefs.push_back(thisScaleFactorRef);
        
        // Check that the length of scale factors == 1; As the goal does not
        // support multiple integrals currently
        if (m_scaleFactorRefs.size() != 1)
            OPENSIM_THROW_FRMOBJ(Exception,
                "Expected size of scale factors to be 1, but "
                " got '{}.",
                m_scaleFactorRefs.size());

    }

    // Which axis should the impulse be tracked for?
    if (get_impulse_axis() < 0 || get_impulse_axis() > 2)
        OPENSIM_THROW_FRMOBJ(Exception,
            "Expected 'impulse_axis' to be either 0, 1 or 2, but "
            "got '{}'.",
            get_impulse_axis());

    // Check the number of contact groups which have been added; 
    // The MocoImpulseTrackingGoal currently only supports tracking 
    // the impulse from a single limb at a time
    if (getProperty_contact_groups().size() > 1)
        OPENSIM_THROW_FRMOBJ(Exception,
            "Expected a single (1) contact group, but "
            "got '{}'.",
            getProperty_contact_groups().size());
    
    setRequirements(1, 1, SimTK::Stage::Velocity);
}

int MocoContactImpulseTrackingGoal::findRecordOffset(
        const MocoContactImpulseTrackingGoalGroup& group,
        const SmoothSphereHalfSpaceForce& contactForce,
        const std::string& appliedToBody) const {

    // Is the ExternalForce applied to the sphere's body?
    const auto& sphereBase =
        contactForce.getConnectee<ContactSphere>("sphere")
            .getConnectee<PhysicalFrame>("frame")
            .findBaseFrame();
    const std::string& sphereBaseName = sphereBase.getName();
    if (sphereBaseName == appliedToBody) {
        // We want the first 3 entries in
        // SmoothSphereHalfSpaceForce::getRecordValues(), which contain
        // forces on the sphere.
        return 0;
    }

    // Is the ExternalForce applied to the half space's body?
    const auto& halfSpaceBase =
        contactForce.getConnectee<ContactHalfSpace>("half_space")
            .getConnectee<PhysicalFrame>("frame")
            .findBaseFrame();
    const std::string& halfSpaceBaseName = halfSpaceBase.getName();
    if (halfSpaceBaseName == appliedToBody) {
        // We want the forces applied to the half space, which are
        // entries 6, 7, 8 in
        // SmoothSphereHalfSpaceForce::getRecordValues().
        return 6;
    }

    // Check the group's alternative frames.
    // Check each alternative frame path in the order provided. As soon as one
    // of these paths matches the name of the sphere base frame or
    // half space base frame, use the contact forces applied to that base frame.
    const auto& sphereBasePath = sphereBase.getAbsolutePathString();
    const auto& halfSpaceBasePath = halfSpaceBase.getAbsolutePathString();
    for (int ia = 0; ia < group.getProperty_alternative_frame_paths().size();
            ++ia) {
        const auto& path = group.get_alternative_frame_paths(ia);
        if (path == sphereBasePath) { return 0; }
        if (path == halfSpaceBasePath) { return 6; }
    }

    OPENSIM_THROW_FRMOBJ(Exception,
        "Contact force '{}' has sphere base frame '{}' and half space base "
        "frame '{}'. One of these frames should match the applied_to_body "
        "setting ('{}') of ExternalForce '{}', or match one of the "
        "alternative_frame_paths, but no match found.",
        contactForce.getAbsolutePathString(), sphereBaseName,
        halfSpaceBaseName, appliedToBody, group.get_external_force_name());
}

void MocoContactImpulseTrackingGoal::calcIntegrandImpl(
    const IntegrandInput& input, double& integrand) const {
    const auto& state = input.state;
    const auto& time = state.getTime();
    getModel().realizeVelocity(state);
    SimTK::Vector timeVec(1, time);

    integrand = 0;
    SimTK::Vec3 force_ref;
    for (int ig = 0; ig < (int)m_groups.size(); ++ig) {

        // Get contact groups.
        const auto& group = m_groups[ig];

        // Model force.
        double force_model(0);
        for (const auto& entry : group.contacts) {
            Array<double> recordValues = entry.first->getRecordValues(state);
            const auto& recordOffset = entry.second;
            force_model += recordValues[recordOffset + get_impulse_axis()];
        }

        // Reference force.
        for (int ir = 0; ir < force_ref.size(); ++ir) {
            force_ref[ir] = group.refSplines[ir].calcValue(timeVec);
        }

        // Re-express the reference force.
        if (group.refExpressedInFrame) {
            group.refExpressedInFrame->expressVectorInGround(state, force_ref);
        }

        // Apply scale factors for this marker, if they exist.
        const auto& scaleFactorRef = m_scaleFactorRefs[ig];
        double scaleFactor = 1.0;
        if (scaleFactorRef != nullptr) {
            scaleFactor = scaleFactorRef->getScaleFactor();
        }

        double error = force_model - scaleFactor * force_ref[get_impulse_axis()];
        integrand += error;
    }

}

void MocoContactImpulseTrackingGoal::printDescriptionImpl() const {
    log_cout("        impulse axis: {}", get_impulse_axis());
    for (int ig = 0; ig < getProperty_contact_groups().size(); ++ig) {
        const auto& group = get_contact_groups(ig);
        log_cout("        group {}: ExternalForce: {}",
            ig, group.get_external_force_name());
        log_cout("            forces:");
        for (int ic = 0; ic < group.getProperty_contact_force_paths().size();
                ++ic) {
            log_cout("                {}", group.get_contact_force_paths(ic));
        }
    }
}