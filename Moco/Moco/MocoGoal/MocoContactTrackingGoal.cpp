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
    constructProperty_external_loads(ExternalLoads());
}

void MocoContactTrackingGoal::setExternalLoads(const ExternalLoads& extLoads) {
    set_external_loads(extLoads);
}

void MocoContactTrackingGoal::initializeOnModelImpl(const Model& model) const {

    m_denominator = model.getTotalMass(model.getWorkingState());
    const double gravityAccelMagnitude = model.get_gravity().norm();
    if (gravityAccelMagnitude > SimTK::SignificantReal) {
        m_denominator *= gravityAccelMagnitude;
    }

    TimeSeriesTable data(get_external_loads().getDataFileName());

    // TODO: use different spline sets for different groups.
    GCVSplineSet allRefSplines(data);

    for (int ig = 0; ig < getProperty_contact_groups().size(); ++ig) {
        const auto& group = get_contact_groups(ig);

        OPENSIM_THROW_IF_FRMOBJ(
                !get_external_loads().contains(group.get_external_force_name()),
                Exception,
                format("External force '%s' not found.",
                        group.get_external_force_name()));
        const auto& extForce =
                get_external_loads().get(group.get_external_force_name());
        const std::string& appliedToBody = extForce.get_applied_to_body();

        GroupInfo groupInfo;
        for (int ic = 0; ic < group.getProperty_contact_force_paths().size();
                ++ic) {
            const auto& path = group.get_contact_force_paths(ic);
            const auto& contactForce =
                    model.getComponent<SmoothSphereHalfSpaceForce>(path);

            // Assume we want the first 3 entries in
            // SmoothSphereHalfSpaceForce::getRecordValues(), which contain
            // forces on the sphere.
            int recordOffset = 0;
            const std::string& sphereBaseName =
                    contactForce.getConnectee<PhysicalFrame>("sphere_frame")
                            .findBaseFrame()
                            .getName();
            if (sphereBaseName != appliedToBody) {
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
                // We want forces applied to the half space, which are entries
                // 6, 7, 8 in SmoothSphereHalfSpaceForce::getRecordValues().
                recordOffset = 6;
            }
            groupInfo.contacts.push_back(
                    std::make_pair(&contactForce, recordOffset));
        }

        const std::string& forceID = extForce.get_force_identifier();
        groupInfo.refSplines.cloneAndAppend(allRefSplines.get(forceID + "x"));
        groupInfo.refSplines.cloneAndAppend(allRefSplines.get(forceID + "y"));
        groupInfo.refSplines.cloneAndAppend(allRefSplines.get(forceID + "z"));

        m_groups.push_back(groupInfo);
    }

    // TODO: Make sure ExternalForce applied_to_body and all forces connect to that body.
    // Use applied_to_body to determine whether we want the force on the sphere or the half space.


    // TODO: Error-check the groups.

    setNumIntegralsAndOutputs(1, 1);
}

void MocoContactTrackingGoal::calcIntegrandImpl(
        const SimTK::State& state, double& integrand) const {
    const auto& time = state.getTime();
    getModel().realizeVelocity(state);
    SimTK::Vector timeVec(1, time);

    // TODO: This duplicates some of the code in ExternalForce; perhaps use that
    // code directly?

    // TODO support projecting or omitting a component/measure.

    integrand = 0;
    SimTK::Vec3 force_model(0);
    SimTK::Vec3 force_ref;
    for (const auto& group : m_groups) {

        for (const auto& entry : group.contacts) {
            Array<double> recordValues =
                    entry.first->getRecordValues(state);
            const auto& recordOffset = entry.second;
            for (int im = 0; im < force_model.size(); ++im) {
                force_model[im] += recordValues[recordOffset + im];
            }
        }

        for (int ir = 0; ir < force_ref.size(); ++ir) {
            force_ref[ir] = group.refSplines[ir].calcValue(timeVec);
            // TODO: re-express in ground if necessary.
        }

        integrand += (force_model - force_ref).normSqr();
    }
}

void MocoContactTrackingGoal::printDescriptionImpl(std::ostream& stream) const {
    // TODO
    // for (int i = 0; i < getProperty_contact_groups().size(); ++i) {
    //     stream << "                ";
    //     stream << "group " << i << ": contact forces: "; // TODO
    // }
}
