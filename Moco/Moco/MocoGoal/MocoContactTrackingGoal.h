#ifndef MOCO_MOCOCONTACTTRACKINGGOAL_H
#define MOCO_MOCOCONTACTTRACKINGGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoContactTrackingGoal.h                                    *
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

#include "MocoGoal.h"
#include <OpenSim/Simulation/Model/ExternalLoads.h>
#include "../Components/SmoothSphereHalfSpaceForce.h"

namespace OpenSim {

class OSIMMOCO_API MocoContactTrackingGoalGroup : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoContactTrackingGoalGroup, Object);
public:
    OpenSim_DECLARE_LIST_PROPERTY(contact_force_paths, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(external_force_name, std::string, "TODO");
    MocoContactTrackingGoalGroup();
    MocoContactTrackingGoalGroup(
            const std::vector<std::string>& contactForcePaths,
            const std::string& externalForceName);
private:
    void constructProperties();
};


/// The only contact element supported is SmoothSphereHalfSpaceForce.
/// TODO: The error is computed in ground.
/// @ingroup mocogoal
class OSIMMOCO_API MocoContactTrackingGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoContactTrackingGoal, MocoGoal);

public:
    MocoContactTrackingGoal() { constructProperties(); }
    MocoContactTrackingGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoContactTrackingGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }
    // TODO allow specifying the data directly.

    // TODO: How to set external_loads by filename.
    void setExternalLoads(const ExternalLoads& extLoads);

    void addContactGroup(MocoContactTrackingGoalGroup group) {
        append_contact_groups(std::move(group));
    }

    void addContactGroup(
            const std::vector<std::string>& contactForcePaths,
            const std::string& externalForceName) {
        append_contact_groups(MocoContactTrackingGoalGroup(
                contactForcePaths, externalForceName));
    }

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const SimTK::State& state, double& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override {
        cost[0] = input.integral / m_denominator;
    }
    void printDescriptionImpl(std::ostream& stream = std::cout) const override;

private:
    OpenSim_DECLARE_LIST_PROPERTY(contact_groups, MocoContactTrackingGoalGroup,
            "Associate contact elements in the model with force data.");
    OpenSim_DECLARE_PROPERTY(external_loads, ExternalLoads,
            "TODO");
    mutable double m_denominator;

    void constructProperties();

    struct GroupInfo {
        std::vector<std::pair<const SmoothSphereHalfSpaceForce*, int>> contacts;
        GCVSplineSet refSplines;
    };
    mutable std::vector<GroupInfo> m_groups;
};

} // namespace OpenSim

#endif // MOCO_MOCOCONTACTTRACKINGGOAL_H
