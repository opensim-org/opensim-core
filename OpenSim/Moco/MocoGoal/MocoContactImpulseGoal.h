#ifndef OPENSIM_MOCOCONTACTIMPULSEGOAL_H
#define OPENSIM_MOCOCONTACTIMPULSEGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoContactImpulseGoal.h                                          *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2023 Stanford University and the Authors                     *
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

#include <OpenSim/Moco/osimMoco.h>

namespace OpenSim {

class SmoothSphereHalfSpaceForce;

class OSIMMOCO_API MocoContactImpulseGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoContactImpulseGoal, MocoGoal)

public:
    MocoContactImpulseGoal() { constructProperties(); }
    MocoContactImpulseGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoContactImpulseGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }

    void setContactForcePaths(
            const std::vector<std::string>& contactForcePaths) {
        for (const auto& path : contactForcePaths) {
            append_contact_force_paths(path);
        }
    }

    void setImpulseAxis(int impulseAxis) { set_impulse_axis(impulseAxis); }
    int getImpulseAxis() const { return get_impulse_axis(); }

    void setExtremumType(std::string extremum_type) {
        set_extremum_type(extremum_type);
    }
    std::string getExtremumType() const { return get_extremum_type(); }

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& input, double& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override {
        cost[0] = (input.integral * input.integral) / m_denominator;
    }
    //void printDescriptionImpl() const override;

private:
    OpenSim_DECLARE_LIST_PROPERTY(contact_force_paths, std::string,
            "Paths to SmoothSphereHalfSpaceForce objects in the model whose "
            "combined contact impulse is optimized.");
    OpenSim_DECLARE_PROPERTY(impulse_axis, int,
            "The axis of the ground reaction force impulse component to be "
            "optimized in the goal (X = 0, Y = 1, Z = 2; default = -1).");
    OpenSim_DECLARE_PROPERTY(extremum_type, std::string,
            "The type of extremum to be taken in the goal: "
            "'minimum', 'maximum', or 'none' (default = 'none').");
    OpenSim_DECLARE_PROPERTY(smoothing_factor, double,
            "The smoothing factor applied in the approximation of the "
            "extremum function (default = 1.0). This property can be set "
            "between [0.2, 1.0]. For forces that may take on large values "
            "(> ~2000) during a simulation, it is recommended to set this "
            "property closer to 0.2.");

    void constructProperties();

    mutable double m_denominator;
    mutable std::vector<const SmoothSphereHalfSpaceForce*> m_contacts;
    mutable int m_beta = 1;
    mutable bool m_apply_extremum = false;
    mutable double m_smoothing_factor = 1.0;
};
} // namespace OpenSim

#endif // OPENSIM_MOCOCONTACTIMPULSEGOAL_H