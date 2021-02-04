#ifndef OPENSIM_MOCOSTEPTIMEASYMMETRYGOAL_H
#define OPENSIM_MOCOSTEPTIMEASYMMETRYGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoStepTimeAsymmetryGoal.h                                       *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2020 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Russell Johnson, Nicholas Bianco                                *
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

namespace OpenSim {

class SmoothSphereHalfSpaceForce;

class OSIMMOCO_API MocoStepTimeAsymmetryGoalGroup : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoStepTimeAsymmetryGoalGroup, Object);
public:
    OpenSim_DECLARE_LIST_PROPERTY(contact_force_paths, std::string,
            "Paths to SmoothSphereHalfSpaceForce objects on one foot of the "
            "model whose forces are summed to determine when the foot is in "
            "contact with the ground.");
    OpenSim_DECLARE_PROPERTY(foot_position_contact_force_path, std::string,
            "Path to a SmoothSphereHalfSpaceForce whose ContactSphere is used "
            "to locate the position of the foot, which is necessary for "
            "computing the step time during the double support phase of "
            "walking. This path should match one of the paths in "
            "'contact_force_paths'.");
    MocoStepTimeAsymmetryGoalGroup();
    MocoStepTimeAsymmetryGoalGroup(
            const std::vector<std::string>& contactForcePaths,
            const std::string& footPositionForcePath);
private:
    void constructProperties();
};

// TODO does the user need to specify "front foot"
// TODO try to reduce the number of properties
// TODO what should be the range for target asymmetry?

/** Minimize or constrain the error between a model's step time asymmetry and a
specified target asymmetry value over a gait cycle.

Step Time Asymmetry is calculated as follows:
Right Step Time (RST) = Time from left heel-strike to right heel-strike
Left Step Time (LST)  = Time from right heel-strike to left heel-strike
Step Time Asymmetry = (RST - LST) / (RST + LST)

This Moco goal is to calculate step time (a)symmetry for a
MOCO gait optimization. The target symmetry (=0) or asymmetry can be set
via a Target Asymmetry Input. This goal could be implemented more like a
constraint (like the Moco Speed Goal), or it can be used more like an
objective function term, with a weighting. The naming convention for the
heel contact sphere is used to grab the location of the foot, this could
be made generic by setting that outside and passing in as an option.

Negative values indicate quicker right steps than left steps, positive
values indicate quicker left steps than right steps.

@note Due to necessary limitations within method used here, user should
calculate the asymmetry index after running an optimization to either
confirm that it matched the target symmetry, or find the deviation from the
target symmetry.

@ingroup mocogoal */
class OSIMMOCO_API MocoStepTimeAsymmetryGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoStepTimeAsymmetryGoal, MocoGoal);
public:
    MocoStepTimeAsymmetryGoal() { constructProperties(); }
    MocoStepTimeAsymmetryGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoStepTimeAsymmetryGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }

    void setLeftContactGroup(
            const std::vector<std::string>& contactForcePaths,
            const std::string& footPositionForcePath) {
        set_left_contact_group(MocoStepTimeAsymmetryGoalGroup(
                contactForcePaths, footPositionForcePath));
    }
    void setRightContactGroup(
            const std::vector<std::string>& contactForcePaths,
            const std::string& footPositionForcePath) {
        set_right_contact_group(MocoStepTimeAsymmetryGoalGroup(
                contactForcePaths, footPositionForcePath));
    }

    void setTargetAsymmetry(double asymmetry) {
        set_target_asymmetry(asymmetry);
    }
    double getTargetAsymmetry() { return get_target_asymmetry(); }

    void setContactForceDirection(const std::string& direction) {
        set_contact_force_direction(direction);
    }
    std::string getContactForceDirection() {
        return get_contact_force_direction();
    }

    void setContactForceThreshold(double threshold) {
        set_contact_force_threshold(threshold);
    }
    double getContactForceThreshold() { return get_contact_force_threshold(); }

    void setWalkingDirection(const std::string& direction) {
        set_walking_direction(direction);
    }
    std::string getWalkingDirection() { return get_walking_direction(); }

    void setSmoothing(double smoothing) { set_smoothing(smoothing); }
    double getSmoothing() { return get_smoothing(); }

protected:
    bool getSupportsEndpointConstraintImpl() const override { return true; }
    Mode getDefaultModeImpl() const override {
        return Mode::EndpointConstraint;
    }
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& input, double& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override;
//    void printDescriptionImpl() const override;

private:
    OpenSim_DECLARE_PROPERTY(left_contact_group, MocoStepTimeAsymmetryGoalGroup,
            "Paths to SmoothSphereHalfSpaceForce objects on the left foot of "
            "the model whose forces are summed to determine when the left foot "
            "is in contact with the ground.");
    OpenSim_DECLARE_PROPERTY(right_contact_group, MocoStepTimeAsymmetryGoalGroup,
            "Paths to SmoothSphereHalfSpaceForce objects on the right foot of "
            "the model whose forces are summed to determine when the right foot "
            "is in contact with the ground.");
    OpenSim_DECLARE_PROPERTY(contact_force_direction, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(contact_force_threshold, double, "TODO");
    OpenSim_DECLARE_PROPERTY(walking_direction, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(smoothing, double, "TODO");
    OpenSim_DECLARE_PROPERTY(target_asymmetry, double, "TODO");
    void constructProperties();

    mutable std::vector<SimTK::ReferencePtr<const SmoothSphereHalfSpaceForce>>
        m_left_contacts;
    mutable std::vector<SimTK::ReferencePtr<const SmoothSphereHalfSpaceForce>>
        m_right_contacts;
    mutable SimTK::ReferencePtr<const Frame> m_left_frame;
    mutable SimTK::ReferencePtr<const Frame> m_right_frame;

    mutable int m_walking_direction_index;
    mutable int m_walking_direction_sign;
    mutable int m_contact_force_index;
    mutable int m_contact_force_sign;

    using ConditionalFunction =
        double(const double&, const double&, const double&, const double&);
    mutable std::function<ConditionalFunction> m_conditional;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOSTEPTIMEASYMMETRYGOAL_H
