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

/** A contact group includes a list of contact force component paths in the
model. One of these force elements is designated to locate the position of the
foot (via the 'foot_position_contact_force_path' property) which is necessary to
compute step time asymmetry. The MocoStepTimeAsymmetryGoal determines when a foot
is in contact with the ground when the total contact force from the sum of the
elements in this group exceeds a user provided threshold.

@see MocoStepTimeAsymmetryGoal */
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

/** Minimize or constrain the error between a model's step time asymmetry and a
specified target asymmetry value over a gait cycle.

Step Time Asymmetry (STA) is a percentage and is calculated as follows:
Right Step Time (RST) = Time from left heel-strike to right heel-strike
Left Step Time (LST)  = Time from right heel-strike to left heel-strike
STA = (RST - LST) / (RST + LST)

In this goal, step time asymmetry is estimated by detecting if a foot in contact
with the ground at a given time point. We count positive values when the left
foot is in contact, and negative values when the right is in contact. Therefore,
positive asymmetry means longer left step times, and negative asymmetry means
longer right step times. At time points when both feet are in contact, the step
time is counted towards the leading foot.

The target asymmetry can be set via the 'target_asymmetry' property; a symmetric
step time solution can be achieved by setting this property to zero. This goal
can be used in either 'cost' mode or 'endpoint constraint' model. In 'cost', mode
the error between the target asymmetry and model asymmetry is squared. To make
this goal suitable for gradient-based optimization, step time values are assigned
via smoothing functions which can be controlled via the <TODO> propertie(s).

TODO notes about goal best practices (i.e., only used for bipedal gait, other
necessary constraints, etc.)

@note The only contact element supported is SmoothSphereHalfSpaceForce.

@note Since this goal approximates step time asymmetry, users should calculate
the true asymmetry index after running an optimization.s

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

    /// Add the group of contact forces that determine the position of the left
    /// foot and when it is in contact with the ground.
    void setLeftContactGroup(
            const std::vector<std::string>& contactForcePaths,
            const std::string& footPositionForcePath) {
        set_left_contact_group(MocoStepTimeAsymmetryGoalGroup(
                contactForcePaths, footPositionForcePath));
    }
    /// Add the group of contact forces that determine the position of the right
    /// foot and when it is in contact with the ground.
    void setRightContactGroup(
            const std::vector<std::string>& contactForcePaths,
            const std::string& footPositionForcePath) {
        set_right_contact_group(MocoStepTimeAsymmetryGoalGroup(
                contactForcePaths, footPositionForcePath));
    }

    /// Set the asymmetry value targeted by this goal. If using 'cost' mode, the
    /// error between the target asymmetry and the model asymmetry is squared.
    void setTargetAsymmetry(double asymmetry) {
        set_target_asymmetry(asymmetry);
    }
    double getTargetAsymmetry() { return get_target_asymmetry(); }

    /// Set the threshold force value used to detect is a foot is in contact with
    /// the ground.
    void setContactForceThreshold(double threshold) {
        set_contact_force_threshold(threshold);
    }
    double getContactForceThreshold() { return get_contact_force_threshold(); }

    /// Set the direction in ground of the total contact force component used to
    /// detect foot contact. When the contact force component for a foot exceeds
    /// the force set by the 'contact_force_threshold' property, we register that
    /// foot as in contact with the ground. Acceptable direction values include
    /// "positive-x", "positive-y", "positive-z", "negative-x", "negative-y", and
    /// "negative-z". Default: "positive-y".
    void setContactForceDirection(const std::string& direction) {
        set_contact_force_direction(direction);
    }
    std::string getContactForceDirection() {
        return get_contact_force_direction();
    }

    /// Set the walking direction of the model in the ground frame, which is used
    /// to determine the leading foot during double support. Acceptable direction
    /// values include "positive-x", "positive-y", "positive-z", "negative-x",
    /// "negative-y", and "negative-z". Default: "positive-x".
    void setWalkingDirection(const std::string& direction) {
        set_walking_direction(direction);
    }
    std::string getWalkingDirection() { return get_walking_direction(); }

    /// TODO smoothing docs
    void setAsymmetrySmoothing(double smoothing) {
        set_asymmetry_smoothing(smoothing);
    }
    double getAsymmetrySmoothing() { return get_asymmetry_smoothing(); }

    /// TODO smoothing docs
    void setContactDetectionSmoothing(double smoothing) {
        set_contact_detection_smoothing(smoothing);
    }
    double getContactDetectionSmoothing() {
        return get_contact_detection_smoothing();
    }

protected:
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
    OpenSim_DECLARE_PROPERTY(target_asymmetry, double, "TODO");
    OpenSim_DECLARE_PROPERTY(contact_force_threshold, double, "TODO");
    OpenSim_DECLARE_PROPERTY(contact_force_direction, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(walking_direction, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(asymmetry_smoothing, double, "TODO");
    OpenSim_DECLARE_PROPERTY(contact_detection_smoothing, double, "TODO");

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
