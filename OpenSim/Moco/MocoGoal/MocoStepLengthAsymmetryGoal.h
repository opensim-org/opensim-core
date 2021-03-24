#ifndef OPENSIM_MOCOSTEPLENGTHASYMMETRY_H
#define OPENSIM_MOCOSTEPLENGTHASYMMETRY_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoStepLengthAsymmetryGoal.h                                     *
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

class OSIMMOCO_API MocoStepLengthAsymmetryGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoStepLengthAsymmetryGoal, MocoGoal);
public:
    MocoStepLengthAsymmetryGoal() { constructProperties(); }
    MocoStepLengthAsymmetryGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoStepLengthAsymmetryGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }

    /// TODO
    void setLeftFootFrame(const std::string& frame) {
        set_left_foot_frame(frame);
    }
    std::string getLeftFootFrame() { return get_left_foot_frame(); }
    /// TODO
    void setRightFootFrame(const std::string& frame) {
        set_right_foot_frame(frame);
    }
    std::string getRightFootFrame() { return get_right_foot_frame(); }

    /// Set the asymmetry value targeted by this goal. If using 'cost' mode, the
    /// error between the target asymmetry and the model asymmetry is squared.
//    void setTargetAsymmetry(double asymmetry) {
//        set_target_asymmetry(asymmetry);
//    }
//    double getTargetAsymmetry() { return get_target_asymmetry(); }

    /// TODO
    void setFootVelocityThreshold(double threshold) {
        set_foot_velocity_threshold(threshold);
    }
    double setFootVelocityThreshold() { return get_foot_velocity_threshold(); }

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
    void setSmoothing(double smoothing) { set_smoothing(smoothing); }
    double getSmoothing() { return get_smoothing(); }

    OpenSim_DECLARE_PROPERTY(target_right_foot_position, double, "TODO");
    OpenSim_DECLARE_PROPERTY(target_left_foot_position, double, "TODO");
    OpenSim_DECLARE_PROPERTY(target_final_right_foot_position, double, "TODO");

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
    // void printDescriptionImpl() const override;

private:
    OpenSim_DECLARE_PROPERTY(left_foot_frame, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(right_foot_frame, std::string, "TODO");
//    OpenSim_DECLARE_PROPERTY(target_asymmetry, double, "TODO");
    OpenSim_DECLARE_PROPERTY(foot_velocity_threshold, double, "TODO");
    OpenSim_DECLARE_PROPERTY(contact_force_direction, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(walking_direction, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(smoothing, double, "TODO");
    void constructProperties();

    mutable SimTK::ReferencePtr<const Frame> m_left_foot_frame;
    mutable SimTK::ReferencePtr<const Frame> m_right_foot_frame;

    mutable int m_walking_direction_index;
    mutable int m_walking_direction_sign;
    mutable int m_contact_force_index;
    mutable int m_contact_force_sign;

    using ConditionalFunction =
        double(const double&, const double&, const double&, const double&);
    mutable std::function<ConditionalFunction> m_conditional;
};

} // namespace OpenSim

#endif //OPENSIM_MOCOSTEPLENGTHASYMMETRY_H
