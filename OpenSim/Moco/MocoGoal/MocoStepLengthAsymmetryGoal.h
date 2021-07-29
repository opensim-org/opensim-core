#ifndef OPENSIM_MOCOSTEPLENGTHASYMMETRY_H
#define OPENSIM_MOCOSTEPLENGTHASYMMETRY_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoStepLengthAsymmetryGoal.h                                     *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2021 Stanford University and the Authors                     *
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

/** Minimize the error between a model's step time asymmetry and a specified
target asymmetry value over a gait cycle.

Step Length Asymmetry (SLA) is a ratio and is calculated as follows:
The Right Step Length (RSL) is the distance between feet at right foot strike
The Left Step Length (LSL) is the distance between feet at left foot strike
Step Length Asymmetry = (RSL - LSL) / (RSL + LSL)

In this goal, the distance between feet, or "foot frames", is limited throughout
the gait cycle. The goal calculates the distance between the left foot and right
foot, then limits the distance between feet to not pass beyond minimum (negative)
or maximum (positive) bounds. There are two limits used: one that limits the
distance between feet when the right foot is in front, and one that limits the
distance between feet when the left foot is in front.

Users must provide the target asymmetry value via 'setTargetAsymmetry()'.
Asymmetry values ranges from -1.0 to 1.0. For example, 0.20 is 20% positive
step length asymmetry with greater right step length than left step length. A
symmetric step length solution can be achieved by setting this property to zero.
This goal can be used only in 'cost' mode. To make this goal suitable for
gradient-based optimization, step length values are assigned via a smoothing
function which can be controlled via 'setAsymmetrySmoothing()'.

Users must also prescribe the stride length via 'setStrideLength()'. The goal
then calculates the minimum and maximum bounds on the distance between right
and left foot. Users must ensure that this stride length is met via problem
bounds or other goals; the value provided to MocoStepLengthAsymmetryGoal is
only used to compute the model's asymmetry in the cost function.

@note This goal is designed for simulations of bipedal gait.

@note Since this goal approximates step length asymmetry, users should calculate
the true asymmetry value after running an optimization.

@ingroup mocogoal */
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

    /// Set the body frame associated with the left foot.
    void setLeftFootFrame(const std::string& frame) {
        set_left_foot_frame(frame);
    }
    std::string getLeftFootFrame() { return get_left_foot_frame(); }
    /// Set the body frame associated with the right foot.
    void setRightFootFrame(const std::string& frame) {
        set_right_foot_frame(frame);
    }
    std::string getRightFootFrame() { return get_right_foot_frame(); }

    /// Set the asymmetry value targeted by this goal. The error between the
    /// target asymmetry and the model asymmetry is squared in the integrand.
    void setTargetAsymmetry(double asymmetry) {
        set_target_asymmetry(asymmetry);
    }
    double getTargetAsymmetry() { return get_target_asymmetry(); }

    /// Set the known stride length of your walking simulation. This value is
    /// necessary to compute step length asymmetry.
    void setStrideLength(double length) {
        set_stride_length(length);
    }
    double getStrideLength() { return get_stride_length(); }

    /// Set the walking direction of the model in the ground frame, which is used
    /// to compute step lengths. Acceptable direction values include
    /// "positive-x", "positive-y", "positive-z", "negative-x", "negative-y", and
    /// "negative-z". Default: "positive-x".
    void setWalkingDirection(const std::string& direction) {
        set_walking_direction(direction);
    }
    std::string getWalkingDirection() { return get_walking_direction(); }

    /// Set the values that determines the smoothing of the asymmetry
    /// computation. This term is necessary since this computation is non-smooth.
    /// Larger smoothing values mean that larger step length errors are required
    /// for a given step to be counted towards the total asymmetry error
    /// minimized in the cost.
    void setAsymmetrySmoothing(double smoothing) {
        set_asymmetry_smoothing(smoothing);
    }
    double getAsymmetrySmoothing() { return get_asymmetry_smoothing(); }

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& input, double& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override;
    void printDescriptionImpl() const override;

private:
    // PROPERTIES
    OpenSim_DECLARE_PROPERTY(left_foot_frame, std::string,
            "The model frame associated with the left foot.");
    OpenSim_DECLARE_PROPERTY(right_foot_frame, std::string,
            "The model frame associated with the right foot.");
    OpenSim_DECLARE_PROPERTY(target_asymmetry, double,
            "The target asymmetry value, between -1.0 and 1.0. Positive "
            "asymmetry is associated with the right leg, and negative asymmetry "
            "for the left. Default: 0");
    OpenSim_DECLARE_PROPERTY(asymmetry_smoothing, double,
            "The value that determines the smoothing of the asymmetry "
            "computation. Larger values mean that larger step length errors are "
            "required for a step to be counted towards the asymmetry error. "
            "Default: 5.");
    OpenSim_DECLARE_PROPERTY(stride_length, double,
            "The known stride length of the simulation, in meters."
            "Default is -1; the user must provide a positive value.");
    OpenSim_DECLARE_PROPERTY(walking_direction, std::string,
            "The walking direction of the model in the ground frame, which "
            "is used to determine the leading foot during double support. "
            "Acceptable direction values include 'positive-x', 'positive-y', "
            "'positive-z', 'negative-x', 'negative-y', and 'negative-z'. "
            "Default: 'positive-x'.");

    void constructProperties();

    mutable SimTK::ReferencePtr<const Frame> m_left_foot_frame;
    mutable SimTK::ReferencePtr<const Frame> m_right_foot_frame;

    mutable int m_walking_direction_index;
    mutable int m_walking_direction_sign;

    mutable double m_left_foot_threshold;
    mutable double m_right_foot_threshold;

    using ConditionalFunction =
        double(const double&, const double&, const double&, const double&);
    mutable std::function<ConditionalFunction> m_conditional;
};

} // namespace OpenSim

#endif //OPENSIM_MOCOSTEPLENGTHASYMMETRY_H
