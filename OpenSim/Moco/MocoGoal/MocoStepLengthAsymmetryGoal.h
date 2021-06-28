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

/** This goal works by constraining the distance between feet, or "foot frames",
throughout the gait cycle. The goal calculates the distance between the left
foot and right foot, then enforces a constraint with minimum (negative) and
maximum (positive) bounds on the distance between feet. There are two
constraints: one that constrains the distance between feet when the right foot
is in front, and one that constrains the distance between fee when the left
foot is in front.

The Right Step Length (RSL) is the distance between feet at right foot strike
The Left Step Length (LSL) is the distance between feet at left foot strike
Step Length Asymmetry = (RSL - LSL)/ (RSL + LSL)

Asymmetry Ranges from -1 to 1, for example: 0.20 is 20% positive step
length asymmetry with greater right step length than left step length.

Users input the stride length and target step length asymmetry. The
goal then calculates the minimum and maximum bounds on the distance
between right and left foot.

Because this goal doesn't directly compute the step length
asymmetry from heel strike data, users should confirm that the step
length asymmetry from the solution matches closely to their target.
Additionally, in some cases users may want to set target asymmetries
above or below the desired value, in the event there is some offset. To do
this, we provide the helper function computeStepAsymmetryValues() below.

For this goal, one potential limitation is you need to specify both the
stride length and target speed (and therefore, the stride time as well).
This is necessary for the way this goal is written, as it needs to
calculate what the bounds are for distance between feet. Users could do a
systematic parameter sweep to sample across a range of stride lengths.

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
    void setTargetAsymmetry(double asymmetry) {
        set_target_asymmetry(asymmetry);
    }
    double getTargetAsymmetry() { return get_target_asymmetry(); }

    /// TODO
    void setStrideLength(double length) {
        set_stride_length(length);
    }
    double getStrideLength() { return get_stride_length(); }

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


protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& input, double& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override;
    // void printDescriptionImpl() const override;

private:
    OpenSim_DECLARE_PROPERTY(left_foot_frame, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(right_foot_frame, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(target_asymmetry, double, "Default: 0.");
    OpenSim_DECLARE_PROPERTY(stride_length, double, "Default: 1.");
    OpenSim_DECLARE_PROPERTY(walking_direction, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(smoothing, double, "TODO");
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
