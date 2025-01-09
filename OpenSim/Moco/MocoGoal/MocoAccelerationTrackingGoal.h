#ifndef OPENSIM_MOCOACCELERATIONTRACKINGGOAL_H
#define OPENSIM_MOCOACCELERATIONTRACKINGGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoAccelerationTrackingGoal.h                                    *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2021 Stanford University and the Authors                     *
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

#include <OpenSim/Moco/MocoWeightSet.h>
#include "MocoGoal.h"
#include "OpenSim/Simulation/TableProcessor.h"

#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Simulation/Model/Frame.h>

namespace OpenSim {

/** The squared difference between a model frame origin's linear acceleration
and a reference acceleration value, summed over the frames for which a
reference is provided, and integrated over the phase. The reference is
a trajectory of SimTK::Vec3%s representing the acceleration reference data.
You must provide either a file name to a STO or CSV file (or other file
types for which there is a FileAdapter) using setAccelerationReferenceFile() or
a TimeSeriesTableVec3 directly using setAccelerationReference(). The model frames
that track the acceleration reference data, or "tracking" frames, are specified
using setFramePaths().

\section MocoAccelerationTrackingGoal
\subsection use_MocoAccelerationTrackingGoal Default Usage

By default, errors for this cost are computed assuming that the provided
reference acceleration data is the derivative of a position vector with respect
to the ground frame and expressed in the ground frame.

\subsection imu_MocoAccelerationTrackingGoal Tracking inertial measurement unit (IMU) signals

If using this cost for tracking acceleration signals from an inertial measurement
unit (IMU), you must subtract the gravity vector from the model accelerations and
express them in the tracking frames using setGravityOffset() and
setExpressAccelerationsInTrackingFrames(). In addition, the tracking frames for
each IMU must be in the same location and orientation in the model as in your
experiment. Therefore, it is recommended to add new frames to your model that
represent the experimental IMU sensors when using these settings.

This cost requires realization to SimTK::Stage::Acceleration.

@ingroup mocogoal */
class OSIMMOCO_API MocoAccelerationTrackingGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoAccelerationTrackingGoal, MocoGoal);

public:
    MocoAccelerationTrackingGoal() { constructProperties(); }
    MocoAccelerationTrackingGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoAccelerationTrackingGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }

    /** Set the acceleration of individual frames in ground to be tracked in the
    cost. The column labels of the provided reference must be paths to
    frames in the model, e.g. `/bodyset/torso`. If the frame_paths property
    is empty, all frames with data in this reference will be tracked.
    Otherwise, only the frames specified via setFramePaths() will be
    tracked. Calling this function clears the table set by
    setAccelerationReference() if it exists. */
    void setAccelerationReferenceFile(const std::string& filepath) {
        m_acceleration_table = TimeSeriesTableVec3();
        set_acceleration_reference_file(filepath);
    }
    /** Each column label must be the path of a valid frame path (see
    setAccelerationReferenceFile()). Calling this function clears the
    `acceleration_reference_file` property. */
    void setAccelerationReference(const TimeSeriesTableVec3& ref) {
        set_acceleration_reference_file("");
        m_acceleration_table = ref;
    }
    /** Set the paths to frames in the model that this cost term will track. The
    names set here must correspond to OpenSim::Component%s that derive from
    OpenSim::Frame, which includes 'linear_acceleration' (SimTK::Vec3) as an
    output. Replaces the frame path set if it already exists. */
    void setFramePaths(const std::vector<std::string>& paths) {
        updProperty_frame_paths().clear();
        for (const auto& path : paths) { append_frame_paths(path); }
    }
    /** Set the weight for an individual frame's acceleration tracking. If a
    weight is already set for the requested frame, then the provided weight
    replaces the previous weight. An exception is thrown if a weight
    for an unknown frame is provided. */
    void setWeightForFrame(const std::string& frameName, const double& weight) {
        if (get_acceleration_weights().contains(frameName)) {
            upd_acceleration_weights().get(frameName).setWeight(weight);
        } else {
            upd_acceleration_weights().cloneAndAppend({frameName, weight});
        }
    }
    /** Provide a MocoWeightSet to weight frame acceleration tracking in the
    cost. Replaces the weight set if it already exists. */
    void setWeightSet(const MocoWeightSet& weightSet) {
        upd_acceleration_weights() = weightSet;
    }
    /** If no acceleration reference file has been provided, this returns an
    empty string. */
    std::string getAccelerationReferenceFile() const {
        return get_acceleration_reference_file();
    }

    /** Subtract the model's gravity vector from the model-generated
    accelerations. This offset is performed while the model accelerations are
    expressed in the ground frame, before the accelerations are expressed in the
    tracking frames when setExpressAccelerationsInTrackingFrames() is set to
    true. This setting is useful when tracking accelerations recorded from
    inertial measurement units. */
    void setGravityOffset(bool tf) { set_gravity_offset(tf); }
    bool getGravityOffset() { return get_gravity_offset(); }

    /** Express the model-generated accelerations in the individual model
    tracking frames specified by the 'frame_paths' property. Accelerations are
    expressed after the gravity offset is applied when setGravityOffset() is set
    to true. This setting is useful when tracking accelerations recorded from
    inertial measurement units. */
    void setExpressAccelerationsInTrackingFrames(bool tf) {
        set_express_accelerations_in_tracking_frames(tf);
    }
    bool getExpressAccelerationsInTrackingFrames() {
        return get_express_accelerations_in_tracking_frames();
    }

protected:
    void initializeOnModelImpl(const Model& model) const override;
    void calcIntegrandImpl(
            const IntegrandInput& input, double& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& goal) const override {
            goal[0] = input.integral;
    }
    void printDescriptionImpl() const override;

private:
    OpenSim_DECLARE_PROPERTY(acceleration_reference_file, std::string,
            "Path to file (.sto, .csv, ...) containing acceleration reference "
            "data to track. Column labels should be paths to frames in the "
            "model, e.g. '/bodyset/torso'.");
    OpenSim_DECLARE_LIST_PROPERTY(frame_paths, std::string,
            "The frames in the model (i.e., tracking frames) that this cost "
            "term will track. The names set here must correspond to Components "
            "that derive from class Frame.");
    OpenSim_DECLARE_PROPERTY(acceleration_weights, MocoWeightSet,
            "Set of weight objects to weight the tracking of "
            "individual frames' accelerations in the cost.");
    OpenSim_DECLARE_PROPERTY(gravity_offset, bool,
            "Subtract the model's gravity vector from the model tracking frame "
            "accelerations. This offset is performed while the model "
            "accelerations are expressed in the ground frame, before the "
            "accelerations are expressed in the tracking frames when the "
            "'express_accelerations_in_tracking_frames' property is set to "
            "true.");
    OpenSim_DECLARE_PROPERTY(express_accelerations_in_tracking_frames, bool,
            "Express the model-generated accelerations in the individual model "
            "tracking frames specified by the 'frame_paths' property. "
            "Accelerations are expressed after the gravity offset is applied "
            "when 'gravity_offset' is set to true.");

    void constructProperties() {
        constructProperty_acceleration_reference_file("");
        constructProperty_frame_paths();
        constructProperty_acceleration_weights(MocoWeightSet());
        constructProperty_gravity_offset(false);
        constructProperty_express_accelerations_in_tracking_frames(false);
    }

    TimeSeriesTableVec3 m_acceleration_table;
    mutable GCVSplineSet m_ref_splines;
    mutable std::vector<std::string> m_frame_paths;
    mutable std::vector<SimTK::ReferencePtr<const Frame>> m_model_frames;
    mutable std::vector<double> m_acceleration_weights;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOACCELERATIONTRACKINGGOAL_H
