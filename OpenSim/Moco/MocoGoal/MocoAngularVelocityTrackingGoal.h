#ifndef OPENSIM_MOCOANGULARVELOCITYTRACKINGGOAL_H
#define OPENSIM_MOCOANGULARVELOCITYTRACKINGGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoAngularVelocityTrackingGoal.h                                 *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
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

#include "MocoGoal.h"

#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Moco/MocoWeightSet.h>
#include <OpenSim/Simulation/Model/Frame.h>
#include <OpenSim/Simulation/TableProcessor.h>

namespace OpenSim {

/** The squared difference between a model frame's angular velocity and a
reference angular velocity value, summed over the frames for which a
reference is provided, and integrated over the phase. This can be used to
track angular velocity quantities in the model that don't correspond to
model degrees of freedom. The reference can be provided as a trajectory of
SimTK::Vec3%s representing the angular velocity reference data, or as a
states trajectory from which the tracked angular velocity reference is
computed. Both angular velocity and states references can be provided as a
file name to a STO or CSV file (or other file types for which there is a
FileAdapter), or programmatically as a TimeSeriesTableVec3 (for the angular
velocity reference) or as a scalar TimeSeriesTable (for the states
reference).

Errors for this cost are computed assuming that the provided reference
angular velocity data is expressed in the ground frame. If you are using
this cost to track raw signals from an inertial measurement unit (IMU), make
sure that the frame you're tracking produces angular velocity values that
correspond to the real-world placement of your IMU.

This cost requires realization to SimTK::Stage::Velocity.

@ingroup mocogoal */
class OSIMMOCO_API MocoAngularVelocityTrackingGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoAngularVelocityTrackingGoal, MocoGoal);

public:
    MocoAngularVelocityTrackingGoal() { constructProperties(); }
    MocoAngularVelocityTrackingGoal(std::string name)
            : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoAngularVelocityTrackingGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }

    /** Set directly the angular velocities of individual frames in ground to be
    tracked in the cost. The column labels of the provided reference must
    be paths to frames in the model, e.g. `/bodyset/torso`. If the
    frame_paths property is empty, all frames with data in this reference
    will be tracked. Otherwise, only the frames specified via
    setFramePaths() will be tracked. Calling this function clears the values
    provided via setStatesReference(), setAngularVelocityReference(), or the
    `states_reference_file` property, if any. */
    void setAngularVelocityReferenceFile(const std::string& filepath) {
        set_states_reference(TableProcessor());
        m_angular_velocity_table = TimeSeriesTableVec3();
        set_angular_velocity_reference_file(filepath);
    }
    /** Each column label must be the path of a valid frame path (see
    setAngularVelocityReferenceFile()). Calling this function clears the
    `states_reference_file` and `angular_velocity_reference_file` properties
    or the table provided via setStatesReference(), if any. */
    void setAngularVelocityReference(const TimeSeriesTableVec3& ref) {
        set_states_reference(TableProcessor());
        set_angular_velocity_reference_file("");
        m_angular_velocity_table = ref;
    }
    /** Provide a table containing values of model state
    variables. These data are used to create a StatesTrajectory internally,
    from which the angular velocity data for the frames specified in
    setFramePaths() are computed. Each column label in the reference must be
    the path of a state variable, e.g., `/jointset/ankle_angle_r/value`.
    Calling this function clears the table provided via
    setAngularVelocityReference(), or the
    `angular_velocity_reference_file` property, if any. The table is not
    loaded until the MocoProblem is initialized. */
    void setStatesReference(const TableProcessor& ref) {
        set_angular_velocity_reference_file("");
        m_angular_velocity_table = TimeSeriesTableVec3();
        set_states_reference(std::move(ref));
    }
    /** Set the paths to frames in the model that this cost term will track. The
    names set here must correspond to OpenSim::Component%s that derive from
    OpenSim::Frame, which includes 'angular_velocity' (SimTK::Vec3) as an
    output. Replaces the frame path set if it already exists. */
    void setFramePaths(const std::vector<std::string>& paths) {
        updProperty_frame_paths().clear();
        for (const auto& path : paths) { append_frame_paths(path); }
    }
    /** Set the weight for an individual frame's angular velocity tracking. If a
    weight is already set for the requested frame, then the provided weight
    replaces the previous weight. An exception is thrown if a weight
    for an unknown frame is provided. */
    void setWeightForFrame(const std::string& frameName, const double& weight) {
        if (get_angular_velocity_weights().contains(frameName)) {
            upd_angular_velocity_weights().get(frameName).setWeight(weight);
        } else {
            upd_angular_velocity_weights().cloneAndAppend({frameName, weight});
        }
    }
    /** Provide a MocoWeightSet to weight frame angular velocity tracking in the
    cost. Replaces the weight set if it already exists. */
    void setWeightSet(const MocoWeightSet& weightSet) {
        upd_angular_velocity_weights() = weightSet;
    }
    /** If no states reference has been provided, this returns an empty
    processor. */
    const TableProcessor& getStatesReference() const {
        return get_states_reference();
    }
    /** If no angular velocity reference file has been provided, this returns an
    empty string. */
    std::string getAngularVelocityReferenceFile() const {
        return get_angular_velocity_reference_file();
    }

protected:
    void initializeOnModelImpl(const Model& model) const override;
    void calcIntegrandImpl(
            const IntegrandInput& input, double& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override {
        cost[0] = input.integral;
    }
    void printDescriptionImpl() const override;

private:
    OpenSim_DECLARE_PROPERTY(states_reference, TableProcessor,
            "Trajectories of model state variables from which tracked angular "
            "velocity data is computed. Column labels should be model state "
            "paths, e.g., '/jointset/ankle_angle_r/value'");
    OpenSim_DECLARE_PROPERTY(angular_velocity_reference_file, std::string,
            "Path to file (.sto, .csv, ...) containing angular velocity "
            "reference data to track. Column labels should be paths to frames "
            "in the model, e.g. '/bodyset/torso'.");
    OpenSim_DECLARE_LIST_PROPERTY(frame_paths, std::string,
            "The frames in the model that this cost term will track. "
            "The names set here must correspond to Components that "
            "derive from class Frame.");
    OpenSim_DECLARE_PROPERTY(angular_velocity_weights, MocoWeightSet,
            "Set of weight objects to weight the tracking of "
            "individual frames' angular velocities in the cost.");

    void constructProperties() {
        constructProperty_states_reference(TableProcessor());
        constructProperty_angular_velocity_reference_file("");
        constructProperty_frame_paths();
        constructProperty_angular_velocity_weights(MocoWeightSet());
    }

    TimeSeriesTableVec3 m_angular_velocity_table;
    mutable GCVSplineSet m_ref_splines;
    mutable std::vector<std::string> m_frame_paths;
    mutable std::vector<SimTK::ReferencePtr<const Frame>> m_model_frames;
    mutable std::vector<double> m_angular_velocity_weights;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOANGULARVELOCITYTRACKINGGOAL_H
