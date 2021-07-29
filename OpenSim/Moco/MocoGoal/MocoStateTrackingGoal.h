#ifndef OPENSIM_MOCOSTATETRACKINGGOAL_H
#define OPENSIM_MOCOSTATETRACKINGGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoStateTrackingGoal.h                                           *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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

#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Moco/MocoWeightSet.h>
#include <OpenSim/Simulation/TableProcessor.h>

namespace OpenSim {

// TODO allow raising error to different powers (cubed).
// TODO allow a "deadband."

/** 
\section MocoStateTrackingGoal
The squared difference between a state variable
value and a reference state variable value, summed over the state variables for which a
reference is provided, and integrated over the phase. This can be used to
track joint angles, activations, etc.
The reference can be provided as a file name to a STO or CSV file (or
other file types for which there is a FileAdapter), or programmatically
as a TimeSeriesTable. If columns for rotational coordinates are in degrees,
those columns will be converted to radians.
Tracking problems in direct collocation perform best when tracking smooth
data, so it is recommended to filter the data in the reference you provide
to the cost.

## Scale factors

Use `addScaleFactor()` to add a MocoParameter to the MocoProblem that will
scale the tracking reference data associated with a state in the tracking cost.
Scale factors for this goal can be useful if the magnitude of the tracking
reference data is either unknown or unreliable (e.g., pelvis height).
Scale factors are applied to the tracking error calculations based on the
following equation:

    error = modelValue - scaleFactor * referenceValue

In other words, scale factors are applied when computing the tracking error for
each state, not to the reference data directly.

Adding a scale factor to a MocoStateTrackingGoal.
@code
auto* stateTrackingGoal = problem.addGoal<MocoStateTrackingGoal>();
...
stateTrackingGoal->addScaleFactor(
        'pelvis_ty_scale_factor', '/jointset/ground_pelvis/pelvis_ty/value',
        {0.5, 2.0});
@endcode

@ingroup mocogoal */
class OSIMMOCO_API MocoStateTrackingGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoStateTrackingGoal, MocoGoal);

public:
    MocoStateTrackingGoal() { constructProperties(); }
    MocoStateTrackingGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoStateTrackingGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }
    /// Provide a table containing reference values for the
    /// states you want to track. Each column label must be the path of a state
    /// variable, e.g., `knee/flexion/value`.
    /// The table is not loaded until the MocoProblem is initialized.
    void setReference(TableProcessor ref) {
        set_reference(std::move(ref));
    }

    /// If no reference has been provided, this returns an empty processor.
    const TableProcessor& getReference() const { return get_reference(); }

    /// Set the weight for an individual state variable. If a weight is
    /// already set for the requested state, then the provided weight
    /// replaces the previous weight. An exception is thrown if a weight
    /// for an unknown state is provided.
    void setWeightForState(const std::string& stateName, const double& weight) {
        if (get_state_weights().contains(stateName)) {
            upd_state_weights().get(stateName).setWeight(weight);
        } else {
            upd_state_weights().cloneAndAppend({stateName, weight});
        }
    }

    /// Provide a MocoWeightSet to weight the state variables in the cost.
    /// Replaces the weight set if it already exists.
    void setWeightSet(const MocoWeightSet& weightSet) {
        upd_state_weights() = weightSet;
    }

    /// Only state paths matching the regular expression are tracked. The
    /// regular expression must match the entire state path for a state path to
    /// be tracked (that is, we use std::regex_match, not std::regex_search).
    /// To track only generalized coordinates, use `.*value$`.
    /// To track generalized coordinates and speeds, use `.*(value|speed)$`.
    /// To track only activations, use `.*activation$`.
    /// If the reference contains columns for states whose path does not match
    /// this pattern, you will get an error unless you use
    /// `setAllowUnusedReferences(true)`.
    void setPattern(std::string pattern) { set_pattern(pattern); }
    /// Unset the pattern, which causes all states to be matched.
    void clearPattern() { updProperty_pattern().clear(); }
    std::string getPattern() const { return get_pattern(); }

    /// Specify whether or not extra columns in the reference are allowed.
    /// If set true, the extra references will be ignored by the cost.
    /// If false, extra reference will cause an Exception to be raised.
    void setAllowUnusedReferences(bool tf) { set_allow_unused_references(tf); }

    /// Use the range, or the distance between the maximum and minimum value, of 
    /// each reference quantity to scale the weight for the associated tracking 
    /// error in the cost. The scale is computed by the inverse of the range, 
    /// so a reference quantity that changes less across the trajectory has a 
    /// larger weight. Each reference has a default weight of 1, so this flag
    /// works even if no user weights have be set. This may be useful when 
    /// tracking quantities with different units, which may have tracking errors 
    /// with different magnitudes.
    void setScaleWeightsWithRange(bool tf) { set_scale_weights_with_range(tf); }

    /// Add a MocoParameter to the problem that will scale the tracking reference
    /// data associated with the specified state. Scale factors are applied
    /// to the tracking error calculations based on the following equation:
    ///
    ///     error = modelValue - scaleFactor * referenceValue
    ///
    /// In other words, the scale factor is applied when computing the tracking
    /// error for each state, not to the reference data directly.
    void addScaleFactor(const std::string& name, const std::string& state,
                        const MocoBounds& bounds);

protected:
    // TODO check that the reference covers the entire possible time range.
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& input, SimTK::Real& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override {
        cost[0] = input.integral;
    }
    void printDescriptionImpl() const override;

private:
    // PROPERTIES
    OpenSim_DECLARE_PROPERTY(reference, TableProcessor,
            "Trajectories of states "
            "(coordinates, speeds, activation, etc.) to track. Column labels "
            "should be state variable paths, e.g., 'knee/flexion/value'");

    OpenSim_DECLARE_PROPERTY(state_weights, MocoWeightSet,
            "Set of weight objects to weight the tracking of individual "
            "state variables in the cost.");

    OpenSim_DECLARE_OPTIONAL_PROPERTY(pattern, std::string,
            "If provided, only states matching this regular expression are "
            "tracked (default: no pattern).");

    OpenSim_DECLARE_PROPERTY(allow_unused_references, bool,
            "Flag to determine whether or not references contained in the "
            "reference_file are allowed to be ignored by the cost.");

    OpenSim_DECLARE_PROPERTY(scale_weights_with_range, bool, 
            "Use the range, or the distance between the maximum and minimum "
            "value, of each reference quantity to scale the weight "
            "for the associated tracking error in the cost. The scale is "
            "computed by the inverse of the range, so a reference quantity "
            "that changes less across the trajectory has a larger weight. ");

    void constructProperties() {
        constructProperty_reference(TableProcessor());
        constructProperty_state_weights(MocoWeightSet());
        constructProperty_pattern();
        constructProperty_allow_unused_references(false);
        constructProperty_scale_weights_with_range(false);
    }

    mutable GCVSplineSet m_refsplines;
    /// The indices in Y corresponding to the provided reference coordinates.
    mutable std::vector<int> m_sysYIndices;
    mutable std::vector<double> m_state_weights;
    mutable std::vector<std::string> m_state_names;
    mutable std::unordered_map<std::string, std::string> m_scaleFactorMap;
    mutable std::vector<SimTK::ReferencePtr<const MocoScaleFactor>>
            m_scaleFactorRefs;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOSTATETRACKINGGOAL_H
