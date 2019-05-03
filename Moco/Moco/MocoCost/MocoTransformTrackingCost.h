#ifndef MOCO_MOCOTRANSFORMTRACKINGCOST_H
#define MOCO_MOCOTRANSFORMTRACKINGCOST_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoTransformTrackingCost.h                                  *
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

#include "MocoCost.h"

#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Common/GCVSplineSet.h>

// TODO: track multiple frames in one cost?
// TODO: track velocity-level quantities?
namespace OpenSim {

using SimTK::Transform;

class OSIMMOCO_API MocoTransformTrackingCost : public MocoCost {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoTransformTrackingCost, MocoCost);
public:
    MocoTransformTrackingCost() {
        constructProperties();
    }
    MocoTransformTrackingCost(std::string name) : MocoCost(std::move(name)) {
        constructProperties();
    }
    MocoTransformTrackingCost(std::string name, double weight)
            : MocoCost(std::move(name), weight) {
        constructProperties();
    }

    /// Set the path to the reference file containing values of model state
    /// variables. This data is used to create a StatesTrajectory internally,
    /// from which the transform data for the frame specified in setFramePath() 
    /// is computed. Each column label in the reference must be the path of a 
    /// state variable, e.g., `/jointset/ankle_angle_r/value`. Calling this 
    /// function clears the table provided via setReference(), if any. The file 
    /// is not loaded until the MocoProblem is initialized.
    void setStatesReferenceFile(const std::string& filepath) {
        m_states_table = TimeSeriesTable();
        m_transform_table = TimeSeriesTable_<Transform>();
        set_reference_file(filepath);
    }
    /// Each column label must be the path of a valid state variable (see
    /// setReferenceFile). Calling this function clears the `reference_file`
    /// property.
    void setStatesReference(const TimeSeriesTable& ref) {
        set_reference_file("");
        m_transform_table = TimeSeriesTable_<Transform>();
        m_states_table = ref;
    }
    // TODO
    void setTransformReference(const TimeSeriesTable_<Transform>& ref) {
        m_states_table = TimeSeriesTable();
        set_reference_file("");
        m_transform_table = ref;
    }
    /// Set the path to a frame in the model that this cost term will track. The 
    /// name set here must correspond to an OpenSim::Component that derives from 
    /// OpenSim::Frame, which includes SimTK::Transform as an output.
    void setFramePath(const std::string& path) {
        set_frame_path(path);
    }
    /// Set the tracking mode, which specifies the components of the transform
    /// to be tracked in the cost. Set to `rotation` to track only rotational
    /// data, set to `translation` to track only translational data, or set to
    /// `full` to track both rotations and translations. Default: `full`.
    void setTrackingMode(const std::string& mode) {
        set_tracking_mode(mode);
    }
    /// Set the weight on the rotation tracking error in the cost. 
    /// Default: 1.0.
    void setRotationTrackingWeight(double weight) {
        set_rotation_tracking_weight(weight);
    }
    /// Set the weight on the translation tracking error in the cost.
    /// Default: 1.0.
    void setTranslationTrackingWeight(double weight) {
        set_translation_tracking_weight(weight);
    }
    /// If no reference file has been provided, this returns an empty string.
    std::string getReferenceFile() const { return get_reference_file(); }
    /// Specify whether or not extra columns in the reference are allowed.
    /// If set true, the extra references will be ignored when computing 
    /// transform internally. If false, extra references will cause an Exception 
    /// to be raised.
    void setAllowUnusedReferences(bool tf) {
        set_allow_unused_references(tf);
    }

protected:
    void initializeOnModelImpl(const Model& model) const override;
    void calcIntegralCostImpl(const SimTK::State& state,
        double& integrand) const override;

private:
    OpenSim_DECLARE_PROPERTY(reference_file, std::string,
            "Path to file (.sto, .csv, ...) containing values of model state "
            "variables from which tracked transform data is computed. Column "
            "labels should be model state paths, "
            "e.g., '/jointset/ankle_angle_r/value'");
    OpenSim_DECLARE_PROPERTY(allow_unused_references, bool,
            "Flag to determine whether or not references contained in the "
            "reference_file are allowed to be ignored when computing the "
            "transform data to be tracked. Default: false.");
    OpenSim_DECLARE_PROPERTY(frame_path, std::string,
            "The frame in the model that this cost term will track. "
            "The name set here must correspond to an OpenSim::Component that "
            "derives from class OpenSim::Frame, which includes "
            "SimTK::Transform as an output.")
    OpenSim_DECLARE_PROPERTY(tracking_mode, std::string,
            "Specify which components of the transform to be tracked in the "
            "cost. Set to 'rotation' to track only rotational data, set to "
            "'translation' to track only translational data, or set to 'full' "
            "to track both rotations and translations. Default: 'full'.");
    OpenSim_DECLARE_PROPERTY(rotation_tracking_weight, double,
            "The weight on the rotation tracking error in the cost. "
            "Default: 1.0.")
    OpenSim_DECLARE_PROPERTY(translation_tracking_weight, double,
            "The weight on the translation tracking error in the cost. "
            "Default: 1.0.")

    void constructProperties() {
        constructProperty_reference_file("");
        constructProperty_allow_unused_references(false);
        constructProperty_frame_path("");
        constructProperty_tracking_mode("full");
        constructProperty_rotation_tracking_weight(1.0);
        constructProperty_translation_tracking_weight(1.0);
    }

    // TODO
    GCVSplineSet createReferenceSplines(
        const TimeSeriesTable_<Transform>& table);

    TimeSeriesTable m_states_table;
    TimeSeriesTable_<Transform> m_transform_table;
    mutable GCVSplineSet m_ref_splines;
    mutable int m_tracking_mode;
};

} // namespace OpenSim

#endif // MOCO_MOCOTRANSFORMTRACKINGCOST_H
