#ifndef MOCO_MOCOTRANSLATIONTRACKINGCOST_H
#define MOCO_MOCOTRANSLATIONTRACKINGCOST_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoTranslationTrackingCost.h                                *
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
#include "../MocoWeightSet.h"

#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Simulation/Model/Frame.h>

/// The squared difference between a model frame's origin position and a 
/// reference position value, summed over the frames for which a reference is 
/// provided, and integrated over the phase. This can be used to track position 
/// quantities in the model that don't correspond to model degrees of freedom.
/// The reference can be provided as a trajectory of SimTK::Vec3%s 
/// representing the translation reference data, or as a model-compatible states 
/// trajectory from which the tracked translation reference is computed. Both 
/// translation and states references can be provided as a file name to a STO or 
/// CSV file (or other file types for which there is a FileAdapter), or 
/// programmatically as a TimeSeriesTableVec3 (for the translation reference) 
/// or as a scalar TimeSeriesTable (for the states reference). 
/// 
/// Technically, a cost function with the same effect could be achieved with the
/// MocoMarkerTrackingCost class. However, this class avoids the need for adding
/// markers to the frame origins and provides the convenient
/// setStatesReference() and setStatesReferenceFile() methods which let the user
/// set up a tracking cost given only a states trajectory.
/// 
/// This cost requires realization to SimTK::Stage::Position.
///
/// Tracking problems in direct collocation perform best when tracking smooth
/// data, so it is recommended to filter the data in the reference you provide
/// to the cost.
/// @ingroup mococost
namespace OpenSim {

class OSIMMOCO_API MocoTranslationTrackingCost : public MocoCost {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoTranslationTrackingCost, MocoCost);
public:
    MocoTranslationTrackingCost() {
        constructProperties();
    }
    MocoTranslationTrackingCost(std::string name) : MocoCost(std::move(name)) {
        constructProperties();
    }
    MocoTranslationTrackingCost(std::string name, double weight)
        : MocoCost(std::move(name), weight) {
        constructProperties();
    }

    /// Set directly the translation of individual frames in ground to be 
    /// tracked in the cost. The column labels of the provided reference must 
    /// be paths to frames in the model, e.g. `/bodyset/torso`. If the 
    /// frame_paths property is empty, all frames with data in this reference 
    /// will be tracked. Otherwise, only the frames specified via
    /// setFramePaths() will be tracked. Calling this function clears the values 
    /// provided via setStatesReference(), setTranslationReference(), or the 
    /// `states_reference_file` property, if any.
    void setTranslationReferenceFile(const std::string& filepath) {
        set_states_reference_file("");
        m_states_table = TimeSeriesTable();
        m_translation_table = TimeSeriesTableVec3();
        set_translation_reference_file(filepath);
    }
    /// Each column label must be the path of a valid frame path (see
    /// setTranslationReferenceFile()). Calling this function clears the 
    /// `states_reference_file` and `translation_reference_file` properties or 
    /// the table provided via setStatesReference(), if any. 
    void setTranslationReference(const TimeSeriesTableVec3& ref) {
        set_states_reference_file("");
        m_states_table = TimeSeriesTable();
        set_translation_reference_file("");
        m_translation_table = ref;
    }
    /// Set the path to the reference file containing values of model state
    /// variables. These data are used to create a StatesTrajectory internally,
    /// from which the translation data for the frames specified in 
    /// setFramePaths() are computed. Each column label in the reference must be 
    /// the path of a state variable, e.g., `/jointset/ankle_angle_r/value`. 
    /// Calling this function clears the table provided via 
    /// setStatesReference(), setTranslationReference(), or the 
    /// `translation_reference_file` property, if any. The file is not loaded 
    /// until the MocoProblem is initialized.
    void setStatesReferenceFile(const std::string& filepath) {
        m_states_table = TimeSeriesTable();
        set_translation_reference_file("");
        m_translation_table = TimeSeriesTableVec3();
        set_states_reference_file(filepath);
    }
    /// Each column label must be the path of a valid state variable (see
    /// setStatesReferenceFile()). Calling this function clears the 
    /// `states_reference_file` and `translation_reference_file` properties or 
    /// the table provided via setTranslationReference(), if any.
    void setStatesReference(const TimeSeriesTable& ref) {
        set_states_reference_file("");
        set_translation_reference_file("");
        m_translation_table = TimeSeriesTableVec3();
        m_states_table = ref;
    }
    /// Set the paths to frames in the model that this cost term will track. The 
    /// names set here must correspond to OpenSim::Component%s that derive from 
    /// OpenSim::Frame, which includes 'position' (SimTK::Vec3) as an output.
    /// Replaces the frame path set if it already exists.
    void setFramePaths(const std::vector<std::string>& paths) {
        updProperty_frame_paths().clear();
        for (const auto& path : paths) {
            append_frame_paths(path);
        }
    }
    /// Set the weight for an individual frame's translation tracking. If a 
    /// weight is already set for the requested frame, then the provided weight
    /// replaces the previous weight. An exception is thrown if a weight
    /// for an unknown frame is provided.
    void setWeight(const std::string& frameName, const double& weight) {
        if (get_translation_weights().contains(frameName)) {
            upd_translation_weights().get(frameName).setWeight(weight);
        } else {
            upd_translation_weights().cloneAndAppend({frameName, weight});
        }
    }
    /// Provide a MocoWeightSet to weight frame translation tracking in the 
    /// cost. Replaces the weight set if it already exists.
    void setWeightSet(const MocoWeightSet& weightSet) {
        upd_translation_weights() = weightSet;
    }
    /// If no states reference file has been provided, this returns an empty 
    /// string.
    std::string getStatesReferenceFile() const 
    { return get_states_reference_file(); }
    /// If no translation reference file has been provided, this returns an 
    /// empty string.
    std::string getTranslationReferenceFile() const
    { return get_translation_reference_file(); }

protected:
    void initializeOnModelImpl(const Model& model) const override;
    void calcIntegralCostImpl(const SimTK::State& state,
        double& integrand) const override;

private:
    OpenSim_DECLARE_PROPERTY(states_reference_file, std::string,
            "Path to file (.sto, .csv, ...) containing values of model state "
            "variables from which tracked translation data is computed. Column "
            "labels should be model state paths, "
            "e.g., '/jointset/ankle_angle_r/value'");
    OpenSim_DECLARE_PROPERTY(translation_reference_file, std::string,
            "Path to file (.sto, .csv, ...) containing translation reference "
            "data to track. Column labels should be paths to frames in the "
            "model, e.g. `/bodyset/torso`.")
    OpenSim_DECLARE_LIST_PROPERTY(frame_paths, std::string,
            "The frames in the model that this cost term will track. "
            "The names set here must correspond to Components that "
            "derive from class OpenSim::Frame, which includes 'position' "
            "(Vec3) as an output.")
    OpenSim_DECLARE_PROPERTY(translation_weights, MocoWeightSet,
            "Set of weight objects to weight the tracking of individual "
            "frames' translations in the cost.");

    void constructProperties() {
        constructProperty_states_reference_file("");
        constructProperty_translation_reference_file("");
        constructProperty_frame_paths();
        constructProperty_translation_weights(MocoWeightSet());
    }

    TimeSeriesTable m_states_table;
    TimeSeriesTableVec3 m_translation_table;
    mutable GCVSplineSet m_ref_splines;
    mutable std::vector<SimTK::ReferencePtr<const Frame>> m_model_frames;
    mutable std::vector<double> m_translation_weights;
};

} // namespace OpenSim

#endif // MOCO_MOCOTRANSLATIONTRACKINGCOST_H