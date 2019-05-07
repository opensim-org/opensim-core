#ifndef MOCO_MOCOCONTROLTRACKINGCOST_H
#define MOCO_MOCOCONTROLTRACKINGCOST_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoControlTrackingCost.h                                    *
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

namespace OpenSim {

/// The squared difference between a control variable value and a reference
/// control variable value, summed over the control variables for which a
/// reference is provided, and integrated over the phase. This can be used to
/// track actuator controls, muscle excitations, etc.
/// The reference can be provided as a file name to a STO or CSV file (or
/// other file types for which there is a FileAdapter), or programmatically
/// as a TimeSeriesTable.
/// Tracking problems in direct collocation perform best when tracking smooth
/// data, so it is recommended to filter the data in the reference you provide
/// to the cost.
/// @ingroup mococost
class OSIMMOCO_API MocoControlTrackingCost : public MocoCost {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoControlTrackingCost, MocoCost);

public:
    MocoControlTrackingCost() { constructProperties(); };
    MocoControlTrackingCost(std::string name) : MocoCost(std::move(name)) {
        constructProperties();
    }
    MocoControlTrackingCost(std::string name, double weight)
        : MocoCost(std::move(name), weight) {
        constructProperties();
    }
    /// Provide the path to a data file containing reference values for the
    /// controls you want to track. Each column label must be the path of a 
    /// control variable, e.g., `/forceset/soleus_r`. If the column in the 
    /// reference is for a control variable associated with an non-scalar 
    /// actuator, the name of the variable in the path must include the index 
    /// for the actuator control, e.g., '/forceset/body_actuator_0', where 
    /// 'body_actuator' is the name of the actuator and '_0' specifies the 
    /// control index. Calling this function clears the table provided via 
    /// setReference(), if any. The file is not loaded until the MocoProblem 
    /// is initialized.
    // TODO path relative to working directory or setup file?
    void setReferenceFile(const std::string& filepath) {
        m_table = TimeSeriesTable();
        set_reference_file(filepath);
    }
    /// Each column label must be the path of a valid control variable (see
    /// setReferenceFile). Calling this function clears the `reference_file`
    /// property.
    void setReference(const TimeSeriesTable& ref) {
        set_reference_file("");
        m_table = ref;
    }
    /// Set the weight for an individual control variable. If a weight is
    /// already set for the requested control, then the provided weight
    /// replaces the previous weight. An exception is thrown if a weight
    /// for an unknown state is provided.
    void setWeight(const std::string& controlName, const double& weight) {
        if (get_control_weights().contains(controlName)) {
            upd_control_weights().get(controlName).setWeight(weight);
        } else {
            upd_control_weights().cloneAndAppend({controlName, weight});
        }
    }
    /// Provide a MocoWeightSet to weight the control variables in the cost.
    /// Replaces the weight set if it already exists.
    void setWeightSet(const MocoWeightSet& weightSet) {
        upd_control_weights() = weightSet;
    }

    /// If no reference file has been provided, this returns an empty string.
    std::string getReferenceFile() const { return get_reference_file(); }

    /// Specify whether or not extra columns in the reference are allowed.
    /// If set true, the extra references will be ignored by the cost.
    /// If false, extra references will cause an Exception to be raised.
    void setAllowUnusedReferences(bool tf) {
        set_allow_unused_references(tf);
    }

protected:
    // TODO check that the reference covers the entire possible time range.
    void initializeOnModelImpl(const Model& model) const override;
    void calcIntegralCostImpl(const SimTK::State& state,
        double& integrand) const override;

private:
    OpenSim_DECLARE_PROPERTY(reference_file, std::string,
        "Path to file (.sto, .csv, ...) containing values of controls "
        "(joint moments, excitations, etc.) to track. Column labels "
        "should be control variable paths, e.g., '/forceset/soleus_r'");

    OpenSim_DECLARE_PROPERTY(allow_unused_references, bool,
        "Flag to determine whether or not references contained in the "
        "reference_file are allowed to be ignored by the cost.");

    OpenSim_DECLARE_PROPERTY(control_weights, MocoWeightSet,
        "Set of weight objects to weight the tracking of individual "
        "control variables in the cost.");

    void constructProperties() {
        constructProperty_reference_file("");
        constructProperty_allow_unused_references(false);
        constructProperty_control_weights(MocoWeightSet());
    }

    TimeSeriesTable m_table;
    mutable GCVSplineSet m_refsplines;
    mutable std::vector<int> m_controlIndices;
    mutable std::vector<double> m_control_weights;
};

} // namespace OpenSim

#endif // MOCO_MOCOCONTROLTRACKINGCOST_H
