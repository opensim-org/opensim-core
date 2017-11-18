#ifndef MUSCOLLO_MUCOSTATETRACKINGCOST_H
#define MUSCOLLO_MUCOSTATETRACKINGCOST_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoStateTrackingCost.h                                  *
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

#include "MucoCost.h"

#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Simulation/MarkersReference.h>

namespace OpenSim {

// TODO can we track generailzed speeds too?
// TODO weights for each state.
// TODO allow raising error to different powers (cubed).
// TODO allow a "deadband."

/// The squared difference between a state variable value and a reference
/// state variable value, summed over the state variables for which a
/// reference is provided, and integrated over the phase. This can be used to
/// track joint angles, activations, etc.
/// The reference can be provided as a file name to a STO or CSV file (or
/// other file types for which there is a FileAdapter), or programmatically
/// as a TimeSeriesTable.
class OSIMMUSCOLLO_API MucoStateTrackingCost : public MucoCost {
OpenSim_DECLARE_CONCRETE_OBJECT(MucoStateTrackingCost, MucoCost);
public:
    MucoStateTrackingCost() { constructProperties(); }
    /// Provide the path to a data file containing reference values for the
    /// states you want to track. Each column label must be the path of a state
    /// variable, e.g., `knee/flexion/value`. Calling this function clears the
    /// table provided via setReference(), if any.
    /// The file is not loaded until the MucoProblem is initialized.
    // TODO path relative to working directory or setup file?
    void setReferenceFile(const std::string& filepath) {
        m_table = TimeSeriesTable();
        set_reference_file(filepath);
    }
    /// Each column label must be the path of a state variable, e.g.,
    /// `knee/flexion/value`. Calling this function clears the `reference_file`
    /// property.
    void setReference(const TimeSeriesTable& ref) {
        set_reference_file("");
        m_table = ref;
    }

    /// Set the weight for an individual state variable. 
    void setWeight(const std::string& stateName, const double& weight) {
        if (get_state_weights().contains(stateName)) {
            upd_state_weights().get(stateName).setWeight(weight);
        } else {
            upd_state_weights().cloneAndAppend({stateName, weight});
        }
    }

    // TODO: void setWeightSet() 

    /// If no reference file has been provided, this returns an empty string.
    std::string getReferenceFile() const { return get_reference_file(); }

    /// Specify whether or not extra columns in the reference are allowed.
    /// If set true, the extra references will be ignored by the cost.
    /// If false, extra reference will cause an Exception to be raised.
    void allowUnusedReferences(bool tf) {
        set_allow_unused_refs(tf);
    }

protected:
    // TODO check that the reference covers the entire possible time range.
    void initializeImpl() const override;
    void calcIntegralCostImpl(const SimTK::State& state,
            double& integrand) const override;
private:
    OpenSim_DECLARE_PROPERTY(reference_file, std::string,
            "Path to file (.sto, .csv, ...) containing values of states "
            "(coordinates, speeds, activation, etc.) to track. Column labels "
            "should be state variable paths, e.g., 'knee/flexion/value'");

    OpenSim_DECLARE_PROPERTY(allow_unused_refs, bool,
            "Flag to determine whether or not references contained in the "
            "reference_file are allowed to be ignored by the cost.");

    OpenSim_DECLARE_PROPERTY(state_weights, Set<MarkerWeight>, 
            "Set of weight objects to weight the tracking of individual "
            "state variables in the cost.");

    void constructProperties() {
        constructProperty_reference_file("");
        constructProperty_allow_unused_refs(false);
        constructProperty_state_weights(Set<MarkerWeight>());
    }

    TimeSeriesTable m_table;
    mutable GCVSplineSet m_refsplines;
    /// The indices in Y corresponding to the provided reference coordinates.
    mutable std::vector<int> m_sysYIndices;
    mutable std::vector<double> m_state_weights;

};

} // namespace OpenSim

#endif // MUSCOLLO_MUCOSTATETRACKINGCOST_H
