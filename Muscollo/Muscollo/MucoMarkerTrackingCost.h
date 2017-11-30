#ifndef MUSCOLLO_MUCOMARKERTRACKINGCOST_H
#define MUSCOLLO_MUCOMARKERTRACKINGCOST_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoMarkerTrackingCost.h                                 *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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

#include "MucoCost.h"

#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Simulation/MarkersReference.h>

namespace OpenSim {

class Marker;

/// The squared difference between a model marker location and an experimental
/// reference marker location, summed over the markers for which an 
/// experimental data location is provided, and integrated over the phase.
/// The reference can be provided as a file name to a TRC file, or 
/// programmatically as a TimeSeriesTable.
class OSIMMUSCOLLO_API MucoMarkerTrackingCost : public MucoCost {
OpenSim_DECLARE_CONCRETE_OBJECT(MucoMarkerTrackingCost, MucoCost);
public:
    MucoMarkerTrackingCost() { constructProperties(); }

    /// Provide a MarkersReference object containing the marker trajectories to 
    /// be tracked by a model. The MarkersReferences can be created from a file 
    /// of marker trajectories (e.g. .trc) or created programmatically via a 
    /// TimeSeriesTableVec3. The marker weights property can be optionally 
    /// specified to weight the tracking of individual markers in the cost 
    /// function. Names of markers in the reference to be tracked should match 
    /// the names of corresponding model markers.
    void setMarkersReference(const MarkersReference& ref) {
        set_markers_reference(ref);
    }

    /// If no MarkersReference has been specified, this returns an empty
    /// MarkersReference object.
    MarkersReference getMarkersReference() const {
        return get_markers_reference();
    }

    /// Specify whether or not extra marker references are allowed.
    /// If set true, the extra references will be ignored by the cost.
    /// If false, extra reference will cause an Exception to be raised.
    void setAllowUnusedReferences(bool tf) {
        set_allow_unused_references(tf);
    }

protected:
    void initializeImpl() const override;
    void calcIntegralCostImpl(const SimTK::State& state,
        double& integrand) const override;
private:
    OpenSim_DECLARE_PROPERTY(markers_reference, MarkersReference,
            "MarkersReference object containing the marker trajectories to be "
            "tracked by a model. Marker weights can be optionally specified "
            "to weight the tracking of individual markers in the cost "
            "function. Names of markers in the reference desired to be track " 
            "should match the names of corresponding model markers.");

    OpenSim_DECLARE_PROPERTY(allow_unused_references, bool,
        "Flag to determine whether or not references contained in the "
        "markers_reference are allowed to be ignored by the cost.");

    void constructProperties() {
        constructProperty_markers_reference(MarkersReference());
        constructProperty_allow_unused_references(false);
    };

    mutable GCVSplineSet m_refsplines;
    mutable std::vector<SimTK::ReferencePtr<const Marker>> m_model_markers;
    mutable std::vector<int> m_refindices;
    mutable SimTK::Array_<double> m_marker_weights;
};

} // namespace OpenSim

#endif // MUSCOLLO_MUCOMARKERTRACKINGCOST_H