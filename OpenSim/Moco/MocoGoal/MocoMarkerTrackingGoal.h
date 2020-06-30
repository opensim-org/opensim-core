#ifndef OPENSIM_MOCOMARKERTRACKINGGOAL_H
#define OPENSIM_MOCOMARKERTRACKINGGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoMarkerTrackingGoal.h                                          *
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

#include "MocoGoal.h"

#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <OpenSim/Simulation/MarkersReference.h>

namespace OpenSim {

class Marker;

/** The squared difference between a model marker location and an experimental
reference marker location, summed over the markers for which an
experimental data location is provided, and integrated over the phase.
The reference can be provided as a file name to a TRC file, or
programmatically as a TimeSeriesTable.
@ingroup mocogoal */
class OSIMMOCO_API MocoMarkerTrackingGoal : public MocoGoal {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoMarkerTrackingGoal, MocoGoal);
public:
    MocoMarkerTrackingGoal() { constructProperties(); }
    MocoMarkerTrackingGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoMarkerTrackingGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }

    /** Provide a MarkersReference object containing the marker trajectories to
    be tracked by a model. The MarkersReferences can be created from a file
    of marker trajectories (e.g. .trc) or created programmatically via a
    TimeSeriesTableVec3. The marker weights property can be optionally
    specified to weight the tracking of individual markers in the cost
    function. Names of markers in the reference to be tracked should match
    the names of corresponding model markers. */
    void setMarkersReference(const MarkersReference& ref) {
        set_markers_reference(ref);
    }

    /** If no MarkersReference has been specified, this returns an empty
    MarkersReference object. */
    MarkersReference getMarkersReference() const {
        return get_markers_reference();
    }

    /** Specify if the markers_reference can contain marker data for a marker
    not in the model. An exception is raised if set to false and marker
    data exists for a marker not included in the model. */
    void setAllowUnusedReferences(bool tf) {
        set_allow_unused_references(tf);
    }

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& input, SimTK::Real& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override {
        cost[0] = input.integral;
    }
    void printDescriptionImpl() const override;

    OpenSim_DECLARE_PROPERTY(markers_reference, MarkersReference,
            "MarkersReference object containing the marker trajectories to be "
            "tracked by a model. Marker weights can be optionally specified "
            "to weight the tracking of individual markers in the cost "
            "function. Names of markers in the reference desired to be track "
            "should match the names of corresponding model markers.");

    OpenSim_DECLARE_PROPERTY(allow_unused_references, bool,
            "Allow markers_reference to contain marker data for a marker "
            "not in the model (such data would be ignored). Default: false.");

    mutable GCVSplineSet m_refsplines;
    mutable std::vector<SimTK::ReferencePtr<const Marker>> m_model_markers;
    mutable std::vector<int> m_refindices;
    mutable SimTK::Array_<double> m_marker_weights;
    mutable SimTK::Array_<std::string> m_marker_names;

private:
    void constructProperties() {
        constructProperty_markers_reference(MarkersReference());
        constructProperty_allow_unused_references(false);
    };

};

} // namespace OpenSim

#endif // OPENSIM_MOCOMARKERTRACKINGGOAL_H
