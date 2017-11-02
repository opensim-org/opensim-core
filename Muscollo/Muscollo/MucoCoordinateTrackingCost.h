#ifndef MUSCOLLO_MUCOCOORDINATETRACKINGCOST_H
#define MUSCOLLO_MUCOCOORDINATETRACKINGCOST_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoCoordinateTrackingCost.h                             *
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

namespace OpenSim {

// TODO could implement as a very general "state tracker", could even track
// desired activation signals.
// TODO can we track generailzed speeds too?
// TODO weights for each coordinate.
class MucoCoordinateTrackingCost : public MucoCost {
OpenSim_DECLARE_CONCRETE_OBJECT(MucoCoordinateTrackingCost, MucoCost);
public:
    void setReference(const TimeSeriesTable& ref) {
        m_table = ref;
    }
protected:
    void initializeImpl() const override;
    void calcIntegralCostImpl(const SimTK::State& state,
            double& integrand) const override;
private:
    // TODO accept a filename.
    //OpenSim_DECLARE_PROPERTY(coordinates_file, std::string, "TODO");
    TimeSeriesTable m_table;
    mutable GCVSplineSet m_refsplines;
    /// The indices in Q corresponding to the provided reference coordinates.
    mutable std::vector<int> m_sysYIndices;
};

} // namespace OpenSim

#endif // MUSCOLLO_MUCOCOORDINATETRACKINGCOST_H
