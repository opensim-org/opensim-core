#ifndef MUSCOLLO_MUCOMARKERTRACKINGCOST_H
#define MUSCOLLO_MUCOMARKERTRACKINGCOST_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoStateTrackingCost.h                                  *
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

namespace OpenSim {

/// The squared difference between a model marker location and an experimental
/// reference marker location, summed over the markers for which an 
/// experimental data location is provided, and integrated over the phase.
/// The reference can be provided as a file name to a TRC file, or 
/// programmatically as a TimeSeriesTable.







} // namespace OpenSim

#endif // MUSCOLLO_MUCOMARKERTRACKINGCOST_H