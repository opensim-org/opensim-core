#ifndef MUSCOLLO_MUSCOLLOUTILITIES_H
#define MUSCOLLO_MUSCOLLOUTILITIES_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MuscolloUtilities.h                                               *
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

#include <OpenSim/Common/Storage.h>

#include "osimMuscolloDLL.h"

namespace OpenSim {

class StatesTrajectory;

// TODO move to the Storage class.
OSIMMUSCOLLO_API Storage convertTableToStorage(const TimeSeriesTable& table);

OSIMMUSCOLLO_API void visualize(const StatesTrajectory& st);

} // namespace OpenSim

#endif // MUSCOLLO_MUSCOLLOUTILITIES_H
