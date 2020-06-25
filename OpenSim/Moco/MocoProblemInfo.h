#ifndef OPENSIM_MOCOPROBLEMINFO_H
#define OPENSIM_MOCOPROBLEMINFO_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoProblemInfo.h                                                 *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
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

namespace OpenSim {

/// This class is mostly for internal use for MocoProblemRep to pass select
/// information about a problem to the MocoGoal%s and MocoPathConstraint%s of
/// the problem during initializeOnModel().
class MocoProblemInfo {
public:
    double minInitialTime;
    double maxFinalTime;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOPROBLEMINFO_H
