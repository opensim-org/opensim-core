#ifndef OPENSIM_MOCOSTUDYUTILITIES_H
#define OPENSIM_MOCOSTUDYUTILITIES_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoStudyFactory.h                                                *
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

#include "MocoStudy.h"

namespace OpenSim {

/// This class creates MocoStudies of common interest.
class OSIMMOCO_API MocoStudyFactory {
public:
    /// In the "linear tangent steering" problem, we control the direction to
    /// apply a constant thrust to a point mass to move the mass a given
    /// vertical distance and maximize its final horizontal speed.
    /// This function defines internal classes: a DirectionActuator, and a
    /// LinearTangentFinalSpeed. This function is intended for use in testing.
    /// This problem has an analytical solution, and
    /// is described in Section 2.4 of Bryson and Ho [1]. Bryson, A. E., Ho,
    /// Y.C., Applied Optimal Control, Optimization, Estimation, and Control.
    /// New York London Sydney Toronto. John Wiley & Sons. 1975.
    static MocoStudy createLinearTangentSteeringStudy(
            double acceleration, double finalTime, double finalHeight);
};

} // namespace OpenSim

#endif // OPENSIM_MOCOSTUDYUTILITIES_H
