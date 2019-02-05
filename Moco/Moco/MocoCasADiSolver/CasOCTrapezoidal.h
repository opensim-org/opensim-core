#ifndef MOCO_CASOCTRAPEZOIDAL_H
#define MOCO_CASOCTRAPEZOIDAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: CasOCTrapezoidal.h                                           *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2018 Stanford University and the Authors                     *
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

#include "CasOCTranscription.h"

namespace CasOC {

/// Enforce the differential equations in the problem using a trapezoidal
/// (second-order) approximation. The integral in the objective function is
/// approximated by trapezoidal quadrature.
class Trapezoidal : public Transcription {
public:
    Trapezoidal(const Solver& solver, const Problem& problem);

private:
    Iterate createInitialGuessFromBoundsImpl() const override;
    Iterate createRandomIterateWithinBoundsImpl() const override;
    casadi::DM createTimesImpl(
            casadi::DM initialTime, casadi::DM finalTime) const override {
        return createTimes<casadi::DM>(initialTime, finalTime);
    }

    template <typename T>
    T createTimes(const T& initialTime, const T& finalTime) const {
        return (finalTime - initialTime) * m_mesh + initialTime;
    }
    casadi::DM m_mesh;
    casadi::MX m_times;
    casadi::MX m_duration;
};

} // namespace CasOC

#endif // MOCO_CASOCTRAPEZOIDAL_H
