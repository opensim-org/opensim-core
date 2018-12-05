#ifndef MUSCOLLO_CASADITRAPEZOIDAL_H
#define MUSCOLLO_CASADITRAPEZOIDAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: CasADiTrapezoidal.h                                      *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2018 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s):                                                                 *
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

#include "CasADiTranscription.h"

class CasADiTrapezoidal : public CasADiTranscription {
public:
    using CasADiTranscription::CasADiTranscription;

    DM createIntegralQuadratureCoefficients(const DM& meshIntervals)
    const override {
        DM trapezoidalQuadCoeffs(m_numTimes, 1);
        trapezoidalQuadCoeffs(Slice(0, m_numTimes - 1)) = 0.5 * meshIntervals;
        trapezoidalQuadCoeffs(Slice(1, m_numTimes)) += 0.5 * meshIntervals;
        return trapezoidalQuadCoeffs;
    }

    void addDefectConstraintsImpl() override {

        auto h = m_duration / (m_numTimes - 1);
        m_dynamicsFunction =
                make_unique<DynamicsFunction>("dynamics", *this, m_probRep);

        // Defects.
        MX xdot_im1 = m_dynamicsFunction->operator()(
                {m_vars.initialTime,
                 m_vars.states(Slice(), 0),
                 m_vars.controls(Slice(), 0),
                 m_vars.parameters
                }).at(0);
        for (int itime = 1; itime < m_numTimes; ++itime) {
            const auto t = m_times(itime);
            auto x_i = m_vars.states(Slice(), itime);
            auto x_im1 = m_vars.states(Slice(), itime - 1);
            MX xdot_i = m_dynamicsFunction->operator()(
                    {t,
                     m_vars.states(Slice(), itime),
                     m_vars.controls(Slice(), itime),
                     m_vars.parameters}).at(0);
            m_opti.subject_to(x_i == (x_im1 + 0.5 * h * (xdot_i + xdot_im1)));
            xdot_im1 = xdot_i;
        }
    }
private:
    void createVariables() override {
        m_vars.initialTime = m_opti.variable();
        m_vars.finalTime = m_opti.variable();
        m_vars.states = m_opti.variable(m_numStates, m_numTimes);
        m_vars.controls = m_opti.variable(m_numControls, m_numTimes);
        m_vars.parameters = m_opti.variable(m_numParameters, 1);
        m_mesh = DM::linspace(0, 1, m_numTimes);
    }

};

#endif // MUSCOLLO_CASADITRAPEZOIDAL_H
