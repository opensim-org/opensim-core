/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: CasADiTrapezoidal.cpp                                    *
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
#include "CasADiTrapezoidal.h"

void CasADiTrapezoidal::addDefectConstraintsImpl() {

    m_dynamicsFunction =
            make_unique<DynamicsFunction>("dynamics", *this, m_probRep);

    // Compute qdot symbolically.
    OPENSIM_THROW_IF(m_state.getNQ() != m_state.getNU(), Exception);
    const int NQ = m_state.getNQ();

    // Defects.
    const auto& states = m_vars[Var::states];
    MX qdot = states(Slice(NQ, 2 * NQ), 0);
    MX udotzdot = m_dynamicsFunction->operator()(
            {m_vars[Var::initial_time],
             states(Slice(), 0),
             m_vars[Var::controls](Slice(), 0),
             m_vars[Var::parameters]
            }).at(0);
    MX xdot_im1 = casadi::MX::vertcat({qdot, udotzdot});
    for (int itime = 1; itime < m_numTimes; ++itime) {
        auto h = m_times(itime) - m_times(itime - 1);
        auto x_i = states(Slice(), itime);
        auto x_im1 = states(Slice(), itime - 1);
        qdot = states(Slice(NQ, 2 * NQ), itime);
        udotzdot = m_dynamicsFunction->operator()(
                {m_times(itime),
                 m_vars[Var::states](Slice(), itime),
                 m_vars[Var::controls](Slice(), itime),
                 m_vars[Var::parameters]}).at(0);
        MX xdot_i = MX::vertcat({qdot, udotzdot});
        m_opti.subject_to(x_i == (x_im1 + 0.5 * h * (xdot_i + xdot_im1)));
        xdot_im1 = xdot_i;
    }
}

casadi::Sparsity
CasADiTrapezoidal::DynamicsFunction::get_sparsity_out(casadi_int i) {
    if (i == 0) {
        int numRows = m_transcrip.m_state.getNU() + m_transcrip.m_state.getNZ();
        return casadi::Sparsity::dense(numRows, 1);
    }
    else return casadi::Sparsity(0, 0);
}
int CasADiTrapezoidal::DynamicsFunction::eval(
        const double** inputs, double** outputs,
        casadi_int*, double*, void*) const {
    m_transcrip.applyParametersToModel(
            SimTK::Vector(p.getNumParameters(), inputs[3], true));
    auto& state = m_transcrip.m_state;
    convertToSimTKState(inputs[0], inputs[1], inputs[2], p.getModel(), state);
    p.getModel().realizeAcceleration(state);

    // TODO create member variable for numRowsOut.
    int numRowsOut = state.getNU() + state.getNZ();
    std::copy_n(state.getYDot().getContiguousScalarData() + state.getNQ(),
            numRowsOut, outputs[0]);
    return 0;
}
