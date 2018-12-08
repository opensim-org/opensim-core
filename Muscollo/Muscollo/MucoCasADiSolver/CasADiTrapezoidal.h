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

    class DynamicsFunction;

    using CasADiTranscription::CasADiTranscription;

    DM createIntegralQuadratureCoefficients(const DM& meshIntervals)
    const override {
        DM trapezoidalQuadCoeffs(m_numTimes, 1);
        trapezoidalQuadCoeffs(Slice(0, m_numTimes - 1)) = 0.5 * meshIntervals;
        trapezoidalQuadCoeffs(Slice(1, m_numTimes)) += 0.5 * meshIntervals;
        return trapezoidalQuadCoeffs;
    }

    void addDefectConstraintsImpl() override;
private:

    void createVariables() override {
        m_vars[Var::initial_time] = m_opti.variable();
        m_vars[Var::final_time] = m_opti.variable();
        m_vars[Var::states] = m_opti.variable(m_numStates, m_numTimes);
        m_vars[Var::controls] = m_opti.variable(m_numControls, m_numTimes);
        m_vars[Var::parameters] = m_opti.variable(m_numParameters, 1);
        m_mesh = DM::linspace(0, 1, m_numTimes);
    }

    std::unique_ptr<DynamicsFunction> m_dynamicsFunction;

};

/// Currently, this class only returns the right-hand-side of the generalized
/// accelerations differential equations. Currently, we only support
/// models for which qdot = u. That is, the N matrix (qdot = N u) is identity.
/// This allows us to compute qdot symbolically through CasADi.
class CasADiTrapezoidal::DynamicsFunction : public CasADiFunction {
public:
    DynamicsFunction(const std::string& name,
            const CasADiTranscription& transcrip,
            const OpenSim::MucoProblemRep& problem,
            casadi::Dict opts=casadi::Dict())
            : CasADiFunction(transcrip, problem) {
        setCommonOptions(opts);
        construct(name, opts);
    }
    ~DynamicsFunction() override {}
    /// 0. time
    /// 1. states
    /// 2. controls
    /// 3. parameters
    casadi_int get_n_in() override { return 4; }
    casadi_int get_n_out() override { return 1; }
    std::string get_name_in(casadi_int i) override {
        switch (i) {
        case 0: return "time";
        case 1: return "states";
        case 2: return "controls";
        case 3: return "parameters";
        default:
            OPENSIM_THROW(Exception, "Internal error.");
        }
    }
    std::string get_name_out(casadi_int i) override {
        switch (i) {
        case 0: return "dynamics";
        default:
            OPENSIM_THROW(Exception, "Internal error.");
        }
    }
    casadi::Sparsity get_sparsity_in(casadi_int i) override {
        if (i == 0) {
            return casadi::Sparsity::dense(1, 1);
        } else if (i == 1) {
            return casadi::Sparsity::dense(p.getNumStates(), 1);
        } else if (i == 2) {
            return casadi::Sparsity::dense(p.getNumControls(), 1);
        } else if (i == 3) {
            return casadi::Sparsity::dense(p.getNumParameters(), 1);
        } else {
            return casadi::Sparsity(0, 0);
        }
    }
    casadi::Sparsity get_sparsity_out(casadi_int i) override;
    void init() override {}

    int eval(const double** arg, double** res, casadi_int*, double*, void*)
    const override;
};

/*
class CasADiTrapezoidalImplicit : public CasADiTrapezoidal {
private:
    void addDefectConstraintsImpl() override {
        const int NQ = m_state.getNQ();
        const auto& states = m_vars[Var::states];
        MX qdot = states(Slice(NQ, 2 * NQ), 0);
        MX udot = states(Slice(2 * NQ, NQ), 0);
        MX zdot = m_dynamicsFunction->operator()(
                {m_vars[Var::initial_time],
                 states(Slice(), 0),
                 m_vars[Var::controls](Slice(), 0),
                 m_vars[Var::parameters]
                }).at(0);
        MX xdot_im1 = casadi::MX::vertcat({qdot, udot, zdot});
        for (int itime = 1; itime < m_numTimes; ++itime) {
            auto h = m_times(itime) - m_times(itime - 1);
            auto x_i = states(Slice(), itime);
            auto x_im1 = states(Slice(), itime - 1);
            qdot = states(Slice(NQ, 2 * NQ), itime);
            udot = states(Slice(2 * NQ, NQ), itime);
            zdot = m_dynamicsFunction->operator()(
                    {m_times(itime),
                     m_vars[Var::states](Slice(), itime),
                     m_vars[Var::controls](Slice(), itime),
                     m_vars[Var::parameters]}).at(0);
            MX xdot_i = MX::vertcat({qdot, udot, zdot});
            m_opti.subject_to(x_i == (x_im1 + 0.5 * h * (xdot_i + xdot_im1)));
            m_opti.subject_to(residual == 0);
            xdot_im1 = xdot_i;
        }

    }
    void createVariables() override {
        CasADiTranscription::createVariables();
        m_vars[Var::derivatives] = m_opti.variable(m_genSpeeds, m_numTimes);
    }
};
*/

#endif // MUSCOLLO_CASADITRAPEZOIDAL_H
