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

protected:
    void createVariables() override {
        m_vars[Var::initial_time] = m_opti.variable();
        m_vars[Var::final_time] = m_opti.variable();
        m_vars[Var::states] = m_opti.variable(m_numStates, m_numTimes);
        m_vars[Var::controls] = m_opti.variable(m_numControls, m_numTimes);
        m_vars[Var::parameters] = m_opti.variable(m_numParameters, 1);
        m_mesh = DM::linspace(0, 1, m_numTimes);
    }

private:
    std::unique_ptr<DynamicsFunction> m_dynamicsFunction;

};

/// Currently, this class only returns the right-hand-side of the generalized
/// accelerations and auxiliary differential equations. Currently, we only
/// support models for which qdot = u. That is, the N matrix (qdot = N u) is
/// identity. This allows us to compute qdot symbolically through CasADi.
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

// TODO: Throw exception if costs realize to acceleration.
class CasADiTrapezoidalImplicit : public CasADiTrapezoidal {
    using CasADiTrapezoidal::CasADiTrapezoidal;
    // TODO: Rename to include path constraints.
    void addDefectConstraintsImpl() override;
    void createVariables() override {
        CasADiTrapezoidal::createVariables();
        m_vars[Var::derivatives] = m_opti.variable(m_state.getNU(), m_numTimes);
        std::cout << "DEBUG TrapImpl::createVariables()" << std::endl;
    }
private:
    class DynamicsFunction;
    class ResidualFunction;
    std::unique_ptr<DynamicsFunction> m_dynamicsFunction;
    std::unique_ptr<ResidualFunction> m_residualFunction;
};

class CasADiTrapezoidalImplicit::DynamicsFunction : public CasADiFunction {
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
        case 0: return "auxiliary_dynamics";
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

class CasADiTrapezoidalImplicit::ResidualFunction : public CasADiFunction {
public:
    ResidualFunction(const std::string& name,
            const CasADiTranscription& transcrip,
            const OpenSim::MucoProblemRep& problem,
            casadi::Dict opts=casadi::Dict())
            : CasADiFunction(transcrip, problem) {
        setCommonOptions(opts);
        construct(name, opts);
    }
    ~ResidualFunction() override {}
    casadi_int get_n_in() override { return 5; }
    casadi_int get_n_out() override { return 1; }
    std::string get_name_in(casadi_int i) override {
        switch (i) {
        case 0: return "time";
        case 1: return "states";
        case 2: return "controls";
        case 3: return "accelerations";
        case 4: return "parameters";
        default:
            OPENSIM_THROW(Exception, "Internal error.");
        }
    }
    std::string get_name_out(casadi_int i) override {
        switch (i) {
        case 0: return "residual";
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
            return casadi::Sparsity::dense(m_transcrip.m_state.getNU(), 1);
        } else if (i == 4) {
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

#endif // MUSCOLLO_CASADITRAPEZOIDAL_H
