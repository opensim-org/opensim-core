#ifndef OPENSIM_CASOCFUNCTION_H
#define OPENSIM_CASOCFUNCTION_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoCasOCFunction.h                                               *
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

#include "CasOCIterate.h"

#include <OpenSim/Common/Exception.h>

namespace CasOC {

class Problem;

using VectorDM = std::vector<casadi::DM>;

class Function : public casadi::Callback {
public:
    virtual ~Function() = default;
    void constructFunction(const Problem* casProblem, const std::string& name,
            const std::string& finiteDiffScheme,
            std::shared_ptr<const std::vector<VariablesDM>>
                    pointsForSparsityDetection);
    void setCommonOptions(casadi::Dict& opts) {
        // Compute the derivatives of this function using finite differences.
        opts["enable_fd"] = true;
        opts["fd_method"] = getFiniteDifferenceScheme();
        // Using "forward", iterations are 10x faster but problems are less
        // likely to converge.
    }
    std::string getFiniteDifferenceScheme() {
        return m_finite_difference_scheme;
    }
    casadi_int get_n_in() override { return 6; }
    std::string get_name_in(casadi_int i) override {
        switch (i) {
        case 0: return "time";
        case 1: return "states";
        case 2: return "controls";
        case 3: return "multipliers";
        case 4: return "derivatives";
        case 5: return "parameters";
        default: OPENSIM_THROW(OpenSim::Exception, "Internal error.");
        }
    }
    casadi::Sparsity get_sparsity_in(casadi_int i) override;
    bool has_jacobian_sparsity() const override {
        return !m_fullPointsForSparsityDetection->empty();
    }
    casadi::Sparsity get_jacobian_sparsity() const override;

protected:
    const Problem* m_casProblem;

private:
    /// Here, "point" refers to a vector of all variables in the optimization
    /// problem.
    VectorDM getSubsetPointsForSparsityDetection() const {
        VectorDM out(m_fullPointsForSparsityDetection->size());
        for (int i = 0; i < (int)out.size(); ++i) {
            out[i] = getSubsetPoint(m_fullPointsForSparsityDetection->at(i));
        }
        return out;
    }
    virtual casadi::DM getSubsetPoint(const VariablesDM& fullPoint) const {
        int itime = 0;
        using casadi::Slice;
        return casadi::DM::vertcat({fullPoint.at(initial_time),
                fullPoint.at(states)(Slice(), itime),
                fullPoint.at(controls)(Slice(), itime),
                fullPoint.at(multipliers)(Slice(), itime),
                fullPoint.at(derivatives)(Slice(), itime),
                fullPoint.at(parameters)});
    }

    std::string m_finite_difference_scheme = "central";

    std::shared_ptr<const std::vector<VariablesDM>>
            m_fullPointsForSparsityDetection;
};

class PathConstraint : public Function {
public:
    void constructFunction(const Problem* casProblem, const std::string& name,
            int index, int numEquations, const std::string& finiteDiffScheme,
            std::shared_ptr<const std::vector<VariablesDM>>
                    pointsForSparsityDetection) {
        m_index = index;
        m_numEquations = numEquations;
        Function::constructFunction(
                casProblem, name, finiteDiffScheme, pointsForSparsityDetection);
    }
    casadi_int get_n_out() override final { return 1; }
    std::string get_name_out(casadi_int i) override final {
        switch (i) {
        case 0: return "path_constraint_" + name();
        default: OPENSIM_THROW(OpenSim::Exception, "Internal error.");
        }
    }
    casadi::Sparsity get_sparsity_out(casadi_int i) override final {
        if (i == 0) {
            return casadi::Sparsity::dense(m_numEquations, 1);
        } else
            return casadi::Sparsity(0, 0);
    }
    VectorDM eval(const VectorDM& args) const override;

protected:
    int m_index = -1;
    int m_numEquations = -1;
};

class Integrand : public Function {
public:
    void constructFunction(const Problem* casProblem, const std::string& name,
            int index, const std::string& finiteDiffScheme,
            std::shared_ptr<const std::vector<VariablesDM>>
                    pointsForSparsityDetection) {
        m_index = index;
        Function::constructFunction(
                casProblem, name, finiteDiffScheme, pointsForSparsityDetection);
    }
    casadi_int get_n_out() override final { return 1; }
    std::string get_name_out(casadi_int i) override final {
        switch (i) {
        case 0: return "integrand";
        default: OPENSIM_THROW(OpenSim::Exception, "Internal error.");
        }
    }
    casadi::Sparsity get_sparsity_out(casadi_int i) override final {
        if (i == 0)
            return casadi::Sparsity::scalar();
        else
            return casadi::Sparsity(0, 0);
    }

protected:
    int m_index = -1;
};

class CostIntegrand : public Integrand {
public:
    VectorDM eval(const VectorDM& args) const override;
};

class EndpointConstraintIntegrand : public Integrand {
public:
    VectorDM eval(const VectorDM& args) const override;
};

/// This function takes initial states/controls, final states/controls, and an
/// integral.
class Endpoint : public Function {
public:
    void constructFunction(const Problem* casProblem, const std::string& name,
            int index,
            int numEquations,
            const std::string& finiteDiffScheme,
            std::shared_ptr<const std::vector<VariablesDM>>
            pointsForSparsityDetection) {
        m_index = index;
        m_numEquations = numEquations;
        Function::constructFunction(
                casProblem, name, finiteDiffScheme, pointsForSparsityDetection);
    }
    casadi_int get_n_in() override { return 12; }
    std::string get_name_in(casadi_int i) override final {
        switch (i) {
        case 0: return "initial_time";
        case 1: return "initial_states";
        case 2: return "initial_controls";
        case 3: return "initial_multipliers";
        case 4: return "initial_derivatives";
        case 5: return "final_time";
        case 6: return "final_states";
        case 7: return "final_controls";
        case 8: return "final_multipliers";
        case 9: return "final_derivatives";
        case 10: return "parameters";
        case 11: return "integral";
        default: OPENSIM_THROW(OpenSim::Exception, "Internal error.");
        }
    }
    casadi::Sparsity get_sparsity_in(casadi_int i) override;
    casadi_int get_n_out() override final { return 1; }
    std::string get_name_out(casadi_int i) override final {
        switch (i) {
        case 0: return "value";
        default: OPENSIM_THROW(OpenSim::Exception, "Internal error.");
        }
    }
    casadi::Sparsity get_sparsity_out(casadi_int i) override final {
        if (i == 0)
            return casadi::Sparsity::dense(m_numEquations, 1);
        else
            return casadi::Sparsity(0, 0);
    }
    /// The endpoint input is not simply a subset of the NLP variables; the
    /// endpoint function also depends on an integral, computed from an
    /// integrand function and using a transcription's quadrature scheme.
    /// Ideally, the value for the integral would be computed properly from the
    /// provided point, but applying the integrand function and quadrature
    /// scheme here is complicated. For simplicity, we provide the integral as
    /// 0.
    casadi::DM getSubsetPoint(const VariablesDM& fullPoint) const override {
        using casadi::Slice;
        return casadi::DM::vertcat({fullPoint.at(initial_time),
                fullPoint.at(states)(Slice(), 0),
                fullPoint.at(controls)(Slice(), 0),
                fullPoint.at(multipliers)(Slice(), 0),
                fullPoint.at(derivatives)(Slice(), 0), fullPoint.at(final_time),
                fullPoint.at(states)(Slice(), -1),
                fullPoint.at(controls)(Slice(), -1),
                fullPoint.at(multipliers)(Slice(), -1),
                fullPoint.at(derivatives)(Slice(), -1),
                fullPoint.at(parameters),
                // TODO: We should find a way to actually compute the integral
                // from fullPoint. Or, make the integral an optimization
                // variable.
                casadi::DM::zeros(1, 1)});
    }
protected:
    int m_index = -1;
    int m_numEquations = -1;
};

/// This invokes CasOC::Problem::calcCost().
class Cost : public Endpoint {
public:
    VectorDM eval(const VectorDM& args) const override;
};

/// This invokes CasOC::Problem::calcEndpointConstraint().
class EndpointConstraint : public Endpoint {
public:
    VectorDM eval(const VectorDM& args) const override;

};

/// This function should compute forward dynamics (explicit multibody dynamics),
/// auxiliary explicit dynamics, and the errors for the kinematic constraints.
template <bool CalcKCErrors>
class MultibodySystemExplicit : public Function {
public:
    casadi_int get_n_out() override final { return 4; }
    std::string get_name_out(casadi_int i) override final {
        switch (i) {
        case 0: return "multibody_derivatives";
        case 1: return "auxiliary_derivatives";
        case 2: return "auxiliary_residuals";
        case 3: return "kinematic_constraint_errors";
        default: OPENSIM_THROW(OpenSim::Exception, "Internal error.");
        }
    }
    casadi::Sparsity get_sparsity_out(casadi_int i) override final;
    VectorDM eval(const VectorDM& args) const override;
};

/// This function should compute a velocity correction term to make feasible
/// problems that enforce kinematic constraints and their derivatives.
class VelocityCorrection : public Function {
public:
    casadi_int get_n_in() override final { return 4; }
    casadi_int get_n_out() override final { return 1; }
    std::string get_name_in(casadi_int i) override final {
        switch (i) {
        case 0: return "time";
        case 1: return "multibody_states";
        case 2: return "slacks";
        case 3: return "parameters";
        default: OPENSIM_THROW(OpenSim::Exception, "Internal error.");
        }
    }
    std::string get_name_out(casadi_int i) override final {
        switch (i) {
        case 0: return "velocity_correction";
        default: OPENSIM_THROW(OpenSim::Exception, "Internal error.");
        }
    }
    casadi::Sparsity get_sparsity_in(casadi_int i) override final;
    casadi::Sparsity get_sparsity_out(casadi_int i) override final;
    VectorDM eval(const VectorDM& args) const override;
    casadi::DM getSubsetPoint(const VariablesDM& fullPoint) const override;
};

template <bool CalcKCErrors>
class MultibodySystemImplicit : public Function {
    casadi_int get_n_out() override final { return 4; }
    std::string get_name_out(casadi_int i) override final {
        switch (i) {
        case 0: return "multibody_residuals";
        case 1: return "auxiliary_derivatives";
        case 2: return "auxiliary_residuals";
        case 3: return "kinematic_constraint_errors";
        default: OPENSIM_THROW(OpenSim::Exception, "Internal error.");
        }
    }
    casadi::Sparsity get_sparsity_out(casadi_int i) override final;
    VectorDM eval(const VectorDM& args) const override;
};

} // namespace CasOC

#endif // OPENSIM_CASOCFUNCTION_H
