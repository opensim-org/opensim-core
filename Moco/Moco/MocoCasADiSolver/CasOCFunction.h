#ifndef MOCO_CASOCFUNCTION_H
#define MOCO_CASOCFUNCTION_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoCasOCFunction.h                                          *
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

#include <casadi/casadi.hpp>

#include <OpenSim/Common/Exception.h>

namespace CasOC {

class Problem;

class Function : public casadi::Callback {
public:
    virtual ~Function() = default;
    void constructFunction(const Problem* casProblem, const std::string& name) {
        m_casProblem = casProblem;
        casadi::Dict opts;
        setCommonOptions(opts);
        this->construct(name, opts);
    }
    void setCommonOptions(casadi::Dict& opts) {
        // Compute the derivatives of this function using finite differences.
        opts["enable_fd"] = true;
        opts["fd_method"] = "central";
        // Using "forward", iterations are 10x faster but problems don't
        // converge.
    }

protected:
    const Problem* m_casProblem;
};

class PathConstraint : public Function {
public:
    void constructFunction(const Problem* casProblem, const std::string& name,
            int numEquations) {
        Function::constructFunction(casProblem, name);
        m_numEquations = numEquations;
    }
    casadi_int get_n_in() override final { return 4; }
    casadi_int get_n_out() override final { return 1; }
    std::string get_name_in(casadi_int i) override final {
        switch (i) {
        case 0: return "time";
        case 1: return "states";
        case 2: return "controls";
        case 3: return "parameters";
        default: OPENSIM_THROW(OpenSim::Exception, "Internal error.");
        }
    }
    std::string get_name_out(casadi_int i) override final {
        switch (i) {
        case 0: return "path_constraint_" + name();
        default: OPENSIM_THROW(OpenSim::Exception, "Internal error.");
        }
    }
    casadi::Sparsity get_sparsity_in(casadi_int i) override final;
    casadi::Sparsity get_sparsity_out(casadi_int i) override final {
        if (i == 0) {
            return casadi::Sparsity::dense(m_numEquations, 1);
        } else
            return casadi::Sparsity(0, 0);
    }

protected:
    int m_numEquations;
};

class IntegralCostIntegrand : public Function {
public:
    using Function::Function;
    casadi_int get_n_in() override final { return 4; }
    casadi_int get_n_out() override final { return 1; }
    std::string get_name_in(casadi_int i) override final {
        switch (i) {
        case 0: return "time";
        case 1: return "states";
        case 2: return "controls";
        case 3: return "parameters";
        default: OPENSIM_THROW(OpenSim::Exception, "Internal error.");
        }
    }
    std::string get_name_out(casadi_int i) override final {
        switch (i) {
        case 0: return "integral_cost_integrand";
        default: OPENSIM_THROW(OpenSim::Exception, "Internal error.");
        }
    }
    casadi::Sparsity get_sparsity_in(casadi_int i) override final;
    casadi::Sparsity get_sparsity_out(casadi_int i) override final {
        if (i == 0)
            return casadi::Sparsity::scalar();
        else
            return casadi::Sparsity(0, 0);
    }
};

class EndpointCost : public Function {
public:
    casadi_int get_n_in() override final { return 3; }
    casadi_int get_n_out() override final { return 1; }
    std::string get_name_in(casadi_int i) override final {
        switch (i) {
        case 0: return "final_time";
        case 1: return "final_states";
        case 2: return "parameters";
        default: OPENSIM_THROW(OpenSim::Exception, "Internal error.");
        }
    }
    std::string get_name_out(casadi_int i) override final {
        switch (i) {
        case 0: return "endpoint_cost";
        default: OPENSIM_THROW(OpenSim::Exception, "Internal error.");
        }
    }
    casadi::Sparsity get_sparsity_in(casadi_int i) override final;
    casadi::Sparsity get_sparsity_out(casadi_int i) override final {
        if (i == 0)
            return casadi::Sparsity::scalar();
        else
            return casadi::Sparsity(0, 0);
    }
};

/// This function should compute forward dynamics (explicit multibody dynamics),
/// auxiliary explicit dynamics, and the errors for the kinematic constraints.
class MultibodySystem : public Function {
public:
    casadi_int get_n_in() override final { return 5; }
    casadi_int get_n_out() override final { return 3; }
    std::string get_name_in(casadi_int i) override final {
        switch (i) {
        case 0: return "time";
        case 1: return "states";
        case 2: return "controls";
        case 3: return "multipliers";
        case 4: return "parameters";
        default: OPENSIM_THROW(OpenSim::Exception, "Internal error.");
        }
    }
    std::string get_name_out(casadi_int i) override final {
        switch (i) {
        case 0: return "multibody_derivatives";
        case 1: return "auxiliary_derivatives";
        case 2: return "kinematic_constraint_errors";
        default: OPENSIM_THROW(OpenSim::Exception, "Internal error.");
        }
    }
    casadi::Sparsity get_sparsity_in(casadi_int i) override final;
    casadi::Sparsity get_sparsity_out(casadi_int i) override final;
};

} // namespace CasOC

#endif // MOCO_CASOCFUNCTION_H
