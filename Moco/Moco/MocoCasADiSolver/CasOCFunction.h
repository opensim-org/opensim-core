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
            std::shared_ptr<std::vector<const VariablesDM>> pointsForSparsityDetection);
    void setCommonOptions(casadi::Dict& opts) {
        // Compute the derivatives of this function using finite differences.
        opts["enable_fd"] = true;
        opts["fd_method"] = getFiniteDifferenceScheme();
        // Using "forward", iterations are 10x faster but problems don't
        // converge.
    }
    std::string getFiniteDifferenceScheme() {
        return m_finite_difference_scheme;
    }
    bool has_jacobian_sparsity() const override {
        return !m_fullPointsForSparsityDetection->empty();
    }
    casadi::Sparsity get_jacobian_sparsity() const override;

protected:
    const Problem* m_casProblem;

private:
    VectorDM getSubsetPointsForSparsityDetection() const {
        VectorDM out(m_fullPointsForSparsityDetection->size());
        for (int i = 0; i < (int)out.size(); ++i) {
            out[i] = getSubsetPoint(m_fullPointsForSparsityDetection->at(i));
        }
        return out;
    }
    virtual casadi::DM getSubsetPoint(const VariablesDM& fullPoint) const = 0;

    std::string m_finite_difference_scheme = "central";

    std::shared_ptr<std::vector<const VariablesDM>>
            m_fullPointsForSparsityDetection;
};

class PathConstraint : public Function {
public:
    void constructFunction(const Problem* casProblem, const std::string& name,
            int numEquations, const std::string& finiteDiffScheme,
            std::shared_ptr<std::vector<const VariablesDM>>
                    pointsForSparsityDetection) {
        m_numEquations = numEquations;
        Function::constructFunction(
                casProblem, name, finiteDiffScheme, pointsForSparsityDetection);
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
    casadi::DM getSubsetPoint(const VariablesDM& fullPoint) const override {
        int itime = 0;
        using casadi::Slice;
        return casadi::DM::vertcat({fullPoint.at(initial_time),
                fullPoint.at(states)(Slice(), itime),
                fullPoint.at(controls)(Slice(), itime),
                fullPoint.at(parameters)});
    }

protected:
    int m_numEquations;
};

class IntegralCostIntegrand : public Function {
public:
    casadi_int get_n_in() override final { return 4; }
    casadi_int get_n_out() override final { return 1; }
    // TODO: Must pass in Lagrange multipliers to properly minimize joint
    // reactions.
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
    casadi::DM getSubsetPoint(const VariablesDM& fullPoint) const override {
        int itime = 0;
        using casadi::Slice;
        return casadi::DM::vertcat({fullPoint.at(initial_time),
                fullPoint.at(states)(Slice(), itime),
                fullPoint.at(controls)(Slice(), itime),
                fullPoint.at(parameters)});
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
    casadi::DM getSubsetPoint(const VariablesDM& fullPoint) const override {
        using casadi::Slice;
        return casadi::DM::vertcat({fullPoint.at(final_time),
                fullPoint.at(states)(Slice(), -1), fullPoint.at(parameters)});
    }
};

/// This function should compute forward dynamics (explicit multibody dynamics),
/// auxiliary explicit dynamics, and the errors for the kinematic constraints.
template <bool CalcKinConErrors>
class MultibodySystem : public Function {
public:
    casadi_int get_n_in() override final { return 5; }
    casadi_int get_n_out() override final { return CalcKinConErrors ? 3 : 2; }
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
        case 2:
            if (CalcKinConErrors) {
                return "kinematic_constraint_errors";
            } else {
                OPENSIM_THROW(OpenSim::Exception, "Internal error.")
            }
        default: OPENSIM_THROW(OpenSim::Exception, "Internal error.");
        }
    }
    casadi::Sparsity get_sparsity_in(casadi_int i) override final;
    casadi::Sparsity get_sparsity_out(casadi_int i) override final;
    casadi::DM getSubsetPoint(const VariablesDM& fullPoint) const override {
        int itime = 0;
        using casadi::Slice;
        return casadi::DM::vertcat({fullPoint.at(initial_time),
                fullPoint.at(states)(Slice(), itime),
                fullPoint.at(controls)(Slice(), itime),
                fullPoint.at(multipliers)(Slice(), itime),
                fullPoint.at(parameters)});
    }
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
    casadi::DM getSubsetPoint(const VariablesDM& fullPoint) const override;
};

template <bool CalcKinConErrors>
class MultibodySystemImplicit : public Function {
    casadi_int get_n_in() override final { return 6; }
    casadi_int get_n_out() override final { return CalcKinConErrors ? 3 : 2; }
    std::string get_name_in(casadi_int i) override final {
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
    std::string get_name_out(casadi_int i) override final {
        switch (i) {
        case 0: return "multibody_residuals";
        case 1: return "auxiliary_derivatives";
        case 2:
            if (CalcKinConErrors) {
                return "kinematic_constraint_errors";
            } else {
                OPENSIM_THROW(OpenSim::Exception, "Internal error.")
            }
        default: OPENSIM_THROW(OpenSim::Exception, "Internal error.");
        }
    }
    casadi::Sparsity get_sparsity_in(casadi_int i) override final;
    casadi::Sparsity get_sparsity_out(casadi_int i) override final;
    casadi::DM getSubsetPoint(const VariablesDM& fullPoint) const override {
        int itime = 0;
        using casadi::Slice;
        return casadi::DM::vertcat({fullPoint.at(initial_time),
                fullPoint.at(states)(Slice(), itime),
                fullPoint.at(controls)(Slice(), itime),
                fullPoint.at(multipliers)(Slice(), itime),
                fullPoint.at(derivatives)(Slice(), itime),
                fullPoint.at(parameters)});
    }
};

} // namespace CasOC

#endif // MOCO_CASOCFUNCTION_H
