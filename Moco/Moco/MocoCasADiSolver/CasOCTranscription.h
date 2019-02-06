#ifndef MOCO_CASOCTRANSCRIPTION_H
#define MOCO_CASOCTRANSCRIPTION_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: CasOCTranscription.h                                         *
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

#include "CasOCSolver.h"

namespace CasOC {

/// This is the base class for transcription schemes that convert a
/// CasOC::Problem into a general nonlinear programming problem. If you are
/// creating a new derived class, make sure to override all virtual functions
/// and obey the settings that the user specified in the CasOC::Solver. Build
/// the CasADi problem in the constructor of your derived class by defining the
/// following member variables in this class:
/// - m_vars
/// - m_lowerBounds
/// - m_upperBounds
/// Use setObjective() and addConstraints() to specify the functions in the
/// optimization problem.
class Transcription {
public:
    Transcription(const Solver& solver, const Problem& problem)
            : m_solver(solver), m_problem(problem) {}
    virtual ~Transcription() = default;
    Iterate createInitialGuessFromBounds() const {
        return createInitialGuessFromBoundsImpl();
    }
    Iterate createRandomIterateWithinBounds() const {
        return createRandomIterateWithinBoundsImpl();
    }
    Solution solve(const Iterate& guessOrig) {
        // Resample the guess.
        // -------------------
        const auto guessTimes = createTimesImpl(
                guessOrig.variables.at(Var::initial_time),
                guessOrig.variables.at(Var::final_time));
        const auto guess = guessOrig.resample(guessTimes);

        // Create the CasADi NLP function.
        // -------------------------------
        // Option handling is copied from casadi::OptiNode::solver().
        casadi::Dict options = m_solver.getPluginOptions();
        if (!options.empty()) {
            options[m_solver.getOptimSolver()] = m_solver.getSolverOptions();
        }
        // The inputs to nlpsol() are symbolic (casadi::MX).
        casadi::MXDict nlp;
        nlp.emplace(std::make_pair("x", flatten(m_vars)));
        // The m_objective symbolic variable holds an expression graph including all
        // the calculations performed on the variables x.
        nlp.emplace(std::make_pair("f", m_objective));
        // The m_constraints symbolic vector holds all of the expressions for
        // the constraint functions.
        // veccat() concatenates std::vectors into a single MX vector.
        nlp.emplace(std::make_pair("g", casadi::MX::veccat(m_constraints)));
        // auto hessian = casadi::MX::hessian(nlp["f"], nlp["x"]);
        // hessian.sparsity().to_file(
        //         "CasOCTranscription_objective_Hessian_sparsity.mtx");
        // auto jacobian = casadi::MX::jacobian(nlp["g"], nlp["x"]);
        // jacobian.sparsity().to_file(
        //         "CasOCTranscription_constraint_Jacobian_sparsity.mtx");
        const casadi::Function nlpFunc =
                casadi::nlpsol("nlp", m_solver.getOptimSolver(), nlp, options);

        // Run the optimization (evaluate the CasADi NLP function).
        // --------------------------------------------------------
        // The inputs and outputs of nlpFunc are numeric (casadi::DM).
        const casadi::DMDict nlpResult =
                nlpFunc(casadi::DMDict{{"x0", flatten(guess.variables)},
                        {"lbx", flatten(m_lowerBounds)},
                        {"ubx", flatten(m_upperBounds)},
                        {"lbg", casadi::DM::veccat(m_constraintsLowerBounds)},
                        {"ubg", casadi::DM::veccat(m_constraintsUpperBounds)}});

        // Create a CasOC::Solution.
        // -------------------------
        Solution solution = m_problem.createIterate<Solution>();
        solution.variables = expand(nlpResult.at("x"));

        solution.times = createTimesImpl(solution.variables[Var::initial_time],
                solution.variables[Var::final_time]);
        solution.stats = nlpFunc.stats();
        return solution;
    }

protected:
    template <typename TRow, typename TColumn>
    void setVariableBounds(Var var, const TRow& rowIndices,
            const TColumn& columnIndices, const Bounds& bounds) {
        if (bounds.isSet()) {
            const auto& lower = bounds.lower;
            m_lowerBounds[var](rowIndices, columnIndices) = lower;
            const auto& upper = bounds.upper;
            m_upperBounds[var](rowIndices, columnIndices) = upper;
        } else {
            m_lowerBounds[var](rowIndices, columnIndices) =
                    -std::numeric_limits<double>::infinity();
            m_upperBounds[var](rowIndices, columnIndices) =
                    std::numeric_limits<double>::infinity();
        }
    }

    void setObjective(casadi::MX objective) {
        m_objective = std::move(objective);
    }

    void addConstraints(const casadi::DM& lower, const casadi::DM& upper,
            const casadi::MX& equations);

    const Solver& m_solver;
    const Problem& m_problem;
    VariablesMX m_vars;
    VariablesDM m_lowerBounds;
    VariablesDM m_upperBounds;

private:
    casadi::MX m_objective;
    std::vector<casadi::MX> m_constraints;
    std::vector<casadi::DM> m_constraintsLowerBounds;
    std::vector<casadi::DM> m_constraintsUpperBounds;

private:
    virtual Iterate createInitialGuessFromBoundsImpl() const = 0;
    virtual Iterate createRandomIterateWithinBoundsImpl() const = 0;
    virtual casadi::DM createTimesImpl(
            casadi::DM initialTime, casadi::DM finalTime) const = 0;

    /// Use this function to ensure you iterate through variables in the same
    /// order.
    template <typename T>
    static std::vector<Var> getSortedVarKeys(const Variables<T>& vars) {
        std::vector<Var> keys;
        for (const auto& kv : vars) { keys.push_back(kv.first); }
        std::sort(keys.begin(), keys.end());
        return keys;
    }
    /// Convert the map of variables into a column vector, for passing onto
    /// nlpsol(), etc.
    template <typename T>
    static T flatten(const CasOC::Variables<T>& vars) {
        std::vector<T> stdvec;
        for (const auto& key : getSortedVarKeys(vars)) {
            stdvec.push_back(vars.at(key));
        }
        return T::veccat(stdvec);
    }
    /// Convert the 'x' column vector into separate variables.
    CasOC::VariablesDM expand(const casadi::DM& x) const {
        CasOC::VariablesDM out;
        using casadi::Slice;
        casadi_int offset = 0;
        for (const auto& key : getSortedVarKeys(m_vars)) {
            const auto& value = m_vars.at(key);
            // Convert a portion of the column vector into a matrix.
            out[key] = casadi::DM::reshape(
                    x(Slice(offset, offset + value.numel())), value.rows(),
                    value.columns());
            offset += value.numel();
        }
        return out;
    }
};

} // namespace CasOC

#endif // MOCO_CASOCTRANSCRIPTION_H
