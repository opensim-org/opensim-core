#ifndef MOCO_CASOCTRANSCRIPTION_H
#define MOCO_CASOCTRANSCRIPTION_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoCasOCTranscription.cpp                                   *
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

#include "CasOCProblem.h"

namespace CasOC {

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
        auto guess = guessOrig.resample(
                createTimesImpl(guessOrig.variables.at(Var::initial_time),
                        guessOrig.variables.at(Var::final_time)));

        // Option handling is copied from casadi::OptiNode::solver().
        casadi::Dict options = m_solver.getPluginOptions();
        if (!options.empty()) {
            options[m_solver.getOptimSolver()] = m_solver.getSolverOptions();
        }
        casadi::MXDict nlp;
        nlp.emplace(std::make_pair("x", flatten(m_vars)));
        nlp.emplace(std::make_pair("f", m_objective));
        nlp.emplace(std::make_pair("g", casadi::MX::veccat(m_constraints)));
        auto nlpFunc =
                casadi::nlpsol("nlp", m_solver.getOptimSolver(), nlp, options);

        // Run the optimization.
        // ---------------------
        Solution solution = m_problem.createIterate<Solution>();
        const casadi::DMDict nlpResult =
                nlpFunc(casadi::DMDict{{"x0", flatten(guess.variables)},
                        {"lbx", flatten(m_lowerBounds)},
                        {"ubx", flatten(m_upperBounds)},
                        {"lbg", casadi::DM::veccat(m_constraintsLowerBounds)},
                        {"ubg", casadi::DM::veccat(m_constraintsUpperBounds)}});
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

#endif // MOCO_CASOCTRANSCRIPTION_H
