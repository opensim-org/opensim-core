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
    Solution solve(const Iterate& guess) {
        auto guessResampled = guess.resample(
                createTimesImpl(guess.variables.at(Var::initial_time),
                        guess.variables.at(Var::final_time)));
        for (auto& kv : m_vars) {
            m_opti.set_initial(
                    kv.second, guessResampled.variables.at(kv.first));
        }
        return solveImpl();
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
            m_opti.subject_to(
                    lower <= m_vars[var](rowIndices, columnIndices) <= upper);
        } else {
            m_lowerBounds[var](rowIndices, columnIndices) =
                    -std::numeric_limits<double>::infinity();
            m_upperBounds[var](rowIndices, columnIndices) =
                    std::numeric_limits<double>::infinity();
        }
    }
    template <typename InvokeOn>
    VariablesDM convertToVariablesDM(
            const InvokeOn& obj, const VariablesMX& varsMX) const {
        VariablesDM varsDM;
        for (const auto& kv : varsMX) {
            varsDM[kv.first] = obj.value(kv.second);
        }
        return varsDM;
    }

    const Solver& m_solver;
    const Problem& m_problem;
    mutable casadi::Opti m_opti;
    VariablesMX m_vars;
    VariablesDM m_lowerBounds;
    VariablesDM m_upperBounds;
    VariablesDM m_guess;

private:
    virtual Iterate createInitialGuessFromBoundsImpl() const = 0;
    virtual Iterate createRandomIterateWithinBoundsImpl() const = 0;
    virtual casadi::DM createTimesImpl(
            casadi::DM initialTime, casadi::DM finalTime) const = 0;
    virtual Solution solveImpl() const = 0;
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
    Solution solveImpl() const override;

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
