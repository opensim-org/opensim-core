/* -------------------------------------------------------------------------- *
 * OpenSim Moco: CasOCTranscription.cpp                                       *
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
#include "CasOCTranscription.h"

using casadi::DM;
using casadi::MX;
using casadi::Slice;

namespace CasOC {

void Transcription::transcribe() {

    // Create variables.
    // -----------------    
    m_vars[Var::initial_time] = MX::sym("initial_time");
    m_vars[Var::final_time] = MX::sym("final_time");
    m_vars[Var::states] =
        MX::sym("states", m_problem.getNumStates(), m_numGridPoints);
    m_vars[Var::controls] =
        MX::sym("controls", m_problem.getNumControls(), m_numGridPoints);
    m_vars[Var::multipliers] = MX::sym(
        "multipliers", m_problem.getNumMultipliers(), m_numGridPoints);
    // TODO: This assumes that slack variables are applied at all collocation
    // points on the mesh interval interior.
    m_vars[Var::slacks] = MX::sym(
        "slacks", m_problem.getNumSlacks(), m_numGridPoints - m_numMeshPoints);
    m_vars[Var::parameters] =
        MX::sym("parameters", m_problem.getNumParameters(), 1);

    m_grid = DM::linspace(0, 1, m_numGridPoints);
    m_duration = m_vars[Var::final_time] - m_vars[Var::initial_time];
    m_times = createTimes(m_vars[Var::initial_time], m_vars[Var::final_time]);

    // Set variable bounds.
    // --------------------
    auto initializeBounds = [&](VariablesDM& bounds) {
        for (auto& kv : m_vars) {
            bounds[kv.first] = DM(kv.second.rows(), kv.second.columns());
        }
    };
    initializeBounds(m_lowerBounds);
    initializeBounds(m_upperBounds);

    setVariableBounds(
        Var::initial_time, 0, 0, m_problem.getTimeInitialBounds());
    setVariableBounds(Var::final_time, 0, 0, m_problem.getTimeFinalBounds());

    {
        const auto& stateInfos = m_problem.getStateInfos();
        int is = 0;
        for (const auto& info : stateInfos) {
            setVariableBounds(
                Var::states, is, Slice(1, m_numGridPoints - 1), info.bounds);
            setVariableBounds(Var::states, is, 0, info.initialBounds);
            setVariableBounds(Var::states, is, -1, info.finalBounds);
            ++is;
        }
    }
    {
        const auto& controlInfos = m_problem.getControlInfos();
        int ic = 0;
        for (const auto& info : controlInfos) {
            setVariableBounds(Var::controls, ic, Slice(1, m_numGridPoints - 1),
                info.bounds);
            setVariableBounds(Var::controls, ic, 0, info.initialBounds);
            setVariableBounds(Var::controls, ic, -1, info.finalBounds);
            ++ic;
        }
    }
    {
        const auto& multiplierInfos = m_problem.getMultiplierInfos();
        int im = 0;
        for (const auto& info : multiplierInfos) {
            setVariableBounds(Var::multipliers, 
                im, Slice(1, m_numGridPoints - 1), info.bounds);
            setVariableBounds(Var::multipliers, im, 0, info.initialBounds);
            setVariableBounds(Var::multipliers, im, -1, info.finalBounds);
            ++im;
        }
    }
    {
        const auto& slackInfos = m_problem.getSlackInfos();
        int isl = 0;
        for (const auto& info : slackInfos) {
            setVariableBounds(Var::slacks, isl, Slice(), info.bounds);
            ++isl;
        }
    }
    {
        const auto& paramInfos = m_problem.getParameterInfos();
        int ip = 0;
        for (const auto& info : paramInfos) {
            setVariableBounds(Var::parameters, ip, 0, info.bounds);
        }
    }

    // Cost.
    // -----
    DM quadCoeffs = this->createQuadratureCoefficients();
    MX integralCost = 0;
    for (int itime = 0; itime < m_numGridPoints; ++itime) {
        const auto out = m_problem.getIntegralCostIntegrand().operator()(
        {m_times(itime, 0), m_vars[Var::states](Slice(), itime),
            m_vars[Var::controls](Slice(), itime),
            m_vars[Var::parameters]});
        integralCost += quadCoeffs(itime) * out.at(0);
        // TODO: Obey user option for if this penalty should exist.
        if (m_problem.getNumMultipliers()) {
            const auto mults = m_vars[Var::multipliers](Slice(), itime);
            const int multiplierWeight = 100.0; // TODO m_mocoSolver.get_lagrange_multiplier_weight();
            integralCost += multiplierWeight * dot(mults, mults);
        }
    }
    integralCost *= m_duration;

    const auto endpointCostOut =
        m_problem.getEndpointCost().operator()({m_vars[Var::final_time],
            m_vars[Var::states](Slice(), -1), m_vars[Var::parameters]});
    const auto endpointCost = endpointCostOut.at(0);
    setObjective(integralCost + endpointCost);

    // Compute DAE at necessary grid points.
    // -------------------------------------
    const int NQ = m_problem.getNumCoordinates();
    const int NU = m_problem.getNumSpeeds();
    OPENSIM_THROW_IF(NQ != NU, OpenSim::Exception, "NQ != NU");
    const DM kinConIndices = createKinematicConstraintIndices();
    const auto shape = kinConIndices.size();
    OPENSIM_THROW_IF(shape.first != 1 && shape.second != 1, OpenSim::Exception, 
        OpenSim::format("createKinematicConstraintIndicesImpl() must "
            "return a vector, but a matrix with shape [%i, %i] was returned.", 
            shape.first, shape.second));
    OPENSIM_THROW_IF(shape.first != m_numGridPoints || 
        shape.second != m_numGridPoints, OpenSim::Exception, OpenSim::format(
            "createKinematicConstraintIndicesImpl() must return a "
            "vector of length m_numGridPoints, but a vector of length %i was "
            "returned.", shape.first == 1 ? shape.second : shape.first));

    // TODO: Does creating all this memory have efficiency implications in
    // CasADi?
    xdot = MX(m_problem.getNumStates(), m_numGridPoints);
    pvaerr = MX(m_problem.getNumKinematicConstraintEquations(), 
        kinConIndices.nnz());
    MX this_xdot;
    MX this_pvaerr;
    int kinIdx = 0;
    for (int itime = 0; itime < m_numGridPoints; ++itime) {
        bool calcPVAErr = kinConIndices(Slice(itime)).__nonzero__() ? true : 
            false;
        calcDAE(itime, NQ, this_xdot, calcPVAErr, this_pvaerr);
        xdot(Slice(), itime) = this_xdot;
        if (calcPVAErr) {
            pvaerr(Slice(), kinIdx) = this_pvaerr;
            ++kinIdx;
        }
    }

    // Apply constraints.
    // ------------------
    applyConstraints();
}

Iterate Transcription::createInitialGuessFromBounds() const {
    auto setToMidpoint = [](DM& output, const DM& lowerDM, const DM& upperDM) {
        for (int irow = 0; irow < output.rows(); ++irow) {
            for (int icol = 0; icol < output.columns(); ++icol) {
                const auto& lower = double(lowerDM(irow, icol));
                const auto& upper = double(upperDM(irow, icol));
                if (!std::isinf(lower) && !std::isinf(upper)) {
                    output(irow, icol) = 0.5 * (upper + lower);
                }
                else if (!std::isinf(lower))
                    output(irow, icol) = lower;
                else if (!std::isinf(upper))
                    output(irow, icol) = upper;
                else
                    output(irow, icol) = 0;
            }
        }
    };
    Iterate casGuess = m_problem.createIterate();
    casGuess.variables = m_lowerBounds;
    for (auto& kv : casGuess.variables) {
        setToMidpoint(kv.second, m_lowerBounds.at(kv.first),
            m_upperBounds.at(kv.first));
    }
    casGuess.times = createTimes(casGuess.variables[Var::initial_time],
        casGuess.variables[Var::final_time]);
    return casGuess;
}

Iterate Transcription::createRandomIterateWithinBounds() const {
    static SimTK::Random::Uniform randGen(-1, 1);
    auto setRandom = [](DM& output, const DM& lowerDM, const DM& upperDM) {
        for (int irow = 0; irow < output.rows(); ++irow) {
            for (int icol = 0; icol < output.columns(); ++icol) {
                const auto& lower = double(lowerDM(irow, icol));
                const auto& upper = double(upperDM(irow, icol));
                const auto rand = randGen.getValue();
                auto value = 0.5 * (rand + 1.0) * (upper - lower) + lower;
                if (std::isnan(value)) value = SimTK::clamp(lower, rand, upper);
                output(irow, icol) = value;
            }
        }
    };
    Iterate casIterate = m_problem.createIterate();
    casIterate.variables = m_lowerBounds;
    for (auto& kv : casIterate.variables) {
        setRandom(kv.second, m_lowerBounds.at(kv.first),
            m_upperBounds.at(kv.first));
    }
    casIterate.times = createTimes(casIterate.variables[Var::initial_time],
        casIterate.variables[Var::final_time]);
    return casIterate;
}

void Transcription::calcDAE(casadi_int itime, const int& NQ, MX& xdot, 
    bool calcPVAErr, MX& pvaerr)
{
    const auto& states = m_vars[Var::states];
    const MX u = states(Slice(NQ, 2 * NQ), itime);
    const MX qdot = u; // TODO: This assumes the N matrix is identity.
    const auto& multibodyFunc = calcPVAErr ? m_problem.getMultibodySystem() :
        m_problem.getMultibodySystemUnconstrained();
    const auto dynamicsOutput = multibodyFunc.operator()(
        {m_times(itime), m_vars[Var::states](Slice(), itime),
            m_vars[Var::controls](Slice(), itime),
            m_vars[Var::multipliers](Slice(), itime),
            m_vars[Var::slacks](Slice(), itime),
            m_vars[Var::parameters]});
    const MX udot = dynamicsOutput.at(0);
    const MX zdot = dynamicsOutput.at(1);
    xdot = casadi::MX::vertcat({qdot, udot, zdot});
    if (calcPVAErr) {
        pvaerr = dynamicsOutput.at(2);
    }
}

void Transcription::addConstraints(const casadi::DM& lower,
        const casadi::DM& upper, const casadi::MX& equations) {
    OPENSIM_THROW_IF(
            lower.size() != upper.size() || lower.size() != equations.size(),
            OpenSim::Exception,
            "Arguments must have the same size.");
    m_constraintsLowerBounds.push_back(lower);
    m_constraintsUpperBounds.push_back(upper);
    m_constraints.push_back(equations);
}

} // namespace CasOC
