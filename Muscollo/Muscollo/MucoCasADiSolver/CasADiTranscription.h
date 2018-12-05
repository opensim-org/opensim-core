#ifndef MUSCOLLO_CASADITRANSCRIPTION_H
#define MUSCOLLO_CASADITRANSCRIPTION_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: CasADiTranscription.h                                    *
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

#include "../MuscolloUtilities.h"
#include "../MucoProblemRep.h"
#include "MucoCasADiSolver.h"
#include <casadi/casadi.hpp>

// TODO: temporary
using namespace OpenSim;
using casadi::MX;
using casadi::DM;
using casadi::Sparsity;
using casadi::Slice;
using casadi::Callback;
using casadi::Dict;

class CasADiTranscription;

class EndpointCost : public casadi::Callback {
public:
    EndpointCost(const std::string& name,
            const CasADiTranscription& transcrip,
            const OpenSim::MucoProblemRep& problem,
            casadi::Dict opts=casadi::Dict()) : m_transcrip(transcrip), p(problem) {
        opts["enable_fd"] = true;
        construct(name, opts);
    }
    ~EndpointCost() override {}
    casadi_int get_n_in() override { return 3; }
    casadi_int get_n_out() override { return 1; }
    void init() override {}
    casadi::Sparsity get_sparsity_in(casadi_int i) override {
        // TODO fix when using a matrix as input for states.
        // TODO detect this sparsity.
        if (i == 0) {
            return casadi::Sparsity::scalar();
        } else if (i == 1) {
            return casadi::Sparsity::dense(p.getNumStates(), 1);
        } else if (i == 3) {
            return casadi::Sparsity::dense(p.getNumParameters(), 1);
        } else {
            return casadi::Sparsity(0, 0);
        }
    }
    casadi::Sparsity get_sparsity_out(casadi_int i) override {
        if (i == 0) return casadi::Sparsity::scalar();
        else return casadi::Sparsity(0, 0);
    }
    /// Arguments:
    /// 0. time
    /// 1. final state
    /// 2. parameters
    std::vector<casadi::DM>
    eval(const std::vector<casadi::DM>& arg) const override;
private:
    const CasADiTranscription& m_transcrip;
    const OpenSim::MucoProblemRep& p;
};

class IntegrandCost : public casadi::Callback {
public:
    IntegrandCost(const std::string& name,
            const CasADiTranscription& transcrip,
            const OpenSim::MucoProblemRep& problem,
            casadi::Dict opts=casadi::Dict()) : m_transcrip(transcrip), p(problem) {
        opts["enable_fd"] = true;
        construct(name, opts);
    }
    ~IntegrandCost() override {}
    casadi_int get_n_in() override { return 4; }
    casadi_int get_n_out() override { return 1; }
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
    casadi::Sparsity get_sparsity_out(casadi_int i) override {
        if (i == 0) return casadi::Sparsity::scalar();
        else return casadi::Sparsity(0, 0);
    }
    std::vector<casadi::DM>
    eval(const std::vector<casadi::DM>& arg) const override;
private:
    const CasADiTranscription& m_transcrip;
    const OpenSim::MucoProblemRep& p;
};

class DynamicsFunction : public casadi::Callback {
public:
    DynamicsFunction(const std::string& name,
            const CasADiTranscription& transcrip,
            const OpenSim::MucoProblemRep& problem,
            casadi::Dict opts=casadi::Dict()) : m_transcrip(transcrip), p(problem) {
        opts["enable_fd"] = true;
        construct(name, opts);
    }
    ~DynamicsFunction() override {}
    casadi_int get_n_in() override { return 4; }
    casadi_int get_n_out() override { return 1; }
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
    casadi::Sparsity get_sparsity_out(casadi_int i) override {
        if (i == 0) return casadi::Sparsity::dense(p.getNumStates(), 1);
        else return casadi::Sparsity(0, 0);
    }
    void init() override {}

    std::vector<casadi::DM>
    eval(const std::vector<casadi::DM>& arg) const override;
private:
    const CasADiTranscription& m_transcrip;
    const OpenSim::MucoProblemRep& p;
};

template <typename T>
struct CasADiVariables {
    T initialTime;
    T finalTime;
    T states;
    T controls;
    T parameters;
};

class CasADiTranscription {
public:
    CasADiTranscription(const MucoCasADiSolver& solver,
            const MucoProblemRep& probRep)
            : m_solver(solver), m_probRep(probRep),
              m_model(m_probRep.getModel()),
              m_state(m_model.getWorkingState()) {}

    virtual ~CasADiTranscription() = default;

    template <typename T>
    T createTimes(const T& initialTime, const T& finalTime) const {
        return (finalTime - initialTime) * m_mesh + initialTime;
    }
    template <typename TIn, typename TOut = MucoIterate>
    TOut convertToMuco(const CasADiVariables<TIn>& casVars) const {
        SimTK::Matrix simtkStates;
        if (m_numStates) {
            const auto statesValue = m_opti.value(casVars.states);
            simtkStates.resize(m_numTimes, m_numStates);
            for (int istate = 0; istate < m_numStates; ++istate) {
                for (int itime = 0; itime < m_numTimes; ++itime) {
                    simtkStates(itime, istate) =
                            double(statesValue(istate, itime));
                }
            }
        }
        SimTK::Matrix simtkControls;
        if (m_numControls) {
            const auto controlsValue = m_opti.value(casVars.controls);
            simtkControls.resize(m_numTimes, m_numControls);
            for (int icontrol = 0; icontrol < m_numControls; ++icontrol) {
                for (int itime = 0; itime < m_numTimes; ++itime) {
                    simtkControls(itime, icontrol) =
                            double(controlsValue(icontrol, itime));
                }
            }
        }
        SimTK::RowVector simtkParameters;
        if (m_numParameters) {
            const auto paramsValue = m_opti.value(casVars.parameters);
            simtkParameters.resize(m_numParameters);
            for (int iparam = 0; iparam < m_numParameters; ++iparam) {
                simtkParameters[iparam] = double(paramsValue(iparam));
            }
        }

        const auto timesValue = m_opti.value(
                        createTimes(casVars.initialTime, casVars.finalTime));
        SimTK::Vector simtkTimes(m_numTimes);
        for (int itime = 0; itime < m_numTimes; ++itime) {
            simtkTimes[itime] = double(timesValue(itime, 0));
        }
        TOut mucoIterate(simtkTimes,
                m_stateNames, m_controlNames, {}, m_parameterNames,
                simtkStates, simtkControls, {}, simtkParameters);
        return mucoIterate;
    }
    /// Create an initial guess for this problem according to the
    /// following rules:
    ///   - unconstrained variable: 0.
    ///   - lower and upper bounds: midpoint of the bounds.
    ///   - only one bound: value of the bound.
    MucoIterate createInitialGuessFromBounds() const {
        auto setToMidpoint = [](DM& output, const DM& lowerDM,
                const DM& upperDM) {
            for (int irow = 0; irow < output.rows(); ++irow) {
                for (int icol = 0; icol < output.columns(); ++icol) {
                    const auto& lower = double(lowerDM(irow, icol));
                    const auto& upper = double(upperDM(irow, icol));
                    if (!std::isinf(lower) && !std::isinf(upper)) {
                        output(irow, icol) = 0.5 * (upper + lower);
                    }
                    else if (!std::isinf(lower)) output(irow, icol) = lower;
                    else if (!std::isinf(upper)) output(irow, icol) = upper;
                    else output(irow, icol) = 0;
                }
            }
        };
        CasADiVariables<DM> casGuess = m_lowerBounds;
        setToMidpoint(casGuess.initialTime,
                m_lowerBounds.initialTime, m_upperBounds.initialTime);
        setToMidpoint(casGuess.finalTime,
                m_lowerBounds.finalTime, m_upperBounds.finalTime);
        setToMidpoint(casGuess.states,
                m_lowerBounds.states, m_upperBounds.states);
        setToMidpoint(casGuess.controls,
                m_lowerBounds.controls, m_upperBounds.controls);
        setToMidpoint(casGuess.parameters,
                m_lowerBounds.parameters, m_upperBounds.parameters);

        return convertToMuco<DM>(casGuess);
    }
    /// Create a vector with random variable values within the variable
    /// bounds, potentially for use as an initial guess. If, for a given
    /// variable, either bound is infinite, then the element is a random number
    /// in [-1, 1] clamped by the bounds.
    MucoIterate createRandomIterateWithinBounds() const {
        static SimTK::Random::Uniform randGen(-1, 1);
        auto setRandom = [](DM& output, const DM& lowerDM,
                const DM& upperDM) {
            for (int irow = 0; irow < output.rows(); ++irow) {
                for (int icol = 0; icol < output.columns(); ++icol) {
                    const auto& lower = double(lowerDM(irow, icol));
                    const auto& upper = double(upperDM(irow, icol));
                    const auto rand = randGen.getValue();
                    auto value = 0.5 * (rand + 1.0) * (upper - lower) + lower;
                    if (std::isnan(value))
                        value = SimTK::clamp(lower, rand, upper);
                    output(irow, icol) = value;
                }
            }
        };
        CasADiVariables<DM> casIterate = m_lowerBounds;
        setRandom(casIterate.initialTime,
                m_lowerBounds.initialTime, m_upperBounds.initialTime);
        setRandom(casIterate.finalTime,
                m_lowerBounds.finalTime, m_upperBounds.finalTime);
        setRandom(casIterate.states,
                m_lowerBounds.states, m_upperBounds.states);
        setRandom(casIterate.controls,
                m_lowerBounds.controls, m_upperBounds.controls);
        setRandom(casIterate.parameters,
                m_lowerBounds.parameters, m_upperBounds.parameters);

        return convertToMuco<DM>(casIterate);
    }
    void initialize() {
        m_opti = casadi::Opti();

        m_numTimes = m_solver.get_num_mesh_points();

        // Get number and names of variables.
        // ----------------------------------
        // TODO: Add this as a method to MucoProblemRep.
        m_stateNames = createStateVariableNamesInSystemOrder(m_model);
        m_numStates = (int)m_stateNames.size();
        // TODO use getControlNames().
        m_numControls = [&]() {
            int count = 0;
            for (const auto& actuator : m_model.getComponentList<Actuator>()) {
                // TODO check if it's enabled.
                actuator.getName();
                ++count;
            }
            return count;
        }();

        m_parameterNames = m_probRep.createParameterNames();
        m_numParameters = (int)m_parameterNames.size();

        // Create variables and set bounds.
        // --------------------------------
        createVariables();
        auto resizeLike = [](DM& resizeThis, MX& likeThis) {
            resizeThis.resize(likeThis.rows(), likeThis.columns());
        };
        auto initializeBounds = [&](CasADiVariables<DM>& bounds) {
            resizeLike(bounds.initialTime, m_vars.initialTime);
            resizeLike(bounds.finalTime, m_vars.finalTime);
            resizeLike(bounds.states, m_vars.states);
            resizeLike(bounds.controls, m_vars.controls);
            resizeLike(bounds.parameters, m_vars.parameters);
        };
        initializeBounds(m_lowerBounds);
        initializeBounds(m_upperBounds);

        setVariableBounds(
                m_lowerBounds.initialTime,
                m_vars.initialTime,
                m_upperBounds.initialTime, 0, 0,
                m_probRep.getTimeInitialBounds());
        setVariableBounds(
                m_lowerBounds.finalTime,
                m_vars.finalTime,
                m_upperBounds.finalTime, 0, 0,
                m_probRep.getTimeFinalBounds());

        for (int is = 0; is < m_numStates; ++is) {
            const auto& info = m_probRep.getStateInfo(m_stateNames[is]);
            const auto& bounds = info.getBounds();
            auto initialBounds = info.getInitialBounds();
            auto finalBounds = info.getFinalBounds();
            setVariableBounds(
                    m_lowerBounds.states, m_vars.states, m_upperBounds.states,
                    // TODO do not specify bounds twice for endpoints.
                    is, Slice(), bounds);
            setVariableBounds(
                    m_lowerBounds.states, m_vars.states, m_upperBounds.states,
                    is, 0, initialBounds);
            // Last state can be obtained via -1.
            setVariableBounds(
                    m_lowerBounds.states, m_vars.states, m_upperBounds.states,
                    is, -1, finalBounds);
        }

        int ic = 0;
        for (const auto& actuator : m_model.getComponentList<Actuator>()) {
            const auto actuPath = actuator.getAbsolutePathString();
            m_controlNames.push_back(actuPath);
            const auto& info = m_probRep.getControlInfo(actuPath);
            const auto& bounds = info.getBounds();
            const auto& initialBounds = info.getInitialBounds();
            const auto& finalBounds = info.getFinalBounds();
            setVariableBounds(
                    m_lowerBounds.controls, m_vars.controls,
                    m_upperBounds.controls,
                    ic, Slice(), bounds);
            setVariableBounds(
                    m_lowerBounds.controls, m_vars.controls,
                    m_upperBounds.controls,
                    ic, 0, initialBounds);
            setVariableBounds(
                    m_lowerBounds.controls, m_vars.controls,
                    m_upperBounds.controls,
                    ic, -1, finalBounds);
            ++ic;
        }

        for (int iparam = 0; iparam < m_numParameters; ++iparam) {
            const auto& param =
                    m_probRep.getParameter(m_parameterNames[iparam]);
            setVariableBounds(m_lowerBounds.parameters,
                    m_vars.parameters, m_upperBounds.parameters,
                    iparam, 0, param.getBounds());
        }

        m_duration = m_vars.finalTime - m_vars.initialTime;
        DM meshIntervals = m_mesh(Slice(1, m_mesh.rows())) -
                m_mesh(Slice(0, m_mesh.rows() - 1));
        m_times = createTimes(m_vars.initialTime, m_vars.finalTime);

        // Initial guess.
        // TODO m_opti.set_initial(m_vars.parameters, ...);

        addDefectConstraints();

        m_endpointCostFunction =
                make_unique<EndpointCost>("endpoint_cost", *this, m_probRep);

        // TODO: Evaluate individual endpoint costs separately.
        auto endpoint_cost = m_endpointCostFunction->operator()(
                {m_vars.finalTime,
                 m_vars.states(Slice(), -1),
                 m_vars.parameters}).at(0);

        DM quadCoeffs = createIntegralQuadratureCoefficients(meshIntervals);

        m_integrandCostFunction =
                make_unique<IntegrandCost>("integrand", *this, m_probRep);
        MX integral_cost = m_opti.variable();
        integral_cost = 0;
        for (int i = 0; i < m_numTimes; ++i) {
            const auto out = m_integrandCostFunction->operator()(
                    {m_times(i, 0),
                     m_vars.states(Slice(), i),
                     m_vars.controls(Slice(), i),
                     m_vars.parameters});
            integral_cost += quadCoeffs(i) * out.at(0);
        }
        integral_cost *= m_duration;
        m_opti.minimize(endpoint_cost + integral_cost);
    }

    MucoSolution solve() {
        m_opti.solver("ipopt", {},
                {{"hessian_approximation", "limited-memory"}});
        auto casadiSolution = m_opti.solve();
        return convertToMuco<MX, MucoSolution>(m_vars);
    }

    /// @precondition The following are set: m_numTimes, m_numStates,
    ///     m_numControls.
    /// @postcondition All fields in member variable m_vars are set, and
    ///     and m_mesh is set.
    virtual void createVariables() = 0;
    void addDefectConstraints() {
        if (m_numStates) addDefectConstraintsImpl();
    }
    virtual void addDefectConstraintsImpl() = 0;
    virtual DM createIntegralQuadratureCoefficients(const DM& meshIntervals)
    const = 0;

    void applyParametersToModel(const DM& parameters) const
    {
        if (m_numParameters) {
            SimTK::Vector mucoParams(m_numParameters);
            for (int iparam = 0; iparam < m_numParameters; ++iparam) {
                mucoParams[iparam] = double(parameters(iparam));
            }
            m_probRep.applyParametersToModel(mucoParams);
            const_cast<Model&>(m_model).initSystem();
        }
    }

    const MucoCasADiSolver& m_solver;
    const MucoProblemRep& m_probRep;
    const Model& m_model;
    mutable SimTK::State m_state;

protected:
    template <typename TRow, typename TColumn>
    void setVariableBounds(
            DM& lowerBounds, const MX& variable, DM& upperBounds,
            const TRow& rowIndices, const TColumn& columnIndices,
            const MucoBounds& bounds) {
        if (bounds.isSet()) {
            const auto& lower = bounds.getLower();
            lowerBounds(rowIndices, columnIndices) = lower;
            const auto& upper = bounds.getUpper();
            upperBounds(rowIndices, columnIndices) = upper;
            m_opti.subject_to(
                    lower <= variable(rowIndices, columnIndices) <= upper);
        } else {
            lowerBounds(rowIndices, columnIndices) = -SimTK::Infinity;
            upperBounds(rowIndices, columnIndices) =  SimTK::Infinity;
        }
    }
    casadi::Opti m_opti;
    CasADiVariables<casadi::MX> m_vars;
    CasADiVariables<casadi::DM> m_lowerBounds;
    CasADiVariables<casadi::DM> m_upperBounds;
    int m_numTimes = -1;
    int m_numStates = -1;
    int m_numControls = -1;
    int m_numParameters = -1;
    MX m_times;
    MX m_duration;
    DM m_mesh;
    std::vector<std::string> m_stateNames;
    std::vector<std::string> m_controlNames;
    std::vector<std::string> m_parameterNames;

    std::unique_ptr<DynamicsFunction> m_dynamicsFunction;
    std::unique_ptr<EndpointCost> m_endpointCostFunction;
    std::unique_ptr<IntegrandCost> m_integrandCostFunction;
};

#endif // MUSCOLLO_CASADITRANSCRIPTION_H
