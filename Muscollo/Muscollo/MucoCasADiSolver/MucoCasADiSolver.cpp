#include "MucoCasADiSolver.h"/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoCasADiSolver.cpp                                     *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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

#include "MucoCasADiSolver.h"
#include "../MuscolloUtilities.h"

#include <casadi/casadi.hpp>

// TODO
// - get all tests to pass using CasADi, adding in features as necessary.
// - create separate tests for tropter and CasADi.
// - copy TropterSolver options to CasADi (num_mesh_points).
// - initial guess
// - converting an iterate between muco and casadi.
// - creating parameters.
// - compute kinematic diff eqs directly if possible (not finite differences).

//

using casadi::MX;
using casadi::DM;
using casadi::Sparsity;
using casadi::Slice;
using casadi::Callback;
using casadi::Dict;

using namespace OpenSim;

class CasADiTranscription;

class EndpointCost : public Callback {
public:
    EndpointCost(const std::string& name, const MucoProblemRep& problem,
            Dict opts=Dict()) : p(problem) {
        opts["enable_fd"] = true;
        construct(name, opts);
    }
    ~EndpointCost() override {}
    casadi_int get_n_in() override { return 2; }
    casadi_int get_n_out() override { return 1;}
    void init() override {}
    Sparsity get_sparsity_in(casadi_int i) override {
        // TODO fix when using a matrix as input for states.
        // TODO detect this sparsity.
        if (i == 0) return Sparsity::scalar();
        else if (i == 1) return Sparsity::dense(p.getNumStates(), 1);
        else return Sparsity(0, 0);
    }
    Sparsity get_sparsity_out(casadi_int i) override {
        // TODO fix when we have an actual integral cost!
        if (i == 0) return Sparsity::scalar();
        else return Sparsity(0, 0);
    }
    /// Arguments:
    /// 0. time
    /// 1. final state
    /// 2. parameters (TODO)
    std::vector<DM> eval(const std::vector<DM>& arg) const override {
        double time = double(arg.at(0));
        auto state = p.getModel().getWorkingState();
        state.setTime(time);
        for (int i = 0; i < state.getNY(); ++i) {
            state.updY()[i] = double(arg.at(1)(i));
        }
        DM cost = p.calcEndpointCost(state);
        return {cost};
    }
private:
    const MucoProblemRep& p;
};

class IntegrandCost : public Callback {
public:
    IntegrandCost(const std::string& name,
            const CasADiTranscription& transcrip,
            const MucoProblemRep& problem,
            Dict opts=Dict()) : m_transcrip(transcrip), p(problem) {
        opts["enable_fd"] = true;
        construct(name, opts);
    }
    ~IntegrandCost() override {}
    casadi_int get_n_in() override { return 3; }
    casadi_int get_n_out() override { return 1; }
    Sparsity get_sparsity_in(casadi_int i) override {
        if (i == 0) {
            return Sparsity::dense(1, 1);
        } else if (i == 1) {
            return Sparsity::dense(p.getNumStates(), 1);
        } else if (i == 2) {
            return Sparsity::dense(p.getNumControls(), 1);
        } else {
            return Sparsity(0, 0);
        }
    }
    Sparsity get_sparsity_out(casadi_int i) override {
        if (i == 0) return Sparsity::scalar();
        else return Sparsity(0, 0);
    }
    std::vector<DM> eval(const std::vector<DM>& arg) const override;
private:
    const CasADiTranscription& m_transcrip;
    const MucoProblemRep& p;
};

class DynamicsFunction : public Callback {
public:
    DynamicsFunction(const std::string& name,
            const CasADiTranscription& transcrip,
            const MucoProblemRep& problem,
            Dict opts=Dict()) : m_transcrip(transcrip), p(problem) {
        opts["enable_fd"] = true;
        construct(name, opts);
    }
    ~DynamicsFunction() override {}
    casadi_int get_n_in() override { return 3; }
    casadi_int get_n_out() override { return 1; }
    Sparsity get_sparsity_in(casadi_int i) override {
        if (i == 0) {
            return Sparsity::dense(1, 1);
        } else if (i == 1) {
            return Sparsity::dense(p.getNumStates(), 1);
        } else if (i == 2) {
            return Sparsity::dense(p.getNumControls(), 1);
        } else {
            return Sparsity(0, 0);
        }
    }
    Sparsity get_sparsity_out(casadi_int i) override {
        if (i == 0) return Sparsity::dense(p.getNumStates(), 1);
        else return Sparsity(0, 0);
    }
    void init() override {}

    std::vector<DM> eval(const std::vector<DM>& arg) const override;
private:
    const CasADiTranscription& m_transcrip;
    const MucoProblemRep& p;
};

struct CasADiVariables {
    MX initialTime;
    MX finalTime;
    MX states;
    MX controls;
};

class CasADiTranscription {
public:
    CasADiTranscription(const MucoCasADiSolver& solver,
            const MucoProblemRep& probRep)
            : m_solver(solver), m_probRep(probRep),
            m_model(m_probRep.getModel()),
            m_state(m_model.getWorkingState()) {}
    void initialize() {
        m_opti = casadi::Opti();

        m_numTimes = m_solver.get_num_mesh_points();

        // Add this as a method to MucoProblemRep.
        m_stateNames = createStateVariableNamesInSystemOrder(m_model);
        m_numStates = (int)m_stateNames.size();
        m_numControls = [&]() {
            int count = 0;
            for (const auto& actuator : m_model.getComponentList<Actuator>()) {
                // TODO check if it's enabled.
                actuator.getName();
                ++count;
            }
            return count;
        }();

        createVariables();

        m_duration = m_vars.finalTime - m_vars.initialTime;
        DM meshIntervals = m_mesh(Slice(1, m_mesh.rows())) -
                m_mesh(Slice(0, m_mesh.rows() - 1));
        // m_times = MX(m_numTimes, 1);
        // m_times(0) = m_vars.initialTime;
        // for (int itime = 1; itime < m_numTimes - 1; ++itime) {
        //     m_times(itime) = m_times(itime - 1) +
        //             m_duration * meshIntervals(itime - 1);
        // }
        // m_times(m_numTimes - 1) = m_vars.finalTime;
        m_times = m_duration * m_mesh + m_vars.initialTime;

        setVariableBounds(m_vars.initialTime, m_probRep.getTimeInitialBounds());
        setVariableBounds(m_vars.finalTime, m_probRep.getTimeFinalBounds());

        for (int is = 0; is < m_numStates; ++is) {
            const auto& info = m_probRep.getStateInfo(m_stateNames[is]);
            const auto& bounds = info.getBounds();
            const auto& initialBounds = info.getInitialBounds();
            const auto& finalBounds = info.getFinalBounds();
            setVariableBounds(m_vars.states(is, Slice()), bounds);
            setVariableBounds(m_vars.states(is, 0), initialBounds);
            // Last state can be obtained via -1.
            setVariableBounds(m_vars.states(is, -1), finalBounds);
        }

        int ic = 0;
        for (const auto& actuator : m_model.getComponentList<Actuator>()) {
            const auto actuPath = actuator.getAbsolutePathString();
            m_controlNames.push_back(actuPath);
            const auto& info = m_probRep.getControlInfo(actuPath);
            const auto& bounds = info.getBounds();
            const auto& initialBounds = info.getInitialBounds();
            const auto& finalBounds = info.getFinalBounds();
            setVariableBounds(m_vars.controls(ic, Slice()), bounds);
            setVariableBounds(m_vars.controls(ic, 0), initialBounds);
            setVariableBounds(m_vars.controls(ic, -1), finalBounds);
            ++ic;
        }

        addDefectConstraints();

        m_endpointCostFunction =
                make_unique<EndpointCost>("endpoint_cost", m_probRep);

        // TODO: Evaluate individual endpoint costs separately.
        auto endpoint_cost = m_endpointCostFunction->operator()(
                {m_vars.finalTime, m_vars.states(Slice(), -1)}).at(0);

        DM quadCoeffs = createIntegralQuadratureCoefficients(meshIntervals);

        m_integrandCostFunction =
                make_unique<IntegrandCost>("integrand", *this, m_probRep);
        MX integral_cost = m_opti.variable();
        integral_cost = 0;
        for (int i = 0; i < m_numTimes; ++i) {
            const auto out = m_integrandCostFunction->operator()(
                    {m_times(i, 0),
                     m_vars.states(Slice(), i),
                     m_vars.controls(Slice(), i)});
            integral_cost += quadCoeffs(i) * out.at(0);
        }
        integral_cost *= m_duration;
        m_opti.minimize(endpoint_cost + integral_cost);
    }

    MucoSolution solve() {
        m_opti.solver("ipopt", {},
                {{"hessian_approximation", "limited-memory"}});
        auto casadiSolution = m_opti.solve();
        SimTK::Matrix simtkStates;
        if (m_numStates) {
            const auto statesValue = m_opti.value(m_vars.states);
            simtkStates.resize(m_numTimes, m_numStates);
            for (int istate = 0; istate < m_numStates; ++istate) {
                for (int itime = 0; itime < m_numTimes; ++itime) {
                    simtkStates(itime, istate) = double(statesValue(istate, itime));
                }
            }
        }
        SimTK::Matrix simtkControls;
        if (m_numControls) {
            const auto controlsValue = m_opti.value(m_vars.controls);
            simtkControls.resize(m_numTimes, m_numControls);
            for (int icontrol = 0; icontrol < m_numControls; ++icontrol) {
                for (int itime = 0; itime < m_numTimes; ++itime) {
                    simtkControls(itime, icontrol) =
                            double(controlsValue(icontrol, itime));
                }
            }
        }

        const auto timesValue = m_opti.value(m_times);
        SimTK::Vector simtkTimes(m_numTimes);
        for (int itime = 0; itime < m_numTimes; ++itime) {
            simtkTimes[itime] = double(timesValue(itime, 0));
        }
        MucoSolution mucoSolution(simtkTimes,
                m_stateNames, m_controlNames, {}, {},
                simtkStates, simtkControls, {}, {});
        return mucoSolution;
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

    const MucoCasADiSolver& m_solver;
    const MucoProblemRep& m_probRep;
    const Model& m_model;
    mutable SimTK::State m_state;

protected:
    void setVariableBounds(const MX& variable, const MucoBounds& bounds) {
        if (bounds.isSet()) {
            const auto& lower = bounds.getLower();
            const auto& upper = bounds.getUpper();
            m_opti.subject_to(lower <= variable <= upper);
        }
    }
    casadi::Opti m_opti;
    CasADiVariables m_vars;
    int m_numTimes = -1;
    int m_numStates = -1;
    int m_numControls = -1;
    MX m_times;
    MX m_duration;
    DM m_mesh;
    std::vector<std::string> m_stateNames;
    std::vector<std::string> m_controlNames;

    std::unique_ptr<DynamicsFunction> m_dynamicsFunction;
    std::unique_ptr<EndpointCost> m_endpointCostFunction;
    std::unique_ptr<IntegrandCost> m_integrandCostFunction;
};

class CasADiTrapezoidal : public CasADiTranscription {
public:
    using CasADiTranscription::CasADiTranscription;

    DM createIntegralQuadratureCoefficients(const DM& meshIntervals)
            const override {
        DM trapezoidalQuadCoeffs(m_numTimes, 1);
        trapezoidalQuadCoeffs(Slice(0, m_numTimes - 1)) = 0.5 * meshIntervals;
        trapezoidalQuadCoeffs(Slice(1, m_numTimes)) += 0.5 * meshIntervals;
        return trapezoidalQuadCoeffs;
    }

    void addDefectConstraintsImpl() override {

        auto h = m_duration / (m_numTimes - 1);
        m_dynamicsFunction =
                make_unique<DynamicsFunction>("dynamics", *this, m_probRep);

        // Defects.
        MX xdot_im1 = m_dynamicsFunction->operator()(
                {m_vars.initialTime,
                 m_vars.states(Slice(), 0), m_vars.controls(Slice(), 0)}).at(0);
        for (int itime = 1; itime < m_numTimes; ++itime) {
            const auto t = m_times(itime);
            auto x_i = m_vars.states(Slice(), itime);
            auto x_im1 = m_vars.states(Slice(), itime - 1);
            MX xdot_i = m_dynamicsFunction->operator()(
                    {t,
                     m_vars.states(Slice(), itime),
                     m_vars.controls(Slice(), itime)}).at(0);
            m_opti.subject_to(x_i == (x_im1 + 0.5 * h * (xdot_i + xdot_im1)));
            xdot_im1 = xdot_i;
        }
    }
private:
    void createVariables() override {
        m_vars.initialTime = m_opti.variable();
        m_vars.finalTime = m_opti.variable();
        m_vars.states = m_opti.variable(m_numStates, m_numTimes);
        m_vars.controls = m_opti.variable(m_numControls, m_numTimes);
        m_mesh = DM::linspace(0, 1, m_numTimes);
    }

};

std::vector<DM> IntegrandCost::eval(const std::vector<DM>& arg) const {
    auto& state = m_transcrip.m_state;
    double time = double(arg.at(0));
    state.setTime(time);
    for (int i = 0; i < state.getNY(); ++i) {
        state.updY()[i] = double(arg.at(1)(i));
    }
    auto& controls = p.getModel().updControls(state);
    for (int i = 0; i < controls.size(); ++i) {
        controls[i] = double(arg.at(2)(i));
    }
    p.getModel().realizeVelocity(state);
    p.getModel().setControls(state, controls);
    return {p.calcIntegralCost(state)};
}
std::vector<DM> DynamicsFunction::eval(const std::vector<DM>& arg) const {
    auto& state = m_transcrip.m_state;
    // TODO move copying over the state to a separate function.
    double time = double(arg.at(0));
    state.setTime(time);
    for (int i = 0; i < state.getNY(); ++i) {
        state.updY()[i] = double(arg.at(1)(i));
    }
    auto& controls = p.getModel().updControls(state);
    for (int i = 0; i < controls.size(); ++i) {
        controls[i] = double(arg.at(2)(i));
    }
    p.getModel().realizeVelocity(state);
    p.getModel().setControls(state, controls);
    p.getModel().realizeAcceleration(state);
    DM deriv(state.getNY(), 1);
    for (int i = 0; i < state.getNY(); ++i) {
        deriv(i, 0) = state.getYDot()[i];
    }
    return {deriv};
}

MucoCasADiSolver::MucoCasADiSolver() {
    constructProperties();
}

void MucoCasADiSolver::constructProperties() {
    constructProperty_num_mesh_points(100);
}

MucoSolution MucoCasADiSolver::solveImpl() const {
    checkPropertyIsPositive(*this, getProperty_num_mesh_points());
    CasADiTrapezoidal transcription(*this, getProblemRep());
    transcription.initialize();
    // opt.disp(std::cout, true);
    try {
        return transcription.solve();
    } catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
        // TODO: Return a solution.
    }

    // Some useful functions for debugging:
    // opt.debug().show_infeasibilities();
    // DM controlValues = opt.debug().value(controls);
    // std::cout << "DEBUGg43 " << opt.debug().g_describe(43) << std::endl;
    // std::cout << "DEBUGg43 " << opt.g()(43) << std::endl;
    // std::cout << "DEBUGg43 " << opt.x() << std::endl;

    return {};
}

