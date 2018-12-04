/* -------------------------------------------------------------------------- *
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

using casadi::MX;
using casadi::DM;
using casadi::Sparsity;
using casadi::Slice;
using casadi::Callback;
using casadi::Dict;

using namespace OpenSim;

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
    IntegrandCost(const std::string& name, const MucoProblemRep& problem,
            Dict opts=Dict()) : p(problem) {
        opts["enable_fd"] = true;
        construct(name, opts);
    }
    ~IntegrandCost() override {}
    casadi_int get_n_in() override { return 3; }
    casadi_int get_n_out() override { return 1;}
    void init() override {}
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
        // TODO fix when we have an actual integral cost!
        if (i == 0) return Sparsity(1, 1);
        else return Sparsity(0, 0);
    }
    std::vector<DM> eval(const std::vector<DM>& arg) const override {
        double time = double(arg.at(0));
        auto state = p.getModel().getWorkingState();
        state.setTime(time);
        for (int i = 0; i < state.getNY(); ++i) {
            state.updY()[i] = double(arg.at(1)(i));
        }
        auto& controls = p.getModel().updControls(state);
        for (int i = 0; i < controls.size(); ++i) {
            controls[i] = double(arg.at(2)(i));
        }
        return {p.calcIntegralCost(state)};
    }
private:
    const MucoProblemRep& p;
};

class DynamicsFunction : public Callback {
public:
    DynamicsFunction(const std::string& name, const MucoProblemRep& problem,
            Dict opts=Dict()) : p(problem) {
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

    std::vector<DM> eval(const std::vector<DM>& arg) const override {
        double time = double(arg.at(0));
        auto state = p.getModel().getWorkingState();
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
private:
    const MucoProblemRep& p;
};

// TODO:
// - initial guess
// - converting an iterate between muco and casadi.
// - creating parameters.

namespace OpenSim {

class Transcription {
    virtual int getNumTimes() const = 0;
    virtual void setDefectConstraints() const = 0;
};

class Trapezoidal : public Transcription {
    void addDefectConstraints() {

    }
    void getIntegralQuadratureCoefficients() const {

    }

};

struct CasADiVariables {
    MX initialTime;
    MX finalTime;
    MX states;
    MX controls;
};

class MucoCasADiSolverImpl {
public:
    MucoSolution solve() {
        m_opti.solver("ipopt", {}, {{"hessian_approximation", "limited-memory"}});
        auto casadiSolution = m_opti.solve();
        const auto statesValue = m_opti.value(m_vars.states);
        SimTK::Matrix simtkStates(m_numTimes, m_numStates);
        for (int istate = 0; istate < m_numStates; ++istate) {
            for (int itime = 0; itime < m_numTimes; ++itime) {
                simtkStates(itime, istate) = double(statesValue(istate, itime));
            }
        }
        const auto controlsValue = m_opti.value(m_vars.controls);
        SimTK::Matrix simtkControls(m_numTimes, m_numControls);
        for (int icontrol = 0; icontrol < m_numControls; ++icontrol) {
            for (int itime = 0; itime < m_numTimes; ++itime) {
                simtkControls(itime, icontrol) =
                        double(controlsValue(icontrol, itime));
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
    void setVariableBounds(const MX& variable, const MucoBounds& bounds) {
        if (bounds.isSet()) {
            m_opti.subject_to(bounds.getLower() <= variable <= bounds.getUpper());
        }
    }
    MucoCasADiSolverImpl(const MucoCasADiSolver& solver)
            : m_solver(solver) {
        const MucoProblemRep& rep = m_solver.getProblemRep();

        m_opti = casadi::Opti();

        m_numTimes = 20;

        const auto& model = rep.getModel();

        // Add this as a method to MucoProblemRep.
        m_stateNames = createStateVariableNamesInSystemOrder(model);
        m_numStates = (int)m_stateNames.size();
        m_numControls = [&]() {
            int count = 0;
            for (const auto& actuator : model.getComponentList<Actuator>()) {
                // TODO check if it's enabled.
                actuator.getName();
                ++count;
            }
            return count;
        }();

        m_vars.initialTime = m_opti.variable();
        m_vars.finalTime = m_opti.variable();
        MX duration = m_vars.finalTime - m_vars.initialTime;
        MX mesh = MX::linspace(0, 1, m_numTimes);
        m_times = duration * MX::linspace(0, 1, m_numTimes) +
                m_vars.initialTime;
        MX meshIntervals = mesh(Slice(1)) - mesh(Slice(0, -2));

        setVariableBounds(m_vars.initialTime, rep.getTimeInitialBounds());
        setVariableBounds(m_vars.finalTime, rep.getTimeFinalBounds());

        m_vars.states = m_opti.variable(m_numStates, m_numTimes);
        for (int is = 0; is < m_numStates; ++is) {
            const auto& info = rep.getStateInfo(m_stateNames[is]);
            const auto& bounds = info.getBounds();
            const auto& initialBounds = info.getInitialBounds();
            const auto& finalBounds = info.getFinalBounds();
            // All time except initial and final time??
            // opt.subject_to(bounds.getLower() <= states(is, Slice(1, -1)) <= bounds.getUpper());
            setVariableBounds(m_vars.states(is, Slice()), bounds);
            setVariableBounds(m_vars.states(is, 0), initialBounds);
            // Last state can be obtained via -1.
            setVariableBounds(m_vars.states(is, -1), finalBounds);
        }

        m_vars.controls = m_opti.variable(m_numControls, m_numTimes);
        int ic = 0;
        for (const auto& actuator : model.getComponentList<Actuator>()) {
            const auto actuPath = actuator.getAbsolutePathString();
            m_controlNames.push_back(actuPath);
            const auto& info = rep.getControlInfo(actuPath);
            const auto& bounds = info.getBounds();
            const auto& initialBounds = info.getInitialBounds();
            const auto& finalBounds = info.getFinalBounds();
            setVariableBounds(m_vars.controls(ic, Slice()), bounds);
            setVariableBounds(m_vars.controls(ic, 0), initialBounds);
            setVariableBounds(m_vars.controls(ic, -1), finalBounds);
            ++ic;
        }

        auto h = duration / (m_numTimes - 1);
        m_dynamicsFunction = make_unique<DynamicsFunction>("dynamics", rep);

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

        m_endpointCostFunction =
                make_unique<EndpointCost>("endpoint_cost", rep);

        // TODO: Evaluate individual endpoint costs separately.
        auto endpoint_cost = m_endpointCostFunction->operator()(
                {m_vars.finalTime, m_vars.states(Slice(), -1)}).at(0);

        MX trapezoidalQuadCoeffs(m_numTimes, 1);
        trapezoidalQuadCoeffs(Slice(0, -2)) = 0.5 * meshIntervals;
        trapezoidalQuadCoeffs(Slice(1, -1)) += 0.5 * meshIntervals;

        m_integrandCostFunction = make_unique<IntegrandCost>("integrand", rep);
        MX integral_cost = m_opti.variable();
        integral_cost = 0;
        for (int i = 0; i < m_numTimes; ++i) {
            const auto out = m_integrandCostFunction->operator()(
                    {i * h,
                     m_vars.states(Slice(), i),
                     m_vars.controls(Slice(), i)});
            integral_cost += trapezoidalQuadCoeffs(i) * out.at(0);
        }
        integral_cost *= duration;
        m_opti.minimize(endpoint_cost + integral_cost);

    }
private:
    void createVariables() {
        m_vars.initialTime = m_opti.variable();
        m_vars.finalTime = m_opti.variable();
        m_vars.states = m_opti.variable(m_numStates, m_numTimes);
        m_vars.controls = m_opti.variable(m_numControls, m_numTimes);
    }
    const MucoCasADiSolver& m_solver;
    casadi::Opti m_opti;
    CasADiVariables m_vars;
    std::vector<std::string> m_stateNames;
    std::vector<std::string> m_controlNames;
    int m_numTimes = -1;
    int m_numStates = -1;
    int m_numControls = -1;
    MX m_times;

    std::unique_ptr<DynamicsFunction> m_dynamicsFunction;
    std::unique_ptr<EndpointCost> m_endpointCostFunction;
    std::unique_ptr<IntegrandCost> m_integrandCostFunction;

};

} // namespace OpenSim

MucoSolution MucoCasADiSolver::solveImpl() const {
    MucoCasADiSolverImpl solverImpl(*this);
    // opt.disp(std::cout, true);
    try {
        return solverImpl.solve();
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

