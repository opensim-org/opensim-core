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

using namespace casadi;

using namespace OpenSim;

class EndpointCost : public Callback {
public:
    EndpointCost(const std::string& name, const MucoProblemRep& problem,
            const Dict& opts=Dict()) : p(problem) {
        construct(name, opts);
    }
    ~EndpointCost() override {}
    casadi_int get_n_in() override { return 2; }
    casadi_int get_n_out() override { return 1;}
    void init() override {
        std::cout << "initializing object endpointCost" << std::endl;
    }
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

    // Evaluate numerically
    std::vector<DM> eval(const std::vector<DM>& arg) const override {
        double time = double(arg.at(0));
        auto state = p.getModel().getWorkingState();
        state.setTime(time);
        for (int i = 0; i < state.getNY(); ++i) {
            state.updY()[i] = double(arg.at(1)(i));
        }
        // TODO parameters.
        DM cost = p.calcEndpointCost(state);
        return {cost};
    }
private:
    const MucoProblemRep& p;
};

class IntegrandCost : public Callback {
public:
    IntegrandCost(const std::string& name, const MucoProblemRep& problem,
            const Dict& opts=Dict()) : p(problem) {
        construct(name, opts);
    }
    ~IntegrandCost() override {}
    casadi_int get_n_in() override { return 3; }
    casadi_int get_n_out() override { return 1;}
    void init() override {
        std::cout << "initializing object" << std::endl;
    }
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
    // Evaluate numerically
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

class DynamicsFunc : public Callback {
public:
    DynamicsFunc(const std::string& name, const MucoProblemRep& problem,
            const Dict& opts=Dict()) : p(problem) {
        construct(name, opts);
    }
    ~DynamicsFunc() override {}
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
    void init() override {
        std::cout << "initializing object" << std::endl;
    }

    // Evaluate numerically
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

MucoSolution MucoCasADiSolver::solveImpl() const {
    const MucoProblemRep& rep = getProblemRep();

    casadi::Opti opt;
    int N = 20;

    const auto& model = rep.getModel();

    // Add this as a method to MucoProblemRep.
    const auto svNamesInSysOrder =
            createStateVariableNamesInSystemOrder(model);
    const int numStates = (int)svNamesInSysOrder.size();
    // TODO SX t0
    MX tf = opt.variable();

    {
        const auto& finalBounds = rep.getTimeFinalBounds();
        opt.subject_to(finalBounds.getLower() <= tf <= finalBounds.getUpper());
    }

    // TODO create mesh times.
    MX states = opt.variable(numStates, N);

    for (int is = 0; is < numStates; ++is) {
        const auto& info = rep.getStateInfo(svNamesInSysOrder[is]);
        const auto& bounds = info.getBounds();
        const auto& initialBounds = info.getInitialBounds();
        const auto& finalBounds = info.getFinalBounds();
        // All time except initial and final time??
        // opt.subject_to(bounds.getLower() <= states(is, Slice(1, -1)) <= bounds.getUpper());
        opt.subject_to(bounds.getLower() <= states(is, Slice()) <= bounds.getUpper());
        if (initialBounds.isSet()) {
            opt.subject_to(initialBounds.getLower() <= states(is, 0) <= initialBounds.getUpper());
        }
        if (finalBounds.isSet()) {
            // Last state can be obtained via -1.
            opt.subject_to(finalBounds.getLower() <= states(is, -1) <= finalBounds.getUpper());
        }
    }

    const int numControls = [&]() {
        int count = 0;
        for (const auto& actuator : model.getComponentList<Actuator>()) {
            // TODO check if it's enabled.
            actuator.getName();
            ++count;
        }
        return count;
    }();

    MX controls = opt.variable(numControls, N);

    std::vector<std::string> controlNames;
    int ic = 0;
    for (const auto& actuator : model.getComponentList<Actuator>()) {
        const auto actuPath = actuator.getAbsolutePathString();
        controlNames.push_back(actuPath);
        const auto& info = rep.getControlInfo(actuPath);
        const auto& bounds = info.getBounds();
        const auto& initialBounds = info.getInitialBounds();
        const auto& finalBounds = info.getFinalBounds();
        opt.subject_to(bounds.getLower() <= controls(ic, Slice()) <= bounds.getUpper());
        if (initialBounds.isSet()) {
            opt.subject_to(initialBounds.getLower() <= controls(ic, 0) <= initialBounds.getUpper());
        }
        if (finalBounds.isSet()) {
            // Last state can be obtained via -1.
            opt.subject_to(finalBounds.getLower() <= controls(ic, -1) <= finalBounds.getUpper());
        }
        ++ic;
    }

    auto h = tf / (N - 1);
    DynamicsFunc dynamics("dynamics", rep, {{"enable_fd", true}});

    // Defects.
    MX xdot_im1 = dynamics({0, states(Slice(), 0), controls(Slice(), 0)}).at(0);
    for (int itime = 1; itime < N; ++itime) {
        const auto t = itime * h;
        auto x_i = states(Slice(), itime);
        auto x_im1 = states(Slice(), itime - 1);
        MX xdot_i = dynamics({t, states(Slice(), itime), controls(Slice(), itime)}).at(0);
        opt.subject_to(x_i == (x_im1 + 0.5 * h * (xdot_i + xdot_im1)));
        xdot_im1 = xdot_i;
    }

    EndpointCost endpoint_cost_function("endpoint_cost", rep, {{"enable_fd", true}});

    auto endpoint_cost = endpoint_cost_function({tf, states(Slice(), -1)});

    MX mesh = MX::linspace(0, tf, N);
    MX meshIntervals = mesh(Slice(1)) - mesh(Slice(0, -2));
    MX trapezoidalQuadCoeffs(N, 1);
    trapezoidalQuadCoeffs(Slice(0, -2)) = 0.5 * meshIntervals;
    trapezoidalQuadCoeffs(Slice(1, -1)) += 0.5 * meshIntervals;

    IntegrandCost integrand_cost("integrand", rep, {{"enable_fd", true}});
    MX integral = opt.variable();
    integral = 0;
    for (int i = 0; i < N; ++i) {
        const auto out = integrand_cost({i * h, states(Slice(), i), controls(Slice(), i)});
        integral += trapezoidalQuadCoeffs(i) * out.at(0);
    }
    opt.minimize(endpoint_cost.at(0) + integral);
    opt.disp(std::cout, true);
    opt.solver("ipopt", {}, {{"hessian_approximation", "limited-memory"}});
    try {
        auto casadiSolution = opt.solve();
        const auto statesValue = opt.value(states);
        SimTK::Matrix simtkStates(N, numStates);
        for (int istate = 0; istate < numStates; ++istate) {
            for (int itime = 0; itime < N; ++itime) {
                simtkStates(itime, istate) = double(statesValue(istate, itime));
            }
        }
        const auto controlsValue = opt.value(controls);
        SimTK::Matrix simtkControls(N, numControls);
        for (int icontrol = 0; icontrol < numControls; ++icontrol) {
            for (int itime = 0; itime < N; ++itime) {
                simtkControls(itime, icontrol) =
                        double(controlsValue(icontrol, itime));
            }
        }

        MucoSolution mucoSolution(
                OpenSim::createVectorLinspace(N, 0, double(opt.value(tf))),
                svNamesInSysOrder, controlNames, {}, {},
                simtkStates, simtkControls, {}, {});
        return mucoSolution;

    } catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
    }
    DM statesValues = opt.debug().value(states);
    std::cout << "DEBUG states values " << statesValues << std::endl;
    DM controlValues = opt.debug().value(controls);
    std::cout << "DEBUG control values " << controlValues << std::endl;

    // opt.debug().show_infeasibilities();
    // std::cout << "DEBUGg43 " << opt.debug().g_describe(43) << std::endl;
    // std::cout << "DEBUGg44 " << opt.debug().g_describe(44) << std::endl;
    // std::cout << "DEBUGg43 " << opt.g()(43) << std::endl;
    // std::cout << "DEBUGg43 " << opt.x() << std::endl;
    // std::cout << "DEBUGg43 " << opt.g()(44) << std::endl;

    return {};
}

