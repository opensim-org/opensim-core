/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoTropterSolver.cpp                                               *
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
#include "MucoTropterSolver.h"
#include "MucoProblem.h"

#include <tropter/tropter.h>

using namespace OpenSim;

using tropter::VectorX;

/// The map provides the index of each state variable in
/// SimTK::State::getY() from its each state variable path string.
std::vector<std::string> createStateVariableNamesInSystemOrder(
        const Model& model) {
    std::vector<std::string> svNamesInSysOrder;
    auto s = model.getWorkingState();
    const auto svNames = model.getStateVariableNames();
    s.updY() = 0;
    for (int iy = 0; iy < s.getNY(); ++iy) {
        s.updY()[iy] = SimTK::NaN;
        const auto svValues = model.getStateVariableValues(s);
        for (int isv = 0; isv < svNames.size(); ++isv) {
            if (SimTK::isNaN(svValues[isv])) {
                svNamesInSysOrder.push_back(svNames[isv]);
                s.updY()[iy] = 0;
                break;
            }
        }
    }
    SimTK_ASSERT2_ALWAYS((size_t)svNames.size() == svNamesInSysOrder.size(),
            "Expected to get %i state names but found %i.", svNames.size(),
            svNamesInSysOrder.size());
    return svNamesInSysOrder;
}

tropter::Bounds convert(const MucoBounds& mb) {
    return {mb.lower, mb.upper};
}

tropter::InitialBounds convert(const MucoInitialBounds& mb) {
    return {mb.lower, mb.upper};
}
tropter::FinalBounds convert(const MucoFinalBounds& mb) {
    return {mb.lower, mb.upper};
}

template <typename T>
class MucoTropterSolver::OCProblem : public tropter::OptimalControlProblem<T> {
public:
    OCProblem(const MucoSolver& solver)
            // TODO set name properly.
            : tropter::OptimalControlProblem<T>(solver.getProblem().getName()),
              m_mucoSolver(solver),
              m_mucoProb(solver.getProblem()),
              m_phase0(m_mucoProb.getPhase(0)) {
        m_model = m_phase0.getModel();
        m_state = m_model.initSystem();

        this->set_time(convert(m_phase0.getTimeInitialBounds()),
                convert(m_phase0.getTimeFinalBounds()));
        auto svNamesInSysOrder = createStateVariableNamesInSystemOrder(m_model);
        for (const auto& svName : svNamesInSysOrder) {
            const auto& info = m_phase0.getStateInfo(svName);
            this->add_state(svName, convert(info.getBounds()),
                    convert(info.getInitialBounds()),
                    convert(info.getFinalBounds()));
        }
        for (const auto& actu : m_model.getComponentList<Actuator>()) {
            // TODO handle a variable number of control signals.
            const auto& actuName = actu.getName();
            const auto& info = m_phase0.getControlInfo(actuName);
            this->add_control(actuName, convert(info.getBounds()),
                    convert(info.getInitialBounds()),
                    convert(info.getFinalBounds()));
        }
    }
    // TODO rename argument "states" to "state".
    void calc_differential_algebraic_equations(
            const tropter::DAEInput<T>& in,
            tropter::DAEOutput<T> out) const override {

        // TODO convert to implicit formulation.

        const auto& states = in.states;
        const auto& controls = in.controls;

        m_state.setTime(in.time);

        std::copy(states.data(), states.data() + states.size(),
                &m_state.updY()[0]);
        //
        // TODO do not copy? I think this will still make a copy:
        //m_state.setY(SimTK::Vector(states.size(), states.data(), true));

        auto& osimControls = m_model.updControls(m_state);
        std::copy(controls.data(), controls.data() + controls.size(),
                &osimControls[0]);

        m_model.realizeVelocity(m_state);
        m_model.setControls(m_state, osimControls);

        // TODO Antoine and Gil said realizing Dynamics is a lot costlier than
        // realizing to Velocity and computing forces manually.
        m_model.realizeAcceleration(m_state);
        std::copy(&m_state.getYDot()[0], &m_state.getYDot()[0] + states.size(),
                out.dynamics.data());

    }
    void calc_integral_cost(const T& time,
            const VectorX<T>& states,
            const VectorX<T>& controls, T& integrand) const override {
        m_state.setTime(time);
        std::copy(states.data(), states.data() + states.size(),
                &m_state.updY()[0]);
        auto& osimControls = m_model.updControls(m_state);
        std::copy(controls.data(), controls.data() + controls.size(),
                &osimControls[0]);
        m_model.realizePosition(m_state);
        m_model.setControls(m_state, osimControls);
        integrand = m_phase0.calcIntegralCost(m_state);
    }
    void calc_endpoint_cost(const T& final_time, const VectorX<T>& states,
            T& cost) const override {
        // TODO avoid all of this if there are no endpoint costs.
        m_state.setTime(final_time);
        std::copy(states.data(), states.data() + states.size(),
                &m_state.updY()[0]);
        // TODO cannot use control signals...
        m_model.updControls(m_state).setToNaN();
        cost = m_phase0.calcEndpointCost(m_state);
    }

private:
    const MucoSolver& m_mucoSolver;
    const MucoProblem& m_mucoProb;
    const MucoPhase& m_phase0;
    Model m_model;
    mutable SimTK::State m_state;
};

MucoTropterSolver::MucoTropterSolver() {
    constructProperties();
}

void MucoTropterSolver::constructProperties() {
    constructProperty_num_mesh_points(100);
}

std::shared_ptr<const tropter::OptimalControlProblem<double>>
MucoTropterSolver::getTropterProblem() const {
    if (!m_tropProblem) {
        m_tropProblem = std::make_shared<OCProblem<double>>(*this);
    }
    return m_tropProblem;
}

void MucoTropterSolver::resetProblemImpl() {

}

void MucoTropterSolver::resetProblemImpl(const MucoProblem& /*problem*/) {

}

MucoSolution convert(const tropter::OptimalControlSolution& tropSol) {
    const auto& tropTime = tropSol.time;
    SimTK::Vector time((int)tropTime.size(), tropTime.data());
    const auto& state_names = tropSol.state_names;
    const auto& control_names = tropSol.control_names;

    int numTimes = (int)time.size();
    int numStates = (int)state_names.size();
    int numControls = (int)control_names.size();
    SimTK::Matrix states(numTimes, numStates);
    for (int itime = 0; itime < numTimes; ++itime) {
        for (int istate = 0; istate < numStates; ++istate) {
            states(itime, istate) = tropSol.states(istate, itime);
        }
    }
    SimTK::Matrix controls(numTimes, numControls);
    for (int itime = 0; itime < numTimes; ++itime) {
        for (int icontrol = 0; icontrol < numControls; ++icontrol) {
            controls(itime, icontrol) = tropSol.controls(icontrol, itime);
        }
    }
    return {time, state_names, control_names, states, controls};
}

tropter::OptimalControlIterate convert(const MucoIterate& mucoIter) {
    tropter::OptimalControlIterate tropIter;
    using Eigen::Map;
    using Eigen::RowVectorXd;
    using Eigen::MatrixXd;

    const auto& time = mucoIter.getTime();
    tropIter.time = Map<const RowVectorXd>(&time[0], time.size());

    tropIter.state_names = mucoIter.getStateNames();
    tropIter.control_names = mucoIter.getControlNames();

    int numTimes = (int)time.size();
    int numStates = (int)tropIter.state_names.size();
    int numControls = (int)tropIter.control_names.size();
    const auto& states = mucoIter.getStatesTrajectory();
    const auto& controls = mucoIter.getControlsTrajectory();
    // Muscollo's matrix is numTimes x numStates;
    // tropter's is numStates x numTimes.
    tropIter.states = Map<const MatrixXd>(
            &states(0, 0), numTimes, numStates).transpose();
    tropIter.controls = Map<const MatrixXd>(
            &controls(0, 0), numTimes, numControls).transpose();
    return tropIter;
}

MucoSolution MucoTropterSolver::solveImpl() const {

    auto ocp = getTropterProblem();
    ocp->print_description();

    int N = get_num_mesh_points();
    OPENSIM_THROW_IF_FRMOBJ(N <= 0, Exception,
            "Invalid number of mesh points (" + std::to_string(N) + ")");

    tropter::DirectCollocationSolver<double> dircol(ocp, "trapezoidal", "ipopt",
            N);

    // TODO
    dircol.get_optimization_solver().set_hessian_approximation("limited-memory");

    /*
    using TropterIterate = tropter::OptimalControlIterate;
    tropter::OptimalControlSolution tropterSolution =
            _guess ? dircol.solve(TropterIterate(_guess.getRef()))
                   : dircol.solve();
                   */
    tropter::OptimalControlSolution tropterSolution = dircol.solve();

    dircol.print_constraint_values(tropterSolution);

    return convert(tropterSolution);
}
