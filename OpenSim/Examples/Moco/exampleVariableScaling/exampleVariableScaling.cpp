/* -------------------------------------------------------------------------- *
 * OpenSim Moco: exampleVariableScaling.cpp                                   *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2022 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
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

/// This example ... TODO
/// https://web.casadi.org/blog/nlp-scaling/

#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Moco/MocoCasADiSolver/CasOCProblem.h>
#include <OpenSim/Moco/MocoCasADiSolver/CasOCSolver.h>
#include <OpenSim/Moco/MocoCasADiSolver/MocoCasOCProblem.h>

#include <casadi/casadi.hpp>

using casadi::Callback;
using casadi::Dict;
using casadi::DM;
using casadi::MX;
using casadi::Slice;
using casadi::Sparsity;
using CasOC::Bounds;

using namespace OpenSim;

class RocketProblem : public CasOC::Problem {
public:
    RocketProblem() {
        setTimeBounds(Bounds(0.0, 0.0), Bounds(100.0, 100.0));
        addState("height", CasOC::StateType::Coordinate, Bounds(0.0, 1e5), Bounds(0.0, 0.0), Bounds(1e5, 1e5));
        addState("velocity", CasOC::StateType::Speed, Bounds(0.0, 2e3), Bounds(0.0, 0.0), Bounds(0.0, 2e3));
        addState("mass", CasOC::StateType::Auxiliary, {0, 5e5}, {5e5, 5e5}, {0, 5e5});
        addControl("force", Bounds(0.0, 1e8), Bounds(0.0, 1e8), Bounds(0.0, 1e8));
        addCost("effort", 0, 1);
        setDynamicsMode("explicit");
    }
private:
    void calcMultibodySystemExplicit(const ContinuousInput& input,
            bool calcKCErrors, MultibodySystemExplicitOutput& output) const override {

        SimTK::Vector x(3, input.states.ptr(), true);
        SimTK::Vector u(1, input.controls.ptr(), true);

        SimTK::Vector xdot(1, 0.0);
        SimTK::Vector auxdot(1, 0.0);

        xdot[0] = u[0] / x[2] - 9.81;
        auxdot[0] = -u[0] / (300.0 * 9.81);

        std::copy_n(xdot.getContiguousScalarData(), xdot.size(),
                output.multibody_derivatives.ptr());
        std::copy_n(auxdot.getContiguousScalarData(), auxdot.size(),
                output.auxiliary_derivatives.ptr());
    }
    void calcMultibodySystemImplicit(const ContinuousInput& input,
            bool calcKCErrors, MultibodySystemImplicitOutput& output) const override {
        // not implemented
    }
    void calcVelocityCorrection(const double& time,
            const casadi::DM& multibody_states, const casadi::DM& slacks,
            const casadi::DM& parameters,
            casadi::DM& velocity_correction) const override {
        // not implemented
    }

//    void calcCostIntegrand(int costIndex,
//            const ContinuousInput& input, double& integrand) const override {
//        integrand = 0;
//        SimTK::Vector controls(1, input.controls.ptr(), true);
//        integrand = controls[0] * controls[0];
//    }
    void calcCost(int costIndex, const CostInput& input,
            casadi::DM& cost) const override {
        SimTK::Vector simtkCost((int)cost.rows(), cost.ptr(), true);
        SimTK::Vector init_x(3, input.initial_states.ptr(), true);
        SimTK::Vector final_x(3, input.final_states.ptr(), true);
        simtkCost[0] = init_x[2] - final_x[2];

        //simtkCost[0] += 1e-15 * input.integral;
//        std::copy_n(simtkCost.getContiguousScalarData(), simtkCost.size(),
//                    cost.ptr());
    }

};

std::unique_ptr<RocketProblem> createCasOCProblem() {
    return OpenSim::make_unique<RocketProblem>();
}

std::unique_ptr<CasOC::Solver> createCasOCSolver(
        const RocketProblem& casProblem) {
    auto casSolver = OpenSim::make_unique<CasOC::Solver>(casProblem);

    // Set solver options.
    // -------------------
    Dict solverOptions;
    solverOptions["print_user_options"] = "yes";
    solverOptions["print_level"] = 5;
    solverOptions["hessian_approximation"] = "limited-memory";
    solverOptions["max_iter"] = 10000;
    const double tol = 1e-2;
    solverOptions["tol"] = tol;
    solverOptions["dual_inf_tol"] = tol;
    solverOptions["compl_inf_tol"] = tol;
    solverOptions["acceptable_tol"] = tol;
    solverOptions["acceptable_dual_inf_tol"] = tol;
    solverOptions["acceptable_compl_inf_tol"] = tol;
    solverOptions["constr_viol_tol"] = tol;
    solverOptions["acceptable_constr_viol_tol"] = tol;

    casSolver->setSparsityDetection("none");
    casSolver->setSparsityDetectionRandomCount(3);
    casSolver->setFiniteDifferenceScheme("forward");

    Dict pluginOptions;
    pluginOptions["verbose_init"] = true;

    casSolver->setNumMeshIntervals(100);
    casSolver->setTranscriptionScheme("trapezoidal");
    casSolver->setScaleVariablesUsingBounds(false);

    casSolver->setOptimSolver("ipopt");
    casSolver->setParallelism("thread", 10);
    //casSolver->setInterpolateControlMidpoints(true);
    casSolver->setPluginOptions(pluginOptions);
    casSolver->setSolverOptions(solverOptions);
    return casSolver;
}

void solveRocketProblem() {
    std::cout.flush();
    Logger::setLevelString("Debug");
    std::cout << "logger level: "  << Logger::getLevelString() << std::endl;
    auto casProblem = createCasOCProblem();
    auto casSolver = createCasOCSolver(*casProblem);
    auto casGuess = casSolver->createInitialGuessFromBounds();
    auto casSolution = casSolver->solve(casGuess);

    MocoSolution mocoSolution =
            convertToMocoTrajectory<MocoSolution>(casSolution);

    mocoSolution.write("exampleVariableScaling_solution.sto");

}

int main() {

    solveRocketProblem();

    return EXIT_SUCCESS;
}
