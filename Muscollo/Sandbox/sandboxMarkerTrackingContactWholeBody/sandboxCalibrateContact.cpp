/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: sandboxCalibrateContact.cpp                              *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco, Chris Dembia                                   *
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

/// Minimize

#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Simulation/osimSimulation.h>
#include <OpenSim/Actuators/osimActuators.h>
#include <OpenSim/Tools/InverseKinematicsTool.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <Muscollo/osimMuscollo.h>

#include <MuscolloSandboxShared.h>

#include <tropter/tropter.h>

using namespace OpenSim;
using Eigen::VectorXd;

template <typename T, typename TStiffness>
T contact_force(const TStiffness& stiffness, const T& y) {
    const double stiffness_fictitious = 1.0; // N/m
    const T ground_height = 0;
    // Positive if penetrated.
    const T depth = ground_height - y;
    const T depth_pos = fmax(0, depth);
    const T contact_normal_force = stiffness * depth_pos +
            stiffness_fictitious * depth;
    return contact_normal_force;
}

template<typename T>
class BouncingBallLinear : public tropter::OptimalControlProblem<T> {
public:
    static const double mass; // kg
    static const double stiffness; // N/m
    static const double g; // m/s^2
    BouncingBallLinear() {
        this->set_time(0, 1.25);
        this->add_state("y", {-1, 1}, 1);
        this->add_state("vy", {-10, 10}, 0);
    }
    void calc_differential_algebraic_equations(
            const tropter::DAEInput<T>& in,
            tropter::DAEOutput<T> out) const override {
        const T& y = in.states[0];
        const T& vy = in.states[1];
        out.dynamics[0] = vy;
        const auto contact_normal_force = contact_force(this->stiffness, y);
        out.dynamics[1] = -g + (contact_normal_force) / mass;
    }
    static tropter::OptimalControlSolution run() {
        auto ocp = std::make_shared<BouncingBallLinear<T>>();
        const int N = 1000;
        tropter::DirectCollocationSolver<T> dircol(ocp, "trapezoidal", "ipopt",
                N);
        tropter::OptimalControlSolution solution = dircol.solve();
        //std::cout << "States: " << solution.states << std::endl;
        //solution.write("sandboxCalibrateContact_bouncing_ball_solution.csv");
        return solution;
    }
};
template <typename T>
const double BouncingBallLinear<T>::mass = 50.0; // kg
template <typename T>
const double BouncingBallLinear<T>::stiffness = 3180.0; // N/m
template <typename T>
const double BouncingBallLinear<T>::g = 9.81; // m/s^2

class BallCalibration : public tropter::OptimizationProblem<double> {
public:
    BallCalibration(Eigen::VectorXd yTraj, Eigen::VectorXd contactForceTraj) :
            tropter::OptimizationProblem<double>(1, 0),
            m_yTraj(yTraj), m_contactForceTraj(contactForceTraj) {
        Eigen::VectorXd lower(1); lower[0] = 0;
        Eigen::VectorXd upper(1); upper[0] = 10000;
        set_variable_bounds(lower, upper);
    }
    void calc_objective(const VectorXd& x, double& obj_value) const override {

        const auto& stiffness = x[0];
        obj_value = 0;
        for (int it = 0; it < m_yTraj.size(); ++it) {
            const auto& y = m_yTraj[it];
            const auto simContactForce = contact_force(stiffness, y);
            const auto& expContactForce = m_contactForceTraj[it];
            // TODO normalize.
            obj_value += pow(simContactForce - expContactForce, 2);
        }
    }
private:
    Eigen::VectorXd m_yTraj;
    Eigen::VectorXd m_contactForceTraj;
};

class ContactCalibration : public tropter::OptimizationProblem<double> {
public:
    ContactCalibration() : tropter::OptimizationProblem<double>() {
        // Kinematics data.

        // Foot–ground contact data.
        auto data = STOFileAdapter::read("walk_gait1018_subject01_grf.mot");
        auto time = data.getIndependentColumn();
        SimTK::Vector Fx = data.getDependentColumn("ground_force_vx");
        SimTK::Vector Fy = data.getDependentColumn("ground_force_vy");
        GCVSpline FxSpline(5, (int)time.size(), time.data(), &Fx[0]);
        GCVSpline FySpline(5, (int)time.size(), time.data(), &Fy[0]);

    }

    void calc_objective(const VectorXd& x, double& obj_value) const override {
        obj_value = 0; // TODO
    }
};

void calibrateBall() {
    const auto exp = BouncingBallLinear<adouble>::run();

    Eigen::VectorXd Fy(exp.time.size());
    for (int it = 0; it < exp.time.size(); ++it) {
        Fy[it] = contact_force(BouncingBallLinear<double>::stiffness,
                exp.states(0, it));
    }

    BallCalibration problem(exp.states.row(0).transpose(), Fy);
    tropter::IPOPTSolver solver(problem);
    solver.set_verbosity(1);
    auto solution = solver.optimize();
    std::cout << solution.variables << std::endl;
}


int main() {

    calibrateBall();

    return EXIT_SUCCESS;
}

