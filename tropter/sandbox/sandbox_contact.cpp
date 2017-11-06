// ----------------------------------------------------------------------------
// tropter: sandbox_contact.cpp
// ----------------------------------------------------------------------------
// Copyright (c) 2017 tropter authors
//
// Licensed under the Apache License, Version 2.0 (the "License"); you may
// not use this file except in compliance with the License. You may obtain a
// copy of the License at http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ----------------------------------------------------------------------------

#include <tropter/tropter.h>

using namespace tropter;

template<typename T>
class BouncingBallLinear : public OptimalControlProblem<T> {
public:
    const double mass = 50.0; // kg
    const double stiffness = 6000.0; // N/m
    const double stiffness_fictitious = 1.0; // N/m
    const double g = 9.81; // m/s^2
    BouncingBallLinear() {
        this->set_time(0, 1.25);
        this->add_state("y", {-1, 1}, 1);
        this->add_state("vy", {-10, 10}, 0);
    }
    void calc_differential_algebraic_equations(
            const DAEInput<T>& in, DAEOutput<T> out) const override {
        const T& y = in.states[0];
        const T& vy = in.states[1];
        out.dynamics[0] = vy;
        const T ground_height = 0;
        // Positive if penetrated.
        const T penetration = ground_height - y;
        const T penetration_pos = fmax(0, penetration);
        const T contact_normal_force = stiffness * penetration_pos +
                stiffness_fictitious * penetration;
        out.dynamics[1] = -g + (contact_normal_force) / mass;
    }
    static void run() {
        auto ocp = std::make_shared<BouncingBallLinear<T>>();
        const int N = 1000;
        DirectCollocationSolver<T> dircol(ocp, "trapezoidal", "ipopt", N);
        OptimalControlSolution solution = dircol.solve();
        //std::cout << "States: " << solution.states << std::endl;
        solution.write("bouncing_ball_solution.csv");
    }
};

template<typename T>
class BouncingBallAckermann2010 : public OptimalControlProblem<T> {
public:
    const double mass = 50.0; // kg
    double a; // N/m^3
    const double b = 1.0; // s/m TODO unused right now.
    const double stiffness_fictitious = 1.0; // N/m
    const double g = 9.81; // m/s^2
    /// a: N/m^3
    BouncingBallAckermann2010(double a = 5e7) : a(a) {
        this->set_time(0, 1.25);
        this->add_state("y", {-1, 1}, 1);
        this->add_state("vy", {-10, 10}, 0);
    }
    void calc_differential_algebraic_equations(
            const DAEInput<T>& in, DAEOutput<T> out) const override {
        const T& y = in.states[0];
        const T& vy = in.states[1];
        out.dynamics[0] = vy;
        const T ground_height = 0;
        // Positive if penetrated.
        const T penetration = ground_height - y;
        const T penetration_pos = fmax(0, penetration);
        const T contact_normal_force =
                a * pow(penetration_pos, 3) + // TODO add in damping.
                stiffness_fictitious * penetration;
        out.dynamics[1] = -g + (contact_normal_force) / mass;
    }
    static OptimalControlSolution run(std::string suffix, double a,
        const OptimalControlIterate& guess = OptimalControlIterate()) {
        std::cout << std::string(79, '=') << "\n";
        std::cout << suffix << "\n";
        std::cout << std::string(79, '-') << std::endl;
        auto ocp = std::make_shared<BouncingBallAckermann2010<T>>(a);
        const int N = 1000;
        DirectCollocationSolver<T> dircol(ocp, "trapezoidal", "ipopt", N);
        OptimalControlSolution solution = dircol.solve(guess);
        solution.write(
                "bouncing_ball_Ackermann2010_solution" + suffix + ".csv");
        std::cout << std::string(79, '=') << std::endl;
        return solution;
        /*
        OptimalControlSolution solution0;
        {
            auto ocp = std::make_shared<BouncingBallAckermann2010<T>>(5e5);
            const int N = 1000;
            DirectCollocationSolver<T> dircol(ocp, "trapezoidal", "ipopt", N);
            solution0 = dircol.solve();
            //std::cout << "States: " << solution.states << std::endl;
            solution0.write("bouncing_ball_Ackermann2010_solution0.csv");
        }
        // Does a good initial guess reduce the number of iterations? Yes.
        {
            auto ocp = std::make_shared<BouncingBallAckermann2010<T>>(5e5);
            const int N = 1000;
            DirectCollocationSolver<T> dircol(ocp, "trapezoidal", "ipopt", N);
            OptimalControlSolution solution1 = dircol.solve(solution0);
            //std::cout << "States: " << solution.states << std::endl;
            solution1.write("bouncing_ball_Ackermann2010_solution1.csv");
        }
        // Increase stiffness.
        OptimalControlSolution solution2;
        {
            auto ocp = std::make_shared<BouncingBallAckermann2010<T>>(5e6);
            const int N = 1000;
            DirectCollocationSolver<T> dircol(ocp, "trapezoidal", "ipopt", N);
            solution2 = dircol.solve(solution0);
            //std::cout << "States: " << solution.states << std::endl;
            solution2.write("bouncing_ball_Ackermann2010_solution2.csv");
        }
        // Same stiffness, hot start. Much faster!
        {
            auto ocp = std::make_shared<BouncingBallAckermann2010<T>>(5e6);
            const int N = 1000;
            DirectCollocationSolver<T> dircol(ocp, "trapezoidal", "ipopt", N);
            OptimalControlSolution solution3 = dircol.solve(solution2);
            //std::cout << "States: " << solution.states << std::endl;
            solution3.write("bouncing_ball_Ackermann2010_solution3.csv");
        }
        // Bump up the stiffness again.
        OptimalControlSolution solution1e7;
        {
            auto ocp = std::make_shared<BouncingBallAckermann2010<T>>(1e7);
            const int N = 1000;
            DirectCollocationSolver<T> dircol(ocp, "trapezoidal", "ipopt", N);
            solution1e7 = dircol.solve(solution2);
            //std::cout << "States: " << solution.states << std::endl;
            solution1e7.write("bouncing_ball_Ackermann2010_solution4.csv");
        }
        // Bump up the stiffness again.
        {
            auto ocp = std::make_shared<BouncingBallAckermann2010<T>>(2e7);
            const int N = 1000;
            DirectCollocationSolver<T> dircol(ocp, "trapezoidal", "ipopt", N);
            OptimalControlSolution solution5e7 = dircol.solve(solution1e7);
            //std::cout << "States: " << solution.states << std::endl;
            solution5e7.write("bouncing_ball_Ackermann2010_solution5.csv");
        }
    */
    }
};

int main() {
    BouncingBallLinear<adouble>::run();
    // Necessary to perform homotopy with stiffness to obtain a solution.
    // Using the naive guess with a=5e7 does not find a solution.
    auto sol5e5 = BouncingBallAckermann2010<adouble>::run("5e5", 5e5);
    auto sol5e6 = BouncingBallAckermann2010<adouble>::run("5e6", 5e6, sol5e5);
    auto sol1e7 = BouncingBallAckermann2010<adouble>::run("1e7", 1e7, sol5e6);
    auto sol2e7 = BouncingBallAckermann2010<adouble>::run("2e7", 2e7, sol1e7);
    auto sol3e7 = BouncingBallAckermann2010<adouble>::run("3e7", 3e7, sol2e7);
    auto sol4e7 = BouncingBallAckermann2010<adouble>::run("3e7", 4e7, sol3e7);
    auto sol5e7 = BouncingBallAckermann2010<adouble>::run("5e7", 5e7, sol4e7);
}
