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
class BouncingBallLinear : public Problem<T> {
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
            const Input<T>& in, Output<T> out) const override {
        const T& y = in.states[0];
        const T& vy = in.states[1];
        out.dynamics[0] = vy;
        const T ground_height = 0;
        // Positive if penetrated.
        const T depth = ground_height - y;
        const T depth_pos = fmax(0, depth);
        const T contact_normal_force = stiffness * depth_pos +
                stiffness_fictitious * depth;
        out.dynamics[1] = -g + (contact_normal_force) / mass;
    }
    static void run() {
        auto ocp = std::make_shared<BouncingBallLinear<T>>();
        const int N = 1000;
        DirectCollocationSolver<T> dircol(ocp, "trapezoidal", "ipopt", N);
        Solution solution = dircol.solve();
        //std::cout << "States: " << solution.states << std::endl;
        solution.write("bouncing_ball_solution.csv");
    }
};

void condassign(double& a, const double& b, const double& c, const double& d) {
    a = (b > 0) ? c : d;
}

template<typename T>
class BouncingBallAckermann2010 : public Problem<T> {
public:
    const double mass = 50.0; // kg
    double a; // N/m^3
    const double b; // s/m
    const double stiffness_fictitious = 1.0; // N/m
    const double g = 9.81; // m/s^2
    /// a: N/m^3
    BouncingBallAckermann2010(double a = 5e7, double b = 0) : a(a), b(b) {
        this->set_time(0, 1.25);
        this->add_state("y", {-1, 1}, 1);
        this->add_state("vy", {-10, 10}, 0);
    }
    void calc_differential_algebraic_equations(
            const Input<T>& in, Output<T> out) const override {
        const T& y = in.states[0];
        const T& vy = in.states[1];
        out.dynamics[0] = vy;
        const T ground_height = 0;
        // Positive if penetrated.
        const T depth = ground_height - y;
        const T depth_pos = fmax(0, depth);
        const T depth_rate = -vy;
        const T damping_force = 0;
        //const T contact_normal_force =
        //        fmax(0, a * pow(depth_pos, 3) * (1 + b * depth_rate)) +
        //        stiffness_fictitious * depth;
        // Must use this condassign because if depth < 0 (no contact), we can
        // still generate a positive normal force if the depth_rate is
        // positive, as is the case when the mass is falling.
        // condassign(a, b, c, d) is the same as `a = if (b > 0) ? c : d`.
        T physical_normal_force;
        condassign(physical_normal_force, depth,
                fmax(0, a * pow(depth, 3) * (1 + b * depth_rate)),
                T(0.0));
        const T contact_normal_force = physical_normal_force +
                stiffness_fictitious * depth;
        out.dynamics[1] = -g + (contact_normal_force) / mass;
    }
    static Solution run(std::string suffix, double a,
            const Iterate& guess = Iterate(),
            double b = 0,
            std::string hessian_approx = "exact") {
        std::cout << std::string(79, '=') << "\n";
        std::cout << suffix << "\n";
        std::cout << std::string(79, '-') << std::endl;
        auto ocp = std::make_shared<BouncingBallAckermann2010<T>>(a, b);
        const int N = 1000;
        DirectCollocationSolver<T> dircol(ocp, "trapezoidal", "ipopt", N);
        dircol.get_opt_solver().set_hessian_approximation(hessian_approx);
        Solution solution = dircol.solve(guess);
        dircol.print_constraint_values(solution);
        solution.write(
                "bouncing_ball_Ackermann2010_solution" + suffix + ".csv");
        std::cout << std::string(79, '=') << std::endl;
        return solution;
        /*
        Solution solution0;
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
            Solution solution1 = dircol.solve(solution0);
            //std::cout << "States: " << solution.states << std::endl;
            solution1.write("bouncing_ball_Ackermann2010_solution1.csv");
        }
        // Increase stiffness.
        Solution solution2;
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
            Solution solution3 = dircol.solve(solution2);
            //std::cout << "States: " << solution.states << std::endl;
            solution3.write("bouncing_ball_Ackermann2010_solution3.csv");
        }
        // Bump up the stiffness again.
        Solution solution1e7;
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
            Solution solution5e7 = dircol.solve(solution1e7);
            //std::cout << "States: " << solution.states << std::endl;
            solution5e7.write("bouncing_ball_Ackermann2010_solution5.csv");
        }
    */
    }
};

template<typename T>
class Ball2DAckermann2010 : public Problem<T> {
public:
    const double mass = 50.0; // kg
    double a; // N/m^3
    const double b; // s/m
    const double stiffness_fictitious = 1.0; // N/m
    const double g = 9.81; // m/s^2
    Ball2DAckermann2010(double a = 5e7, double b = 0, double vx0 = 0)
            : a(a), b(b) {
        this->set_time(0, 1/*TODO.25*/);
        this->add_state("x", {0, 5}, 0);
        this->add_state("y", {-1, 1}, 1.0); // TODO used to be 1
        this->add_state("vx", {-10, 10}, vx0);
        this->add_state("vy", {-10, 10}, 0);
    }
    void calc_differential_algebraic_equations(
            const Input<T>& in, Output<T> out) const override {
        const T& x = in.states[0];
        const T& y = in.states[1];
        const T& vx = in.states[2];
        const T& vy = in.states[3];
        out.dynamics[0] = vx;
        out.dynamics[1] = vy;
        const T ground_height = 0;
        // Positive if penetrated.
        const T depth = ground_height - y;
        const T depth_pos = fmax(0, depth);
        const T depth_rate = -vy;
        //const T damping_force = 0;
        /*
                    */
        T physical_normal_force;
        // condassign(a, b, c, d) is the same as `a = if (b > 0) ? c : d`.
        condassign(physical_normal_force, depth,
                fmax(0, a * pow(depth, 3) * (1 + b * depth_rate)),
                T(0.0));
        const T total_normal_force = physical_normal_force +
                    stiffness_fictitious * depth;


        // with vx0!=0, i still get an "exit code 11" with adolc whther using
        // condassign or not.
        /*
        TODO this method for keeping the normal force positive didn't work as
         well as using condassign.
        const T normal_force_stiffness = a * pow(depth_pos, 3);
        // Simbody paper. suggests this fmax(-Fstiff, Fdamp)
        const T normal_force_dissiptation = fmax(
                b * normal_force_stiffness * depth_rate,
                -normal_force_stiffness);
        const T physical_normal_force =
                normal_force_stiffness + normal_force_dissiptation;
        const T total_normal_force =
                physical_normal_force +
                        stiffness_fictitious * depth;
                        */

        // TODO is the x direction drift b/c friction includes the fictitious
        // force???? YES
        const double velocity_scaling_factor = 0.1; // TODO 0.05;
        const T z0 = exp(-vx / velocity_scaling_factor);
        const double coefficient_of_friction = 1.0;
        // TODO does normal force for friction include the soft linear term?
        // TODO only apply force if in contact.
        // TODO normal force should be zero out of contact!!
        // Carmichael says he uses mu = 0.8
        // TODO use total normal force
        // TODO ideally would use physical_normal_force here, but using
        // total_normal_force
        const T friction_force =
                (1 - z0) / (1 + z0) * coefficient_of_friction *
                        total_normal_force; // TODO total_normal_force
        out.dynamics[2] = -friction_force / mass;
//        std::cout << "DEBUG " << in.time << " " << out.dynamics[2] << std::endl;
        out.dynamics[3] = -g + (total_normal_force) / mass;
    }
    static Solution run(std::string suffix, double a,
            const Iterate& guess = Iterate(),
            double b = 0,
            std::string hessian_approx = "exact",
            double vx0 = 0) {
        std::cout << std::string(79, '=') << "\n";
        std::cout << suffix << "\n";
        std::cout << std::string(79, '-') << std::endl;
        auto ocp = std::make_shared<Ball2DAckermann2010<T>>(a, b, vx0);
        const int N = 1000; // TODO can get away with less when T=double, but
        // adolc gives error code 11 if N = 300...
        DirectCollocationSolver<T> dircol(ocp, "trapezoidal", "ipopt", N);
        dircol.get_opt_solver().set_hessian_approximation(hessian_approx);
        Solution solution = dircol.solve(guess);
        dircol.print_constraint_values(solution);
        solution.write("Ball2DAckermann2010" + suffix + ".csv");
        std::cout << std::string(79, '=') << std::endl;
        return solution;
    }
};

int main()
{
    BouncingBallLinear<adouble>::run();
    // Necessary to perform homotopy with stiffness to obtain a solution.
    // Using the naive guess with a=5e7 does not find a solution.
    auto sol5e5 = BouncingBallAckermann2010<adouble>::run("5e5", 5e5);
    auto sol5e6 = BouncingBallAckermann2010<adouble>::run("5e6", 5e6, sol5e5);
    /*
    auto sol1e7 = BouncingBallAckermann2010<adouble>::run("1e7", 1e7, sol5e6);
    auto sol2e7 = BouncingBallAckermann2010<adouble>::run("2e7", 2e7, sol1e7);
    auto sol3e7 = BouncingBallAckermann2010<adouble>::run("3e7", 3e7, sol2e7);
    // These next two problems actually solve really fast:
    auto sol4e7 = BouncingBallAckermann2010<adouble>::run("3e7", 4e7, sol3e7);
    auto sol5e7 = BouncingBallAckermann2010<adouble>::run("5e7", 5e7, sol4e7);
    */

    // ***** DAMPING makes homotopy unnecessary; with b=1.0, the ball sticks
    // to the ground. *****
    /*
    auto s5e5d = BouncingBallAckermann2010<adouble>::run(
            "5e5_damped", 5e5, {}, 1.0);
    auto s5e6d = BouncingBallAckermann2010<adouble>::run(
            "5e5_damped", 5e6, s5e5d, 1.0);
    auto s5e7d = BouncingBallAckermann2010<adouble>::run(
            "5e7_damped", 5e7, s5e6d, 1.0);
    */
    BouncingBallAckermann2010<adouble>::run(
            "5e7_damped_cold", 5e7, {}, 1.0);

    // Changing mass from 50 kg to 25 kg to 100 kg had little effect on the
    // solution or convergence rate. It takes 3 or so more iterations with a
    // lighter mass, but the solutions are qualitatively the same.



    /*
    // Using doubles gives the same solutions, qualitatively at least, and
    // the 5e5 solution takes just as many iterations about, but the 5e7
    // struggled quite a bit but still found a solution (IPOPT went into
    // restoration mode). Finite differences takes a lot more time per
    // iteration than ADOLC.
    auto sold5e5d = BouncingBallAckermann2010<double>::run(
            "double5e5_damped", 5e5, {}, 1.0);
    // 88 iterations:
    //BouncingBallAckermann2010<double>::run(
    //        "double5e7_damped_cold", 5e7, {}, 1.0);
    // Giving a hot start helps a lot (13 iterations vs 88 for the cold start)
    // with finite differences.
    BouncingBallAckermann2010<double>::run(
            "double5e7_damped", 5e7, sold5e5d, 1.0);
    */

    // Do not need a hot start if using quasi-Newton.
    BouncingBallAckermann2010<double>::run(
            "double5e7_damped_quasinewton", 5e7, {}, 1.0, "limited-memory");


    // 2D
    // --
    // Ensure that dropping the ball straight down still works just fine.
    Ball2DAckermann2010<adouble>::run("vx0=0", 5e7, {}, 1.0, "exact",
            0.0);

    // TODO defect values are often exactly 0 for vy dynamics; how is that
    // possible?

    // TODO adolc issue: "process finished with exit code 11". something with
    // Hessian sparsity?
    // Ball2DAckermann2010<adouble>::run("vx0=0.1", 5e3, {}, 1.0, "exact", 0.1);
    // This works for N = 1000, but not for N = 300, 100, 50..... WHAT!?
    // Perhaps related to use of condassign with fmax?
    // Ball2DAckermann2010<adouble>::run("vx0=0.1", 5e7, {}, 1.0,
    //         "limited-memory", 0.1);


    Ball2DAckermann2010<double>::run("dvx0=0.5", 5e7, {}, 1.0,
            "limited-memory", 0.5);
    Ball2DAckermann2010<double>::run("dvx0=1.0", 5e7, {}, 1.0,
            "limited-memory", 1.0);





    // TODO try using squared depth instead of cubic depth. (see Miller 2015).

    // steps:
    // 1d no damping
    // 1d damping
    // 2d damping but no friction
    // 2d damping and friction
    // pendulum with contact element at tip, bounces into floor.
    // opensim: compare forward simulation with direct collocation for all
    // problems above.
}
