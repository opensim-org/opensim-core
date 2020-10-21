// ----------------------------------------------------------------------------
// tropter: sliding_mass.cpp
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

int main() {
    Problem p;
    StateVariable x = p.add_state("x", 0, 5);
    StateVariable u = p.add_state("u", -10, 10);
    StateVariable F = p.add_control("F", -50, 50);
}

struct States {
    State x = {0, 5};
    State u = {-10, 10};
};

struct Controls {
    Control F = {-50, 50};
};


struct States {
    State x = { {0, 5}, {0}, {1} };
    State u = { {-10, 10}, {0}, {0} };
};

struct Controls {
    Control F = {-50, 50};
};

struct States {
    State x = { Bounds(0, 5), InitialBounds(0), FinalBounds(1) };
    State u = { Bounds(-10, 10), InitialBounds{0}, FinalBounds{0} };
};

struct Controls {
    Control F = {-50, 50};
};

struct States {
    State x;
    State u;
};

struct Controls {
    Control F;
};

int main() {

    Problem<States,Controls> problem;
    problem.states.x.set_bounds(0, 5);
    problem.states.x.set_initial_bounds(0);
    problem.states.x.set_final_bounds(1);

    /////

    Problem prob;
    State x = prob.add_state("x");
    x.set_bounds(0, 5);
    x.set_initial_bounds(0);
    x.set_final_bounds(1);
    x.set_scale_factors()

    State u = prob.add_state("u");
    u.set_bounds(-10, 10);
    u.set_initial_bounds(0);
    u.set_final_bounds(0);

    Control F = prob.add_control("F");
    F.set_bounds(-50, 50);

    prob.set_initial_time_bounds(0);
    prob.set_final_time_bounds(2);

    //prob.set_dynamics_function([](StatesVector& states,
    //                              ControlsVector& controls) -> StatesDerivativeVector
    //prob.set_dynamics([](const double& time,
    //                     const StateVector& states,
    //                     const ControlVector& controls,
    //                     StateDerivativeVector& derivatives) {
    //    derivatives["x"] = states["x"];
    //    derivatives["u"] = controls["F"];
    //});

    //prob.set_integral_cost([](const double& time,
    //                          const StateVector& final_states,
    //                          const ControlVector& final_controls,
    //                          adouble& integrand) {
    //    integrand = final_controls["F"] * final_controls["F"];
    //});

    prob.set_dynamics([](const double& time,
                         const StateVector& states,
                         const ControlVector& controls,
                         StateDerivativeVector& derivatives) {
        d.x = states.x;
        d.u = controls.F;
    });

    prob.set_integral_cost([](const double& time,
                              const StateVector& final_states,
                              const ControlVector& final_controls,
                              adouble& integrand) {
        integrand = final_controls.F * final_controls.F;
    });
    // TODO is it necesasry to bound state variables?

    DirectCollocationSolver solver(problem);
    solver.set_initial_guess() // TODO should be optional.
    Output = solver.solve();


}


// COMBINE variables and the dynamics / constraints:
// Problem consists of components,
// each of which can have states/dynamics, path constraints, etc.

class System {
    variables();
    dynamics();
    path();
};








