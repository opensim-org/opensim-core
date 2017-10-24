#ifndef TROPTER_DIRECTCOLLOCATION_H
// ----------------------------------------------------------------------------
// tropter: DirectCollocation.h
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
#define TROPTER_DIRECTCOLLOCATION_H

#include <tropter/common.h>
#include <tropter/optimization/OptimizationProblem.h>
#include "OptimalControlProblem.h"
#include <fstream>

namespace tropter {

class OptimizationSolver;

/// @ingroup optimalcontrol
struct OptimalControlSolution : public OptimalControlIterate {
    double objective;
    // TODO allow reading, to use this as an initial guess.
    // related class: OptimalControlIterate
};

namespace transcription {

template<typename T>
class Base;

} // namespace transcription

/// @ingroup optimalcontrol
template<typename T>
class DirectCollocationSolver {
public:
    typedef OptimalControlProblem<T> OCProblem;
    DirectCollocationSolver(std::shared_ptr<const OCProblem> ocproblem,
                            const std::string& transcription_method,
                            const std::string& optimization_solver,
                            // TODO remove; put somewhere better.
                            const unsigned& num_mesh_points = 20);
    OptimalControlIterate make_initial_guess_from_bounds() const {
        // We only need this decorator to form an initial guess from the bounds.
        auto decorator = m_transcription->make_decorator();
        return m_transcription->deconstruct_iterate(
                decorator->make_initial_guess_from_bounds());
    }
    /// Get the OptimizationSolver, through which you can query optimizer
    /// settings like maximum number of iterations. This provides only const
    /// access, so it does not let you edit settings of the solver; see the
    /// non-const variant below if you need to change settings.
    const OptimizationSolver& get_optimization_solver() const {
        return *m_optsolver.get();
    }
    /// Get the OptimizationSolver, through which you can set optimizer
    /// settings like maximum number of iterations.
    OptimizationSolver& get_optimization_solver() {
        return *m_optsolver.get();
    }

    /// 0 for silent, 1 for verbose. This setting is copied into the
    /// underlying solver.
    void set_verbosity(int verbosity);
    /// @copydoc set_verbosity()
    int get_verbosity() const { return m_verbosity; }

    /// Solve the problem using an initial guess that is based on the bounds
    /// on the variables.
    OptimalControlSolution solve() const;
    /// Solve the problem using the provided initial guess. See
    /// OptimalControlProblem::set_state_guess() and
    /// OptimalControlProblem::set_control_guess() for help with
    /// creating an initial guess.
    ///
    /// Example:
    /// @code
    /// auto ocp = std::make_shared<MyOCP>();
    /// ...
    /// OptimalControlIterate guess;
    /// guess.time.setLinSpaced(N, 0, 1);
    /// ocp->set_state_guess(guess, "x", RowVectorXd::LinSpaced(N, 0, 1));
    /// ...
    /// OptimalControlSolution solution = dircol.solve(guess);
    /// @endcode
    ///
    /// The guess will be linearly interpolated to have the requested number of
    /// mesh points.
    /// TODO right now, initial_guess.time MUST have equally-spaced intervals.
    // TODO make it even easier to create an initial guess; e.g., creating a
    // guess template.
    OptimalControlSolution solve(const OptimalControlIterate& initial_guess)
            const;

    /// Print the value of constraint vector for the given iterate. This is
    /// helpful for troubleshooting why a problem may be infeasible.
    /// This function will try to give meaningful names to the
    /// elements of the constraint vector.
    void print_constraint_values(const OptimalControlIterate& vars,
                                 std::ostream& stream = std::cout) const;
private:
    OptimalControlSolution solve_internal(Eigen::VectorXd& variables) const;
    std::shared_ptr<const OCProblem> m_ocproblem;
    // TODO perhaps ideally DirectCollocationSolver would not be templated?
    std::unique_ptr<transcription::Base<T>> m_transcription;
    std::unique_ptr<OptimizationSolver> m_optsolver;

    int m_verbosity = 1;
};

//namespace transcription {
//
//class Trapezoidal;
//
//class Orthogonal;
//
//class Pseudospectral;
//}

//template<typename T>
//class TrapezoidalTranscription : public OptimizationProblem<T> {
//
//};

//template<typename T>
//class Trapezoidal : public OptimizationProblem<T> {
//public:
//    struct Trajectory {
//        Eigen::RowVectorXd time;
//        Eigen::MatrixXd states;
//        Eigen::MatrixXd controls;
//    };
//protected:
//    virtual void integrate_integral_cost(const VectorX<T>& integrands,
//            T& integral) const = 0;
//    virtual void compute_defects(const StatesView& states,
//            const MatrixX<T>& derivs,
//            DefectsTrajectoryView& defects);
//};

namespace transcription {

/// @ingroup optimalcontrol
template<typename T>
class Base : public OptimizationProblem<T> {
public:
    // TODO do we still need this type? Use OptimalControlIterate instead.
    //struct Trajectory {
    //    Eigen::RowVectorXd time;
    //    Eigen::MatrixXd states;
    //    Eigen::MatrixXd controls;
    //};

    /// Create a vector of optimization variables (for the generic
    /// optimization problem) from an states and controls.
    virtual Eigen::VectorXd
    construct_iterate(const OptimalControlIterate&,
                      bool interpolate = false) const = 0;
    // TODO change interface to be a templated function so users can pass in
    // writeable blocks of a matrix.
    virtual OptimalControlIterate
    deconstruct_iterate(const Eigen::VectorXd& x) const = 0;

    /// Print the value of constraint vector for the given iterate. This is
    /// helpful for troubleshooting why a problem may be infeasible.
    /// This function will try to give meaningful names to the
    /// elements of the constraint vector.
    virtual void print_constraint_values(
            const OptimalControlIterate&,
            std::ostream& stream = std::cout) const {
        stream << "The function print_constraint_values() is unimplemented for "
                "this transcription method." << std::endl;
    }
};

/// @ingroup optimalcontrol
template<typename T>
class Trapezoidal : public Base<T> {
public:
    typedef OptimalControlProblem<T> OCProblem;

    // TODO why would we want a shared_ptr? A copy would use the same Problem.
    // TODO const OCProblem?
    Trapezoidal(std::shared_ptr<const OCProblem> ocproblem,
            unsigned num_mesh_points = 50) {
        set_num_mesh_points(num_mesh_points);
        set_ocproblem(ocproblem);
    }
    // TODO order of calls?
    // TODO right now, must call this BEFORE set_problem.
    void set_num_mesh_points(unsigned N) { m_num_mesh_points = N; }
    void set_ocproblem(std::shared_ptr<const OCProblem> ocproblem);

    void calc_objective(const VectorX<T>& x, T& obj_value) const override;
    void calc_constraints(const VectorX<T>& x,
            Eigen::Ref<VectorX<T>> constr) const override;

    /// This function checks the dimensions of the matrices in traj.
    Eigen::VectorXd
    construct_iterate(const OptimalControlIterate& traj,
                      bool interpolate = false) const override;
    // TODO can this have a generic implementation in the Base class?
    OptimalControlIterate
    deconstruct_iterate(const Eigen::VectorXd& x) const override;
    void print_constraint_values(
            const OptimalControlIterate& vars,
            std::ostream& stream = std::cout) const override;

protected:
    /// Eigen::Map is a view on other data, and allows "slicing" so that we can
    /// view part of the vector of unknowns as a matrix of (num_states x
    /// num_mesh_points).
    /// The second template argument specifies memory alignment; the default is
    /// Unaligned.
    /// The third template argument allows us to specify a stride so that we can
    /// skip over the elements that are the control values (or the state
    /// values, if we seek the trajectory of the controls). The value of the
    /// outer stride is dynamic, since we do not know it at compile-time.
    /// The "outer" stride is the distance between columns and "inner" stride is
    /// the distance between elements of each column (for column-major format).
    template<typename S>
    using TrajectoryViewConst = Eigen::Map<const MatrixX<S>,
                                           Eigen::Unaligned,
                                           Eigen::OuterStride<Eigen::Dynamic>>;
    template<typename S>
    using TrajectoryView = Eigen::Map<MatrixX<S>,
                                      Eigen::Unaligned,
                                      Eigen::OuterStride<Eigen::Dynamic>>;
    // TODO move to a single "make_variables_view"
    template<typename S>
    TrajectoryViewConst<S>
    make_states_trajectory_view(const VectorX<S>& variables) const;
    template<typename S>
    TrajectoryViewConst<S>
    make_controls_trajectory_view(const VectorX<S>& variables) const;
    template<typename S>
    // TODO find a way to avoid these duplicated functions, using SFINAE.
    /// This provides a view to which you can write.
    TrajectoryView<S>
    make_states_trajectory_view(VectorX<S>& variables) const;
    /// This provides a view to which you can write.
    template<typename S>
    TrajectoryView<S>
    make_controls_trajectory_view(VectorX<S>& variables) const;

    // TODO templatize.
    using StatesView = Eigen::Map<VectorX<T>>;
    using ControlsView = Eigen::Map<VectorX<T>>;
    using DefectsTrajectoryView = Eigen::Map<MatrixX<T>>;
    using PathConstraintsTrajectoryView = Eigen::Map<MatrixX<T>>;

    struct ConstraintsView {
        ConstraintsView(StatesView is, ControlsView ic,
                        StatesView fs, ControlsView fc, DefectsTrajectoryView d,
                        PathConstraintsTrajectoryView pc)
                : initial_states(is), initial_controls(ic),
                  final_states(fs), final_controls(fc), defects(d),
                  path_constraints(pc) {}
        StatesView initial_states = {nullptr, 0};
        ControlsView initial_controls = {nullptr, 0};
        StatesView final_states = {nullptr, 0};
        ControlsView final_controls = {nullptr, 0};
        // TODO what is the proper name for this? dynamic defects?
        DefectsTrajectoryView defects = {nullptr, 0, 0};
        PathConstraintsTrajectoryView path_constraints = {nullptr, 0, 0};
    };

    ConstraintsView
    make_constraints_view(Eigen::Ref<VectorX<T>> constraints) const;

private:

    std::shared_ptr<const OCProblem> m_ocproblem;
    int m_num_mesh_points;
    int m_num_time_variables = -1;
    int m_num_defects = -1;
    int m_num_states = -1;
    int m_num_controls = -1;
    int m_num_continuous_variables = -1;
    int m_num_dynamics_constraints = -1;
    int m_num_path_constraints = -1;
    // TODO these should go eventually:
    //double m_initial_time = -1;
    //double m_final_time = -1;
    Eigen::VectorXd m_trapezoidal_quadrature_coefficients;

    // Working memory.
    mutable VectorX<T> m_integrand;
    mutable MatrixX<T> m_derivs;
};

} // namespace transcription
} // namespace tropter

#endif // TROPTER_DIRECTCOLLOCATION_H
