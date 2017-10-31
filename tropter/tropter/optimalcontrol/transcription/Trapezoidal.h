#ifndef MUSCOLLO_TRANSCRIPTION_TRAPEZOIDAL_H
#define MUSCOLLO_TRANSCRIPTION_TRAPEZOIDAL_H
// ----------------------------------------------------------------------------
// tropter: Trapezoidal.h
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

#include "Base.h"
#include <tropter/optimalcontrol/OptimalControlProblem.h>

namespace tropter {
namespace transcription {

/// The variables are ordered as follows:
/// @verbatim
/// ti
/// tf
/// states(t=0)
/// controls(t=0)
/// states(t=1)
/// controls(t=1)
/// ...
/// states(t=N)
/// controls(t=N)
/// @endverbatim
///
/// The constraints are ordered as follows:
/// @verbatim
/// defects(interval 1)
/// defects(interval 2)
/// ...
/// defects(interval N)
/// path(t=0)
/// path(t=1)
/// ...
/// path(t=N)
/// @endverbatim
/// @ingroup optimalcontrol
// TODO reorder constraints by pairing up defects and paths at a given time
// point; does this help the sparsity structure?
template<typename T>
class Trapezoidal : public Base<T> {
public:
    typedef OptimalControlProblem<T> OCProblem;

    // TODO why would we want a shared_ptr? A copy would use the same Problem.
    // TODO const OCProblem?
    Trapezoidal(std::shared_ptr<const OCProblem> ocproblem,
            unsigned num_mesh_points = 50) {
        // TODO
        if (std::is_same<T, double>::value) {
            this->set_use_supplied_sparsity_hessian_lagrangian(true);
        }
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
    /// Use knowledge of the repeated structure of the optimization problem
    /// to efficiently determine the sparsity pattern of the entire Hessian.
    /// We only need to perturb the optimal control functions at one mesh point,
    /// not the entire NLP objective and constraint functions.
    void calc_sparsity_hessian_lagrangian(const Eigen::VectorXd& x,
            SymmetricSparsityPattern&,
            SymmetricSparsityPattern&) const override;

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
    using DefectsTrajectoryView = Eigen::Map<MatrixX<T>>;
    using PathConstraintsTrajectoryView = Eigen::Map<MatrixX<T>>;

    struct ConstraintsView {
        ConstraintsView(DefectsTrajectoryView d,
                PathConstraintsTrajectoryView pc)
                : defects(d),
                  path_constraints(pc) {}
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
    Eigen::VectorXd m_trapezoidal_quadrature_coefficients;

    // Working memory.
    mutable VectorX<T> m_integrand;
    mutable MatrixX<T> m_derivs;
};

} // namespace transcription
} // namespace tropter

#endif // MUSCOLLO_TRANSCRIPTION_TRAPEZOIDAL_H
