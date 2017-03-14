#ifndef MESH_DIRECTCOLLOCATION_H
#define MESH_DIRECTCOLLOCATION_H

#include "common.h"
#include "OptimizationProblem.h"
#include <Eigen/Dense>
#include <adolc/adolc.h>
#include <fstream>

namespace mesh {

template<typename T>
class OptimalControlProblem;

class OptimizationSolver;

struct OptimalControlSolution {
    Eigen::RowVectorXd time;
    Eigen::MatrixXd states;
    Eigen::MatrixXd controls;
    std::vector<std::string> state_names;
    std::vector<std::string> control_names;
    double objective;
    // TODO allow reading, to use this as an initial guess.
    // related class: OptimalControlIterate

    void write(const std::string& filepath) const {
        std::ofstream f(filepath);

        // Column headers.
        f << "time";
        if (state_names.size() == size_t(states.rows())) {
            for (int i_state = 0; i_state < states.rows(); ++i_state) {
                f << "," << state_names[i_state];
            }
        } else {
            for (int i_state = 0; i_state < states.rows(); ++i_state) {
                f << ",state" << i_state;
            }
        }
        if (control_names.size() == size_t(controls.rows())) {
            for (int i_control = 0; i_control < controls.rows(); ++i_control) {
                f << "," << control_names[i_control];
            }
        } else {
            for (int i_control = 0; i_control < controls.rows(); ++i_control) {
                f << ",control" << i_control;
            }
        }
        f << std::endl;

        // Data.
        for (int i_mesh = 0; i_mesh < time.size(); ++i_mesh) {
            f << time[i_mesh];
            for (int i_state = 0; i_state< states.rows(); ++i_state) {
                f << "," << states(i_state, i_mesh);
            }
            for (int i_control = 0; i_control< controls.rows(); ++i_control) {
                f << "," << controls(i_control, i_mesh);
            }
            f << std::endl;
        }
        f.close();
    }
};

namespace transcription {

template<typename T>
class Transcription;

}

template<typename T>
class DirectCollocationSolver {
public:
    typedef OptimalControlProblem<T> OCProblem;
    DirectCollocationSolver(std::shared_ptr<const OCProblem> ocproblem,
                            const std::string& transcription_method,
                            const std::string& optimization_solver,
                            // TODO remove; put somewhere better.
                            const unsigned& num_mesh_points = 20);
    OptimalControlSolution solve() const;
    // Solution solve(const OptimalControlIterate& initial_guess);
private:
    std::shared_ptr<const OCProblem> m_ocproblem;
    // TODO perhaps ideally DirectCollocationSolver would not be templated?
    std::unique_ptr<transcription::Transcription<T>> m_transcription;
    std::unique_ptr<OptimizationSolver> m_optsolver;
};

//namespace transcription {
//
//class LowOrder;
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
//class LowOrder : public OptimizationProblem<T> {
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

template<typename T>
class Transcription : public OptimizationProblem<T> {
public:
    // TODO do we still need this type?
    struct Trajectory {
        Eigen::RowVectorXd time;
        Eigen::MatrixXd states;
        Eigen::MatrixXd controls;
    };

    // TODO change interface to be a templated function so users can pass in
    // writeable blocks of a matrix.
    virtual Trajectory interpret_iterate(const Eigen::VectorXd& x) const = 0;
};

template<typename T>
class LowOrder : public Transcription<T> {
    // TODO should this *BE* an OptimizationProblem, or should it just
    // contain one?
public:
    typedef OptimalControlProblem<T> OCProblem;

    // TODO why would we want a shared_ptr? A copy would use the same Problem.
    // TODO const OCProblem?
    LowOrder(std::shared_ptr<const OCProblem> ocproblem,
            unsigned num_mesh_points = 50) {
        set_num_mesh_points(num_mesh_points);
        set_ocproblem(ocproblem);
    }
    // TODO order of calls?
    // TODO right now, must call this BEFORE set_problem.
    void set_num_mesh_points(unsigned N) { m_num_mesh_points = N; }
    void set_ocproblem(std::shared_ptr<const OCProblem> ocproblem);

    void objective(const VectorX<T>& x, T& obj_value) const override;
    void constraints(const VectorX<T>& x,
            Eigen::Ref<VectorX<T>> constr) const override;

    // TODO can this have a generic implementation in the Transcription class?
    typename Transcription<T>::Trajectory interpret_iterate(
            const Eigen::VectorXd& x) const override;

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
    using TrajectoryView = Eigen::Map<const MatrixX<S>,
                                      Eigen::Unaligned,
                                      Eigen::OuterStride<Eigen::Dynamic>>;
    // TODO move to a single "make_variables_view"
    template<typename S>
    TrajectoryView<S>
    make_states_trajectory_view(const VectorX<S>& variables) const;
    template<typename S>
    TrajectoryView<S>
    make_controls_trajectory_view(const VectorX<S>& variables) const;

    using StatesView = Eigen::Map<VectorX<T>>;
    using ControlsView = Eigen::Map<VectorX<T>>;
    using DefectsTrajectoryView = Eigen::Map<MatrixX<T>>;
    using PathConstraintsTrajectoryView = Eigen::Map<MatrixX<T>>;
    struct ConstraintsView {
        ConstraintsView(StatesView is, ControlsView ic, StatesView fs,
                        ControlsView fc, DefectsTrajectoryView d,
                        PathConstraintsTrajectoryView pc)
                : initial_states(is), initial_controls(ic),
                  final_states(fs), final_controls(fc), defects(d),
                  path_constraints(pc) {}
        StatesView initial_states = StatesView(nullptr, 0);
        ControlsView initial_controls = ControlsView(nullptr, 0);
        StatesView final_states = StatesView(nullptr, 0);
        ControlsView final_controls = ControlsView(nullptr, 0);
        // TODO what is the proper name for this? dynamic defects?
        DefectsTrajectoryView defects = DefectsTrajectoryView(nullptr, 0, 0);
        PathConstraintsTrajectoryView path_constraints =
                PathConstraintsTrajectoryView(nullptr, 0, 0);
    };

    ConstraintsView make_constraints_view(Eigen::Ref<VectorX<T>> constraints)
    const;

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
};

} // namespace transcription
} // namespace mesh

#endif // MESH_DIRECTCOLLOCATION_H
