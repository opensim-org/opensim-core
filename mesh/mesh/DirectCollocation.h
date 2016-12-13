#ifndef MESH_DIRECTCOLLOCATION_H
#define MESH_DIRECTCOLLOCATION_H

#include "common.h"
#include "OptimizationProblem.h"
#include <Eigen/Dense>
#include <adolc/adolc.h>

namespace mesh {

template<typename T>
class OptimalControlProblem;

template<typename T>
class Transcription : public OptimizationProblem<T> {

};

template<typename T>
class DirectCollocationSolver {
public:
    typedef OptimalControlProblem<T> OCProblem;
    DirectCollocationSolver(std::shared_ptr<const OCProblem> ocproblem);
private:
    std::shared_ptr<const OCProblem> m_ocproblem;
    Transcription<T> m_transcription;
};



template<typename T>
class EulerTranscription : public OptimizationProblem<T> {
    // TODO should this *BE* an OptimizationProblem, or should it just
    // contain one?
public:
    typedef OptimalControlProblem<T> OCProblem;
    struct Trajectory {
        Eigen::RowVectorXd time;
        Eigen::MatrixXd states;
        Eigen::MatrixXd controls;
    };

    // TODO why would we want a shared_ptr? A copy would use the same Problem.
    // TODO const OCProblem?
    EulerTranscription(std::shared_ptr<OCProblem> ocproblem,
            unsigned num_mesh_points = 20) {
        set_num_mesh_points(num_mesh_points);
        set_ocproblem(ocproblem);
    }
    // TODO order of calls?
    // TODO right now, must call this BEFORE set_problem.
    void set_num_mesh_points(unsigned N) { m_num_mesh_points = N; }
    void set_ocproblem(std::shared_ptr<OCProblem> ocproblem);

    void objective(const VectorX<T>& x, T& obj_value) const override;
    void constraints(const VectorX<T>& x,
            Eigen::Ref<VectorX<T>> constr) const override;

    // TODO change interface to be a templated function so users can pass in
    // writeable blocks of a matrix.
    Trajectory interpret_iterate(const Eigen::VectorXd& x) const;

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
    template<typename S>
    TrajectoryView<S>
    make_states_trajectory_view(const VectorX<S>& variables) const;
    template<typename S>
    TrajectoryView<S>
    make_controls_trajectory_view(const VectorX<S>& variables) const;

    using StatesView = Eigen::Map<VectorX<T>>;
    using ControlsView = Eigen::Map<VectorX<T>>;
    using DefectsTrajectoryView = Eigen::Map<MatrixX<T>>;
    struct ConstraintsView {
        ConstraintsView(StatesView is, ControlsView ic, StatesView fs,
                ControlsView fc, DefectsTrajectoryView d)
                : initial_states(is), initial_controls(ic),
                  final_states(fs), final_controls(fc), defects(d) {}
        StatesView initial_states = StatesView(nullptr, 0);
        ControlsView initial_controls = ControlsView(nullptr, 0);
        StatesView final_states = StatesView(nullptr, 0);
        ControlsView final_controls = ControlsView(nullptr, 0);
        // TODO what is the proper name for this? dynamic defects?
        DefectsTrajectoryView defects = DefectsTrajectoryView(nullptr, 0, 0);
    };

    ConstraintsView make_constraints_view(Eigen::Ref<VectorX<T>> constraints)
            const;

private:
//    int state_index(int i_mesh_point, int i_state) const {
//        return i_mesh_point * m_num_continuous_variables + i_state;
//    }
//    int control_index(int i_mesh_point, int i_control) const {
//        return i_mesh_point * m_num_continuous_variables
//                + i_control + m_num_states;
//    }
//    int constraint_index(int i_mesh, int i_state) const {
//        const int num_bound_constraints = 2 * m_num_continuous_variables;
//        return num_bound_constraints + (i_mesh - 1) * m_num_states + i_state;
//    }
//    enum BoundsCategory {
//        InitialStates   = 0,
//        FinalStates     = 1,
//        InitialControls = 2,
//        FinalControls   = 3,
//    };
//    int constraint_bound_index(BoundsCategory category, int index) const {
//        if (category <= 1) {
//            assert(index < m_num_states);
//            return category * m_num_states + index;
//        }
//        assert(index < m_num_controls);
//        return 2 * m_num_states + (category - 2) * m_num_controls + index;
//    }

    std::shared_ptr<OCProblem> m_ocproblem;
    int m_num_mesh_points = 20;
    int m_num_defects = -1;
    int m_num_states = -1;
    int m_num_controls = -1;
    int m_num_continuous_variables = -1;
    // TODO these should go eventually:
    double m_initial_time = -1;
    double m_final_time = -1;
};

} // namespace mesh

#endif // MESH_DIRECTCOLLOCATION_H
