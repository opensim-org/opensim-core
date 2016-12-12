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
class EulerTranscription : public OptimizationProblem<T> {
    // TODO should this *BE* an OptimizationProblem, or should it just
    // contain one?
public:
    typedef OptimalControlProblem<T> OCProblem;

    // TODO why would we want a shared_ptr? A copy would use the same Problem.
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

    struct Trajectory {
        Eigen::RowVectorXd time;
        Eigen::MatrixXd states;
        Eigen::MatrixXd controls;
    };
    // TODO change interface to be a templated function so users can pass in
    // writeable blocks of a matrix.
    Trajectory interpret_iterate(const Eigen::VectorXd& x) const;
private:
    int state_index(int i_mesh_point, int i_state) const {
        return i_mesh_point * m_num_continuous_variables + i_state;
    }
    int control_index(int i_mesh_point, int i_control) const {
        return i_mesh_point * m_num_continuous_variables
                + i_control + m_num_states;
    }
    int constraint_index(int i_mesh, int i_state) const {
        const int num_bound_constraints = 2 * m_num_continuous_variables;
        return num_bound_constraints + (i_mesh - 1) * m_num_states + i_state;
    }
    enum BoundsCategory {
        InitialStates   = 0,
        FinalStates     = 1,
        InitialControls = 2,
        FinalControls   = 3,
    };
    int constraint_bound_index(BoundsCategory category, int index) const {
        if (category <= 1) {
            assert(index < m_num_states);
            return category * m_num_states + index;
        }
        assert(index < m_num_controls);
        return 2 * m_num_states + (category - 2) * m_num_controls + index;
    }

    std::shared_ptr<OCProblem> m_ocproblem;
    int m_num_mesh_points = 20;
    int m_num_states = -1;
    int m_num_controls = -1;
    int m_num_continuous_variables = -1;
    // TODO these should go eventually:
    double m_initial_time = -1;
    double m_final_time = -1;
};

} // namespace mesh

#endif // MESH_DIRECTCOLLOCATION_H
