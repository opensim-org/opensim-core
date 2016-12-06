#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>
#include <adolc/adolc.h>
#include <adolc/sparse/sparsedrivers.h>
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Ipopt::Index;
using Ipopt::Number;

// http://www.coin-or.org/Ipopt/documentation/node23.html

// TODO create my own "NonnegativeIndex" or Count type.

// TODO use faster linear solvers from Sherlock cluster.

// TODO want to abstract optimization problem away from IPOPT.

class OptimizationProblem {
    virtual void objective() = 0;
    virtual void constraints() = 0;
};

// TODO templatize Problem.

// TODO interface 0: (inheritance)
// derive from Problem, implement virtual functions
// interface 1: (composition)
// composed of Variables, Controls, Goals, etc.
/*
std::unordered_map<std::string, Goal> m_goals;
std::unordered_map<std::string
 * */

class IpoptADOLC_OptimizationProblem : public Ipopt::TNLP {
private:
    unsigned m_num_variables = -1;
    unsigned m_num_constraints = -1;
    std::vector<double> m_variable_lower_bounds;
    std::vector<double> m_variable_upper_bounds;
    std::vector<double> m_constraint_lower_bounds;
    std::vector<double> m_constraint_upper_bounds;
    std::vector<double> m_initial_guess;
    unsigned m_hessian_num_nonzeros = -1;
    std::vector<unsigned int> m_hessian_row_indices;
    std::vector<unsigned int> m_hessian_col_indices;
    unsigned m_jacobian_num_nonzeros = -1;
    std::vector<unsigned int> m_jacobian_row_indices;
    std::vector<unsigned int> m_jacobian_col_indices;
public:
    IpoptADOLC_OptimizationProblem() = default;
    IpoptADOLC_OptimizationProblem(int num_variables, int num_constraints) :
            m_num_variables(num_variables), m_num_constraints(num_constraints) {}

    void set_num_variables(unsigned num_variables)
    { m_num_variables = num_variables; }
    void set_num_constraints(unsigned num_constraints)
    { m_num_constraints = num_constraints; }

    virtual void objective(const std::vector<adouble>& x,
                           adouble& obj_value) const = 0;
    virtual void constraints(const std::vector<adouble>& x,
                             std::vector<adouble>& constraints) const = 0;
    void lagrangian(double obj_factor, const std::vector<adouble>& x,
                    const std::vector<double>& lambda,
                    adouble& result) const {
        assert(x.size() == m_num_variables);
        assert(lambda.size() == m_num_constraints);

        result = 0;
        //if (obj_factor != 0) {
        //    objective(x, result);
        //    result *= obj_factor;
        //}
        objective(x, result); // TODO should not compute obj if obj_factor = 0.
        // but that optimization makes ADOLC unhappy.
        result *= obj_factor;
        std::vector<adouble> constr(m_num_constraints);
        constraints(x, constr);
        for (unsigned icon = 0; icon < m_num_constraints; ++icon) {
            result += lambda[icon] * constr[icon];
        }
    }
    void set_variable_bounds(const std::vector<double>& lower,
                             const std::vector<double>& upper) {
        assert(lower.size() == m_num_variables);
        assert(upper.size() == m_num_variables);
        m_variable_lower_bounds = lower;
        m_variable_upper_bounds = upper;
        // TODO check their sizes.
    }
    void set_constraint_bounds(const std::vector<double>& lower,
                               const std::vector<double>& upper) {
        assert(lower.size() == m_num_constraints);
        assert(upper.size() == m_num_constraints);
        m_constraint_lower_bounds = lower;
        m_constraint_upper_bounds = upper;
    }

    void set_initial_guess(const std::vector<double>& guess) {
        // TODO be smart about the need to copy "guess" (could be long)?
        m_initial_guess = guess;
        // TODO check their sizes.
        assert(guess.size() == m_num_variables);

        // Determine sparsity patterns.
        // ----------------------------
        // TODO remove from here and utilize new_x.
        // TODO or can I reuse the tape?
        {
            short int tag = 0;
            // =================================================================
            // START ACTIVE
            // -----------------------------------------------------------------
            trace_on(tag);
            std::vector<adouble> x_adouble(m_num_variables);
            for (unsigned i = 0; i < m_num_variables; ++i) x_adouble[i] <<= guess[i];
            std::vector<adouble> g_adouble(m_num_constraints);
            constraints(x_adouble, g_adouble);
            std::vector<double> g(m_num_constraints);
            for (unsigned i = 0; i < m_num_constraints; ++i) g_adouble[i] >>= g[i];
            trace_off();
            // -----------------------------------------------------------------
            // END ACTIVE
            // =================================================================

            int repeated_call = 0;
            int num_nonzeros = -1; /*TODO*/
            unsigned int* row_indices = NULL; // Allocated by ADOL-C.
            unsigned int* col_indices = NULL; // Allocated by ADOL-C.
            double* jacobian = NULL;          // Allocated by ADOL-C.
            int options[4];
            options[0] = 0; /*TODO*/
            options[1] = 0; /*TODO*/
            options[2] = 0; /*TODO*/
            options[3] = 0; /*TODO*/
            int success = sparse_jac(tag, m_num_constraints, m_num_variables,
                                     repeated_call, &guess[0],
                                     &num_nonzeros, &row_indices, &col_indices,
                                     &jacobian, options);
            assert(success);
            m_jacobian_num_nonzeros = num_nonzeros;
            m_jacobian_row_indices.reserve(num_nonzeros);
            m_jacobian_col_indices.reserve(num_nonzeros);
            for (int i = 0; i < num_nonzeros; ++i) {
                m_jacobian_row_indices[i] = row_indices[i];
                m_jacobian_col_indices[i] = col_indices[i];
            }
            delete [] row_indices;
            delete [] col_indices;
            delete [] jacobian;
        }

        {
            short int tag = 0;
            // =================================================================
            // START ACTIVE
            // -----------------------------------------------------------------
            trace_on(tag);
            std::vector<adouble> x_adouble(m_num_variables);
            std::vector<double> lambda_vector(m_num_constraints, 1);
            adouble lagrangian_adouble;
            double lagr;
            for (unsigned i = 0; i < m_num_variables; ++i) {
                x_adouble[i] <<= guess[i];
            }
            lagrangian(1.0, x_adouble, lambda_vector, lagrangian_adouble);
            lagrangian_adouble >>= lagr;
            trace_off();
            // -----------------------------------------------------------------
            // END ACTIVE
            // =================================================================
            // TODO efficiently use the "repeat" argument.
            int repeated_call = 0;
            int options[2];
            options[0] = 0; /* test the computational graph control flow? TODO*/
            options[1] = 0; /* way of recovery TODO */
            unsigned int* row_indices = NULL;
            unsigned int* col_indices = NULL;
            double* hessian = NULL; // We don't actually need the hessian...
            // TODO use hess_pat instead!!!
            int num_nonzeros;
            std::vector<double> x_and_lambda(m_num_variables + m_num_constraints);
            for (unsigned ivar = 0; ivar < m_num_variables; ++ivar) {
                x_and_lambda[ivar] = guess[ivar];
            }
            for (unsigned icon = 0; icon < m_num_constraints; ++icon) {
                x_and_lambda[icon + m_num_variables] = 1; // TODO consistency?
            }
            int success = sparse_hess(tag, m_num_variables, repeated_call,
                                      &guess[0], &num_nonzeros,
                                      &row_indices, &col_indices, &hessian,
                                      options);
            assert(success);
            m_hessian_num_nonzeros = num_nonzeros;
            m_hessian_row_indices.reserve(num_nonzeros);
            m_hessian_col_indices.reserve(num_nonzeros);
            for (int i = 0; i < num_nonzeros; ++i) {
                m_hessian_row_indices[i] = row_indices[i];
                m_hessian_col_indices[i] = col_indices[i];
            }
            // TODO try to use modern memory management.
            delete [] row_indices;
            delete [] col_indices;
            delete [] hessian;
        }
    }

    bool get_nlp_info(Index& num_variables, Index& num_constraints,
                      Index& num_nonzeros_jacobian, Index& num_nonzeros_hessian,
                      IndexStyleEnum& index_style) override {
        num_variables = m_num_variables;
        num_constraints = m_num_constraints;
        num_nonzeros_jacobian = m_jacobian_num_nonzeros;
        num_nonzeros_hessian = m_hessian_num_nonzeros;
        index_style = TNLP::C_STYLE;

        return true;
    }
    bool get_bounds_info(Index num_variables,   Number* x_lower, Number* x_upper,
                         Index num_constraints, Number* g_lower, Number* g_upper)
    override {
        assert((unsigned)num_variables   == m_num_variables);
        assert((unsigned)num_constraints == m_num_constraints);

        // TODO pass onto subclass.
        // TODO efficient copying.

        // TODO make sure bounds have been set.
        for (Index ivar = 0; ivar < num_variables; ++ivar) {
            x_lower[ivar] = m_variable_lower_bounds[ivar];
            x_upper[ivar] = m_variable_upper_bounds[ivar];
        }
        // TODO do not assume that there are no inequality constraints.
        if (m_constraint_lower_bounds.size() != (unsigned)num_constraints ||
            m_constraint_upper_bounds.size() != (unsigned)num_constraints) {
            for (Index icon = 0; icon < num_constraints; ++icon) {
                g_lower[icon] = 0;
                g_upper[icon] = 0;
            }
        } else {
            for (Index icon = 0; icon < num_constraints; ++icon) {
                const auto& lower = m_constraint_lower_bounds[icon];
                const auto& upper = m_constraint_upper_bounds[icon];
                // TODO turn the following into an exception message:
                assert(lower <= upper);
                g_lower[icon] = lower;
                g_upper[icon] = upper;
            }
        }
        return true;
    }
    // z: multipliers for bound constraints on x.
    // warmstart will require giving initial values for the multipliers.
    bool get_starting_point(Index num_variables, bool init_x, Number* x,
                            bool init_z, Number* /*z_L*/, Number* /*z_U*/,
                            Index num_constraints, bool init_lambda,
                            Number* /*lambda*/) override {
        // Must this method provide initial values for x, z, lambda?
        assert(init_x == true);
        assert(init_z == false);
        assert(init_lambda == false);
        assert((unsigned)num_constraints == m_num_constraints);
        // TODO change this interface so that the user specifies the initial
        // guess at the time they request the optimization?
        for (Index ivar = 0; ivar < num_variables; ++ivar) {
            x[ivar] = m_initial_guess[ivar];
        }
        return true;
    }
    bool eval_f(Index num_variables, const Number* x, bool /*new_x*/,
                Number& obj_value) override {
        assert((unsigned)num_variables == m_num_variables);
        std::vector<adouble> x_adouble(num_variables);
        // TODO efficiently store this result so it can be used in grad_f, etc.
        for (Index i = 0; i < num_variables; ++i) x_adouble[i] <<= x[i];
        adouble f = 0;
        // TODO objective() could be templatized... so that we could use finite
        // difference if necessary.
        objective(x_adouble, f);
        obj_value = f.value();
        return true;
    }
    bool eval_grad_f(Index num_variables, const Number* x, bool /*new_x*/,
                     Number* grad_f) override {
        assert((unsigned)num_variables == m_num_variables);
        short int tag = 0;

        // =====================================================================
        // START ACTIVE
        // ---------------------------------------------------------------------
        trace_on(tag);
        std::vector<adouble> x_adouble(num_variables);
        adouble f_adouble = 0;
        double f = 0;
        for (Index i = 0; i < num_variables; ++i) x_adouble[i] <<= x[i];
        objective(x_adouble, f_adouble);
        f_adouble >>= f;
        trace_off();
        // ---------------------------------------------------------------------
        // END ACTIVE
        // =====================================================================
        int success = gradient(tag, num_variables, x, grad_f);
        assert(success);

        return true;
    }
    bool eval_g(Index num_variables, const Number* x, bool /*new_x*/,
                Index num_constraints, Number* g) override {
        assert((unsigned)num_variables   == m_num_variables);
        assert((unsigned)num_constraints == m_num_constraints);
        std::vector<adouble> x_adouble(num_variables);
        // TODO efficiently store this result so it can be used in grad_f, etc.
        for (Index i = 0; i < num_variables; ++i) x_adouble[i] <<= x[i];
        std::vector<adouble> gradient(num_constraints);
        constraints(x_adouble, gradient);
        for (Index i = 0; i < num_constraints; ++i) gradient[i] >>= g[i];
        return true;
    }
    // TODO can Ipopt do finite differencing for us?
    bool eval_jac_g(Index num_variables, const Number* x, bool /*new_x*/,
                    Index num_constraints, Index num_nonzeros_jacobian,
                    Index* iRow, Index *jCol, Number* values) override {
        if (values == nullptr) {
            // TODO document: provide sparsity pattern.
            assert((unsigned)num_nonzeros_jacobian == m_jacobian_num_nonzeros);
            for (Index inz = 0; inz < num_nonzeros_jacobian; ++inz) {
                iRow[inz] = m_jacobian_row_indices[inz];
                jCol[inz] = m_jacobian_col_indices[inz];
            }
            return true;
        }

        short int tag = 0;
        // =====================================================================
        // START ACTIVE
        // ---------------------------------------------------------------------
        trace_on(tag);
        std::vector<adouble> x_adouble(num_variables);
        for (Index i = 0; i < num_variables; ++i) x_adouble[i] <<= x[i];
        std::vector<adouble> g_adouble(num_constraints);
        constraints(x_adouble, g_adouble);
        std::vector<double> g(num_constraints);
        for (Index i = 0; i < num_constraints; ++i) g_adouble[i] >>= g[i];
        trace_off();
        // ---------------------------------------------------------------------
        // END ACTIVE
        // =====================================================================

        int repeated_call = 0;
        int num_nonzeros = -1; /*TODO*/
        unsigned int* row_indices = NULL; // Allocated by ADOL-C.
        unsigned int* col_indices = NULL; // Allocated by ADOL-C.
        double* jacobian = NULL;          // Allocated by ADOL-C.
        int options[4];
        options[0] = 0; /*TODO*/
        options[1] = 0; /*TODO*/
        options[2] = 0; /*TODO*/
        options[3] = 0; /*TODO*/
        int success = sparse_jac(tag, num_constraints, num_variables,
                                 repeated_call, x,
                                 &num_nonzeros, &row_indices, &col_indices,
                                 &jacobian, options);
        assert(success);
        for (int inz = 0; inz < num_nonzeros; ++inz) {
            values[inz] = jacobian[inz];
        }

        delete [] row_indices;
        delete [] col_indices;
        delete [] jacobian;

        return true;
    }
    bool eval_h(Index num_variables, const Number* x, bool /*new_x*/,
                Number obj_factor, Index num_constraints, const Number* lambda,
                bool /*new_lambda*/, Index num_nonzeros_hessian,
                Index* iRow, Index *jCol, Number* values) override {
        assert((unsigned)num_nonzeros_hessian == m_hessian_num_nonzeros);
        if (values == nullptr) {
            // TODO use ADOLC to determine sparsity pattern; hess_pat
            // TODO
            for (Index inz = 0; inz < num_nonzeros_hessian; ++inz) {
                iRow[inz] = m_hessian_row_indices[inz];
                jCol[inz] = m_hessian_col_indices[inz];
            }
            return true;
        }

        // TODO this hessian must include the constraint portion!!!
        // TODO if not new_x, then do NOT re-eval objective()!!!

        // TODO remove from here and utilize new_x.
        // TODO or can I reuse the tape?
        short int tag = 0;
        // -----------------------------------------------------------------
        // START ACTIVE
        trace_on(tag);
        std::vector<adouble> x_adouble(num_variables);
        std::vector<double> lambda_vector(num_constraints);
        adouble lagrangian_adouble;
        double lagr;
        for (Index ivar = 0; ivar < num_variables; ++ivar) {
            // TODO add this operator for std::vector.
            x_adouble[ivar] <<= x[ivar];
        }
        for (Index icon = 0; icon < num_constraints; ++icon) {
            lambda_vector[icon] = lambda[icon];
        }
        lagrangian(obj_factor, x_adouble, lambda_vector, lagrangian_adouble);
        lagrangian_adouble >>= lagr;
        trace_off();
        // END ACTIVE
        // -----------------------------------------------------------------
        // TODO efficiently use the "repeat" argument.
        int repeated_call = 0;
        int options[2];
        options[0] = 0; /* test the computational graph control flow? TODO*/
        options[1] = 0; /* way of recovery TODO */
        // TODO make general:
        unsigned int* row_indices = NULL;
        unsigned int* col_indices = NULL;
        // TODO hope that the row indices are the same between IpOopt and
        // ADOL-C.
        double* vals = NULL;
        int num_nonzeros;
        // TODO compute sparse hessian for each element of the constraint
        // vector....TODO trace with respect to both x and lambda..
        // http://list.coin-or.org/pipermail/adol-c/2013-April/000900.html
        // TODO "since lambda changes, the Lagrangian function has to be
        // repated every time ...cannot set repeat = 1"
        // The following link suggests more efficient methods:
        // http://list.coin-or.org/pipermail/adol-c/2013-April/000903.html
        // Quote:
        // We made the experience that it really depends on the application
        // whether
        //
        // * tracing the Lagrangian once with x and lambda as inputs
        //    and evaluating only a part of the Hessian reusing the trace
        //       in all iterations
        //
        // or
        //
        // *  retracing the Lagrangian with x as adoubles and lambda as doubles
        // in each iteration and computing then the whole Hessian
        //
        // performs better in terms of runtime. You could give both approaches
        // a try and see what works better for you. Both approaches have their
        // pros and cons with respect to efficiency.
        //

        //std::vector<double> x_and_lambda(num_variables + num_constraints);
        //for (unsigned ivar = 0; ivar < num_variables; ++ivar) {
        //    x_and_lambda[ivar] = x[ivar];
        //}
        //for (unsigned icon = 0; icon < num_constraints; ++icon) {
        //    x_and_lambda[icon + num_variables] = lambda[icon];
        //}
        int success = sparse_hess(tag, num_variables, repeated_call,
                                  x, &num_nonzeros, &row_indices, &col_indices,
                                  &vals, options);
        assert(success);
        for (int i = 0; i < num_nonzeros; ++i) {
            values[i] = vals[i];
        }
        // TODO try to use modern memory management.
        delete [] row_indices;
        delete [] col_indices;
        // TODO avoid reallocating vals each time!!!
        delete [] vals;

        return true;
    }
    void finalize_solution(Ipopt::SolverReturn /*TODO status*/,
                           Index num_variables,
                           const Number* x, const Number* z_L, const Number* z_U,
                           Index /*num_constraints*/,
                           const Number* /*g*/, const Number* /*lambda*/,
                           Number obj_value, const Ipopt::IpoptData* /*ip_data*/,
                           Ipopt::IpoptCalculatedQuantities* /*ip_cq*/) override {
        printf("\nSolution of the primal variables, x\n");
        for (Index i = 0; i < num_variables; ++i) {
            printf("x[%d]: %e\n", i, x[i]);
        }
        printf("\nSolution of the bound multipliers, z_L and z_U\n");
        for (Index i = 0; i < num_variables; ++i) {
            printf("z_L[%d] = %e\n", i, z_L[i]);
        }
        for (Index i = 0; i < num_variables; ++i) {
            printf("z_U[%d] = %e\n", i, z_U[i]);
        }
        printf("\nObjective value\n");
        printf("f(x*) = %e\n", obj_value);
    }

};

template <typename T>
class OptimalControlProblem {
public:
    virtual ~OptimalControlProblem() = default;
    // TODO this is definitely not the interface I want.
    // TODO difficult... virtual void initial_guess()
    // TODO really want to declare each state variable individually, and give
    // each one a name.
    virtual int num_states() const = 0;
    virtual int num_controls() const = 0;
    virtual void bounds(double& initial_time, double& final_time,
                        std::vector<double>& states_lower,
                        std::vector<double>& states_upper,
                        std::vector<double>& initial_states_lower,
                        std::vector<double>& initial_states_upper,
                        std::vector<double>& final_states_upper,
                        std::vector<double>& final_states_lower,
                        std::vector<double>& controls_lower,
                        std::vector<double>& controls_upper,
                        std::vector<double>& initial_controls_lower,
                        std::vector<double>& initial_controls_upper,
                        std::vector<double>& final_controls_lower,
                        std::vector<double>& final_controls_upper) const = 0;

    // TODO use Eigen, not std::vector.
    virtual void dynamics(const std::vector<T>& states,
                          const std::vector<T>& controls,
                          std::vector<T>& derivative) const = 0;
    // TODO alternate form that takes a matrix; state at every time.
    //virtual void continuous(const MatrixXd& x, MatrixXd& xdot) const = 0;
    //virtual void endpoint_cost(const T& final_time,
    //                           const std::vector<T>& final_states) const = 0;
    // TODO change time to T.
    virtual void integral_cost(const double& time,
                               const std::vector<T>& states,
                               const std::vector<T>& controls,
                               T& integrand) const = 0;
};

class DirectCollocationSolver : public IpoptADOLC_OptimizationProblem {
    typedef OptimalControlProblem<adouble> Problem;
    int m_num_mesh_points = 20;
    int m_num_states = -1;
    int m_num_controls = -1;
    int m_num_continuous_variables = -1;
    double m_initial_time = -1;
    double m_final_time = -1;
public:
    // TODO why shared_ptr???
    void set_problem(std::shared_ptr<Problem> problem);
    int state_index(int i_mesh_point, int i_state) const {
        return i_mesh_point * m_num_continuous_variables + i_state;
    }
    int control_index(int i_mesh_point, int i_control) const{
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
    void objective(const std::vector<adouble>& x,
                   adouble& obj_value) const override;
    void constraints(const std::vector<adouble>& x,
                     std::vector<adouble>& constraints) const override;
    void finalize_solution(Ipopt::SolverReturn /*TODO status*/,
                           Index /*num_variables*/,
                           const Number* x,
                           const Number* /*z_L*/, const Number* /*z_U*/,
                           Index /*num_constraints*/,
                           const Number* /*g*/, const Number* /*lambda*/,
                           Number obj_value, const Ipopt::IpoptData* /*ip_data*/,
                           Ipopt::IpoptCalculatedQuantities* /*ip_cq*/) override;
private:
    // TODO uh oh how to have generic interface?? the solver should also be
    // templatized??
    std::shared_ptr<Problem> m_problem;
};
