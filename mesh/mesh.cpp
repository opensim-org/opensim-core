#include <iostream>
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


// TODO use faster linear solvers from Sherlock cluster.

// TODO templatize Problem.
class Problem {
public:
    virtual void ode(const VectorXd& x, const VectorXd& u, VectorXd& xdot)
            const = 0;
    //virtual void continuous(const MatrixXd& x, MatrixXd& xdot) const = 0;
    virtual int num_states() const = 0;
    virtual int num_controls() const = 0;
};

double g = 9.81;

class MyProb : public Problem {
    void ode(const VectorXd& x, const VectorXd& u, VectorXd& xdot) const
            override {
        xdot[0] = x[1];
        xdot[1] = u[0];
        //xdot[1] = -g * sin(x[0]);
    }
    int num_states() const override { return 2; }
    int num_controls() const override { return 1; }
};

class IpoptADOLC_OptimizationProblem : public Ipopt::TNLP {
private:
    unsigned m_num_variables = -1;
public:
    IpoptADOLC_OptimizationProblem(int num_variables) :
        m_num_variables(num_variables) {}

    virtual void objective(const std::vector<adouble>& x,
                           adouble& obj_value) const = 0;

    bool get_nlp_info(Index& num_variables, Index& num_constraints,
            Index& num_nonzeros_jacobian, Index& num_nonzeros_hessian,
            IndexStyleEnum& index_style) override {
        num_variables = m_num_variables;
        num_constraints = 0;
        num_nonzeros_jacobian = 0;
        num_nonzeros_hessian = 1;
        index_style = TNLP::C_STYLE;

        return true;
    }
    bool get_bounds_info(Index num_variables,   Number* x_lower, Number* x_upper,
                         Index num_constraints, Number* g_lower, Number* g_upper) 
            override {
        // TODO pass onto subclass.
        x_lower[0] = -5;
        x_upper[0] = 5;
        return true;
    }
    // z: multipliers for bound constraints on x.
    // warmstart will require giving initial values for the multipliers.
    bool get_starting_point(Index num_variables, bool init_x, Number* x,
                            bool init_z, Number* z_L, Number* z_U,
                            Index num_constraints, bool init_lambda, Number* lambda) 
            override {
        // Must this method provide initial values for x, z, lambda?
        assert(init_x == true);
        assert(init_z == false);
        assert(init_lambda == false);
        x[0] = 1.0;
        return true;
    }
    bool eval_f(Index num_variables, const Number* x, bool new_x,
                Number& obj_value) override {
        assert(num_variables == 1);
        std::vector<adouble> x_adouble(num_variables);
        // TODO efficiently store this result so it can be used in grad_f, etc.
        for (unsigned i = 0; i < num_variables; ++i) x_adouble[i] <<= x[i];
        adouble f;
        objective(x_adouble, f);
        obj_value = f.value(); /*(x[0] - 1.5) * (x[0] - 1.5);*/
        return true;
    }
    bool eval_grad_f(Index num_variables, const Number* x, bool new_x,
                     Number* grad_f) override {
        assert(num_variables == m_num_variables);
        short int tag = 0;

        // =====================================================================
        // START ACTIVE
        // ---------------------------------------------------------------------
        trace_on(tag);
        std::vector<adouble> x_adouble(num_variables);
        adouble f_adouble;
        double f;
        for (unsigned i = 0; i < num_variables; ++i) x_adouble[i] <<= x[i];
        objective(x_adouble, f_adouble);
        f_adouble >>= f; 
        trace_off();
        // ---------------------------------------------------------------------
        // END ACTIVE
        // =====================================================================
        int success = gradient(tag, num_variables, x, grad_f);

        return true;
    }
    bool eval_g(Index num_variables, const Number* x, bool new_x,
                Index num_constraints, Number* g) override {
        assert(num_variables == 1);
        assert(num_constraints == 0);
        return true;
    }
    // TODO can Ipopt do finite differencing for us?
    bool eval_jac_g(Index num_variables, const Number* x, bool new_x,
                    Index num_constraints, Index num_nonzeros_jacobian,
                    Index* iRow, Index *jCol, Number* values) override {
        return true;
    }
    bool eval_h(Index num_variables, const Number* x, bool new_x,
                Number obj_factor, Index num_constraints, const Number* lambda,
                bool new_lambda, Index num_nonzeros_hessian,
                Index* iRow, Index *jCol, Number* values) override {
        if (values == nullptr) {
            // TODO use ADOLC to determine sparsity pattern; hess_pat
            iRow[0] = 0;
            jCol[0] = 0;
        } else {
            // TODO remove from here and utilize new_x.
            // TODO or can I reuse the tape?
            short int tag = 0;
            // -----------------------------------------------------------------
            // START ACTIVE
            trace_on(tag);
            std::vector<adouble> x_adouble(num_variables);
            adouble f_adouble;
            double f;
            for (unsigned i = 0; i < num_variables; ++i) x_adouble[i] <<= x[i];
            objective(x_adouble, f_adouble);
            f_adouble >>= f; 
            trace_off();
            // END ACTIVE
            // -----------------------------------------------------------------
            // TODO efficiently use the "repeat" argument.
            int repeated_call = 0;
            int options[2];
            options[0] = 0; /* test the computational graph control flow? TODO*/
            options[1] = 0; /* way of recovery TODO */
            // TODO make general:
            std::unique_ptr<unsigned int[]> row_indices(new unsigned int[1]);
            std::unique_ptr<unsigned int[]> col_indices(new unsigned int[1]);
            unsigned int* row_indices2 = row_indices.get();
            unsigned int* col_indices2 = col_indices.get();
//            unsigned int* row_indices = new unsigned int[1];
//            unsigned int* col_indices = new unsigned int[1];
            // TODO hope that the row indices are the same between IpOopt and
            // ADOL-C.
            int success = sparse_hess(tag, num_variables, repeated_call,
                                      x, &num_nonzeros_hessian, 
                                      &row_indices2, &col_indices2, &values,
                                      options);
        }
        return true;
    }
    void finalize_solution(Ipopt::SolverReturn status, Index num_variables,
            const Number* x, const Number* z_L, const Number* z_U,
            Index num_constraints, const Number* g, const Number* lambda,
            Number obj_value, const Ipopt::IpoptData* ip_data,
            Ipopt::IpoptCalculatedQuantities* ip_cq) override {
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

class ToyProblem : public IpoptADOLC_OptimizationProblem {
public:
    ToyProblem() : IpoptADOLC_OptimizationProblem(1) {}
    void objective(const std::vector<adouble>& x,
                   adouble& obj_value) const override {
        obj_value = (x[0] - 1.5) * (x[0] - 1.5);
    }
};

class IpoptOptimizationProblem : public Ipopt::TNLP {
public:
    bool get_nlp_info(Index& num_variables, Index& num_constraints,
            Index& num_nonzeros_jacobian, Index& num_nonzeros_hessian,
            IndexStyleEnum& index_style) override {
        num_variables = 4;
        num_constraints = 2;
        num_nonzeros_jacobian = 8;
        num_nonzeros_hessian = 10;
        index_style = TNLP::C_STYLE;

        return true;
    }
    bool get_bounds_info(Index num_variables,   Number* x_lower, Number* x_upper,
                         Index num_constraints, Number* g_lower, Number* g_upper) 
            override {
        for (Index i = 0; i < 4; ++i) 
            x_lower[i] = 1.0;
        for (Index i = 0; i < 4; ++i)
            x_upper[i] = 5.0;
        g_lower[0] = 25;
        g_upper[0] = 2e19; // TODO nlp_upper_bound_inf
        g_lower[1] = 40.0;
        g_upper[1] = 40.0;
        return true;
    }
    // z: multipliers for bound constraints on x.
    // warmstart will require giving initial values for the multipliers.
    bool get_starting_point(Index num_variables, bool init_x, Number* x,
                            bool init_z, Number* z_L, Number* z_U,
                            Index num_constraints, bool init_lambda, Number* lambda) 
            override {
        // Must this method provide initial values for x, z, lambda?
        assert(init_x == true);
        assert(init_z == false);
        assert(init_lambda == false);
        x[0] = 1.0;
        x[1] = 5.0;
        x[2] = 5.0;
        x[3] = 1.0;
        return true;
    }
    bool eval_f(Index num_variables, const Number* x, bool new_x,
                Number& obj_value) override {
        assert(num_variables == 4);
        obj_value = x[0] * x[3] * (x[0] + x[1] + x[2]) + x[2];
        return true;
    }
    bool eval_grad_f(Index num_variables, const Number* x, bool new_x,
                     Number* grad_f) override {
        assert(num_variables == 4);
        // TODO try using autodiff.
        grad_f[0] = x[0] * x[3] + x[3] * (x[0] + x[1] + x[2]);
        grad_f[1] = x[0] * x[3];
        grad_f[2] = x[0] * x[3] + 1;
        grad_f[3] = x[0] * (x[0] + x[1] + x[2]);
        return true;
    }
    bool eval_g(Index num_variables, const Number* x, bool new_x,
                Index num_constraints, Number* g) override {
        assert(num_variables == 4);
        assert(num_constraints == 2);
        g[0] = x[0] * x[1] * x[2] * x[3];
        g[1] = x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3];
        return true;
    }
    // TODO can Ipopt do finite differencing for us?
    bool eval_jac_g(Index num_variables, const Number* x, bool new_x,
                    Index num_constraints, Index num_nonzeros_jacobian,
                    Index* iRow, Index *jCol, Number* values) override {
        if (values == nullptr) {
            // Return the structure of the Jacobian.
            iRow[0] = 0; jCol[0] = 0;
            iRow[1] = 0; jCol[1] = 1;
            iRow[2] = 0; jCol[2] = 2;
            iRow[3] = 0; jCol[3] = 3;
            iRow[4] = 1; jCol[4] = 0;
            iRow[5] = 1; jCol[5] = 1;
            iRow[6] = 1; jCol[6] = 2;
            iRow[7] = 1; jCol[7] = 3;
        } else {
            // Return the value of the Jacobian of the constraint vector.
            values[0 /* 0, 0 */] = x[1] * x[2] * x[3];
            values[1 /* 0, 1 */] = x[0] * x[2] * x[3];
            values[2 /* 0, 2 */] = x[0] * x[1] * x[3];
            values[3 /* 0, 3 */] = x[0] * x[1] * x[2];

            values[4 /* 1, 0 */] = 2 * x[0];
            values[5 /* 1, 1 */] = 2 * x[1];
            values[6 /* 1, 2 */] = 2 * x[2];
            values[7 /* 1, 3 */] = 2 * x[3];
        }
        return true;
    }
    bool eval_h(Index num_variables, const Number* x, bool new_x,
                Number obj_factor, Index num_constraints, const Number* lambda,
                bool new_lambda, Index num_nonzeros_hessian,
                Index* iRow, Index *jCol, Number* values) override {
        // TODO why Hessian of Lagrangian, instead of Hessian of f?
        // TODO Ah because it's a second order method...
        // TODO symmetric! Only need to provide lower left triangle.
        if (values == nullptr) {
            Index idx = 0;
            for (Index row = 0; row < 4; ++row) {
                for (Index col = 0; col <= row; ++col) {
                    iRow[idx] = row;
                    jCol[idx] = col;
                    ++idx;
                }
            }
            assert(idx == num_nonzeros_hessian);
        } else {
            // TODO detect value of obj_factor. This is the homotopy factor.
            // TODO can we do something more efficient based on the value of
            // obj_factor?
            // TODO must figure out a way to go between the two types of
            // indices. Create internal mapping?

            // obj_factor * grad^2 f(x)
            values[0] = obj_factor * (2 * x[3]);
            values[1] = obj_factor * (x[3]);
            values[2] = 0;
            values[3] = obj_factor * (x[3]);
            values[4] = 0;
            values[5] = 0;
            values[6] = obj_factor * (2 * x[0] + x[1] + x[2]);
            values[7] = obj_factor * (x[0]);
            values[8] = obj_factor * (x[0]);
            values[9] = 0;

            // lambda_0 * grad^2 g_0(x)
            values[1] += lambda[0] * (x[2] * x[3]);
            values[3] += lambda[0] * (x[1] * x[3]);
            values[4] += lambda[0] * (x[0] * x[3]);
            values[6] += lambda[0] * (x[1] * x[2]);
            values[7] += lambda[0] * (x[0] * x[2]);
            values[8] += lambda[0] * (x[0] * x[1]);

            // lambda_1 * grad^2 g_1(x)
            values[0] += lambda[1] * 2;
            values[2] += lambda[1] * 2;
            values[5] += lambda[1] * 2;
            values[9] += lambda[1] * 2;
        }
        return true;
    }
    void finalize_solution(Ipopt::SolverReturn status, Index num_variables,
            const Number* x, const Number* z_L, const Number* z_U,
            Index num_constraints, const Number* g, const Number* lambda,
            Number obj_value, const Ipopt::IpoptData* ip_data,
            Ipopt::IpoptCalculatedQuantities* ip_cq) override {
        // TODO the user can request a stop to the method!
        // USER_REQUESTED_STOP: intermediate_callback.
        // TODO would like to visualize progress of the optimization as it goes
        // TODO for this, could provide a convenience tool where the 
        // max_iterations is gradually increased.
        // TODO does Ipopt have a callback for intermediate progress?
        // TODO or should this be coded just as part of whenever eval_f gets
        // called?
        // TODO intermediate_callback allows getting access to the iterates!
        printf("\n\nSolution of the primal variables, x\n");
        for (Index i = 0; i < num_variables; ++i) {
            printf("x[%d]: %e\n", i, x[i]);
        }
        printf("\n\nSolution of the bound multipliers, z_L and z_U\n");
        for (Index i = 0; i < num_variables; ++i) {
            printf("z_L[%d] = %e\n", i, z_L[i]);
        }
        for (Index i = 0; i < num_variables; ++i) {
            printf("z_U[%d] = %e\n", i, z_U[i]);
        }
        printf("\n\nObjective value\n");
        printf("f(x*) = %e\n", obj_value);
    }

};

int main(int argc, char* argv[]) {
    Ipopt::ApplicationReturnStatus status;
    /*{
        Ipopt::SmartPtr<Ipopt::TNLP> mynlp = new IpoptOptimizationProblem();
        Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
        app->Options()->SetNumericValue("tol", 1e-9);
        app->Options()->SetStringValue("mu_strategy", "adaptive");
        app->Options()->SetStringValue("output_file", "ipopt.out");
        status = app->Initialize();
        if (status != Ipopt::Solve_Succeeded) {
            printf("\n\n*** Error during initialization!\n");
            return (int) status;
        }
        status = app->OptimizeTNLP(mynlp);
        if (status == Ipopt::Solve_Succeeded) {
            printf("\n\n*** The problem solved!\n");
        } else {
            printf("\n\n*** The problem FAILED!\n");
        }
    }*/
    {
        Ipopt::SmartPtr<Ipopt::TNLP> mynlp = new ToyProblem();
        Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
        app->Options()->SetNumericValue("tol", 1e-9);
        app->Options()->SetStringValue("mu_strategy", "adaptive");
        app->Options()->SetStringValue("output_file", "ipopt.out");
        Ipopt::ApplicationReturnStatus status;
        status = app->Initialize();
        if (status != Ipopt::Solve_Succeeded) {
            printf("\n\n*** Error during initialization!\n");
            return (int) status;
        }
        status = app->OptimizeTNLP(mynlp);
        if (status == Ipopt::Solve_Succeeded) {
            printf("\n\n*** The problem solved!\n");
        } else {
            printf("\n\n*** The problem FAILED!\n");
        }
        ToyProblem toy;
        adouble f;
        toy.objective({1.5}, f);
        std::cout << "DEBUG " << f << std::endl;
    }
    return (int) status;

    int num_points = 100;
    std::unique_ptr<Problem> problem(new MyProb());
    const int num_states = problem->num_states();
    const int num_controls = problem->num_controls();
    //MatrixXd xdot(num_states, num_points);
    //MatrixXd x(num_states, num_points);
    //MatrixXd u(num_controls, num_points);

    // TODO forward integration.
    const double step_size = 0.001;
    const int num_steps = 10000;
    const double final_time = step_size * num_steps;

    VectorXd xdot(num_states);
    VectorXd u(num_controls);
    u[0] = 1.0;
    VectorXd initial_x(num_states);
    initial_x[0] = 0;
    initial_x[1] = 0;
    VectorXd current_x = initial_x;
    for (int itime = 0; itime < num_steps; ++itime) {
        problem->ode(current_x, u/*.col(itime)*/, xdot);
        current_x = current_x + step_size * xdot;
        std::cout << current_x << std::endl << std::endl;
    }
    std::cout << "Final time: " << final_time << std::endl;

    /*
    for (int ipt = 0; ipt < num_points; ++ipt) {
        VectorXd this_xdot = xdot.col(0); // TODO unnecessary copy?
        problem->ode(x.col(ipt), u.col(ipt), this_xdot);
        std::cout << this_xdot << std::endl;
        //problem->ode(x[ipt], xdot[ipt]);
    }
    */
}
