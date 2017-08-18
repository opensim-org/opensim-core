
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
        assert(num_variables == 4);
        assert(num_constraints == 2);
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
                            bool init_z, Number* /*z_L*/, Number* /*z_U*/,
                            Index num_constraints,
                            bool init_lambda, Number* /*lambda*/) 
            override {
        assert(num_variables == 4);
        assert(num_constraints == 2);
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
    bool eval_f(Index num_variables, const Number* x, bool /*new_x*/,
                Number& obj_value) override {
        assert(num_variables == 4);
        obj_value = x[0] * x[3] * (x[0] + x[1] + x[2]) + x[2];
        return true;
    }
    bool eval_grad_f(Index num_variables, const Number* x, bool /*new_x*/,
                     Number* grad_f) override {
        assert(num_variables == 4);
        // TODO try using autodiff.
        grad_f[0] = x[0] * x[3] + x[3] * (x[0] + x[1] + x[2]);
        grad_f[1] = x[0] * x[3];
        grad_f[2] = x[0] * x[3] + 1;
        grad_f[3] = x[0] * (x[0] + x[1] + x[2]);
        return true;
    }
    bool eval_g(Index num_variables, const Number* x, bool /*new_x*/,
                Index num_constraints, Number* g) override {
        assert(num_variables == 4);
        assert(num_constraints == 2);
        g[0] = x[0] * x[1] * x[2] * x[3];
        g[1] = x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3];
        return true;
    }
    // TODO can Ipopt do finite differencing for us?
    bool eval_jac_g(Index num_variables, const Number* x, bool /*new_x*/,
                    Index num_constraints, Index /*num_nonzeros_jacobian*/,
                    Index* iRow, Index *jCol, Number* values) override {
        assert(num_variables == 4);
        assert(num_constraints == 2);
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
    bool eval_h(Index num_variables, const Number* x, bool /*new_x*/,
                Number obj_factor, Index num_constraints, const Number* lambda,
                bool /*new_lambda*/, Index num_nonzeros_hessian,
                Index* iRow, Index *jCol, Number* values) override {
        // TODO why Hessian of Lagrangian, instead of Hessian of f?
        // TODO Ah because it's a second order method...
        // TODO symmetric! Only need to provide lower left triangle.
        assert(num_variables == 4);
        assert(num_constraints == 2);
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
    /*
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
    */

};
