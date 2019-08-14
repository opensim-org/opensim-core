#ifndef TROPTER_TESTING_OPTIMALCONTROL_H
#define TROPTER_TESTING_OPTIMALCONTROL_H
// ----------------------------------------------------------------------------
// tropter: testing_optimalcontrol.h
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

#include "testing.h"

#include <tropter/optimization/ProblemDecorator.h>
#include <tropter/optimalcontrol/DirectCollocation.h>
#include <tropter/optimalcontrol/transcription/Base.h>

#include <Eigen/SparseCore>

namespace tropter {

/// Compare the gradient, Jacobian, and Hessian for an OptimalControlProblem
/// between scalar types double and adouble.
template <template <class> class OCPType>
struct OCPDerivativesComparison {
    int num_mesh_intervals = 5;
    std::string findiff_hessian_mode = "fast";
    double findiff_hessian_step_size = 1e-3;
    double gradient_error_tolerance = 1e-7;
    double jacobian_error_tolerance = 1e-6;
    double hessian_error_tolerance = 1e-7;
    void compare() const {
        using Eigen::VectorXd;
        using Eigen::SparseMatrix;

        // double
        auto d = std::make_shared<OCPType<double>>();
        DirectCollocationSolver<double> ddc(d, "trapezoidal", "ipopt",
                num_mesh_intervals);
        auto dnlp = ddc.get_transcription().make_decorator();
        dnlp->set_findiff_hessian_step_size(findiff_hessian_step_size);
        dnlp->set_findiff_hessian_mode(findiff_hessian_mode);
        // TODO const auto x = dnlp->make_initial_guess_from_bounds();
        auto x = dnlp->make_random_iterate_within_bounds();

        VectorXd dgrad;
        SparseMatrix<double> djac;
        SparseMatrix<double> dhes;
        calc_derivatives(dnlp.get(), x, dgrad, djac, dhes);

        // adouble
        auto a = std::make_shared<OCPType<adouble>>();
        DirectCollocationSolver<adouble> adc(a, "trapezoidal", "ipopt",
                num_mesh_intervals);
        auto anlp = adc.get_transcription().make_decorator();
        VectorXd agrad;
        SparseMatrix<double> ajac;
        SparseMatrix<double> ahes;
        calc_derivatives(anlp.get(), x, agrad, ajac, ahes);

        // Test.
        CAPTURE(dgrad);
        CAPTURE(agrad);
        TROPTER_REQUIRE_EIGEN(dgrad, agrad, gradient_error_tolerance);
        CAPTURE(djac);
        CAPTURE(ajac);
        compare_sparse(djac, ajac, jacobian_error_tolerance);
        CAPTURE(dhes);
        CAPTURE(ahes);
        compare_sparse(dhes, ahes, hessian_error_tolerance);
        //    std::cout << "DEBUGdhes\n" << Eigen::MatrixXd(dhes) << std::endl;
        //    std::cout << "DEBUGahes\n" << Eigen::MatrixXd(ahes) << std::endl;
    }
private:
    static Eigen::SparseMatrix<double> convert_to_SparseMatrix(
            int num_rows, int num_cols,
            const std::vector<unsigned>& row_indices,
            const std::vector<unsigned>& col_indices,
            const Eigen::VectorXd& values) {
        Eigen::SparseMatrix<double> mat(num_rows, num_cols);
        mat.reserve(row_indices.size());
        for (int inz = 0; inz < (int)row_indices.size(); ++inz)
            mat.insert(row_indices[inz], col_indices[inz]) = values[inz];
        mat.makeCompressed();
        return mat;
    }
    static void calc_derivatives(
            const optimization::ProblemDecorator* nlp,
            const Eigen::VectorXd& x, Eigen::VectorXd& grad,
            Eigen::SparseMatrix<double>& jac,
            Eigen::SparseMatrix<double>& hes) {
        const int num_constraints = nlp->get_num_constraints();
        const int num_variables = nlp->get_num_variables();
        SparsityCoordinates jac_sparsity, hes_sparsity;
        nlp->calc_sparsity(x, jac_sparsity, true, hes_sparsity);
        const unsigned num_jac_nonzeros = (unsigned)jac_sparsity.row.size();
        const unsigned num_hes_nonzeros = (unsigned)hes_sparsity.row.size();

        // Gradient.
        grad.resize(x.size());
        nlp->calc_gradient((int)x.size(), x.data(), true, grad.data());

        // Jacobian.
        using Eigen::VectorXd;
        VectorXd jac_vec(num_jac_nonzeros);
        nlp->calc_jacobian((int)x.size(), x.data(), true,
                num_jac_nonzeros, jac_vec.data());
        jac = convert_to_SparseMatrix(num_constraints, num_variables,
                jac_sparsity.row, jac_sparsity.col, jac_vec);

        // Hessian.
        VectorXd hes_vec(num_hes_nonzeros);
        VectorXd lambda = VectorXd::Ones(num_constraints);
        nlp->calc_hessian_lagrangian(
                (int)x.size(), x.data(), true, 1.0,
                num_constraints, lambda.data(), true,
                num_hes_nonzeros, hes_vec.data());
        hes = convert_to_SparseMatrix(num_variables, num_variables,
                hes_sparsity.row, hes_sparsity.col, hes_vec);
    }
    static void compare_sparse(const Eigen::SparseMatrix<double>& a,
            const Eigen::SparseMatrix<double>& b, double rel_error_tolerance) {
        REQUIRE((a.rows() == b.rows()));
        REQUIRE((a.cols() == b.cols()));
        for (int ir = 0; ir < a.rows(); ++ir) {
            for (int ic = 0; ic < a.cols(); ++ic) {
                INFO("(" << ir << "," << ic << "): " <<
                        a.coeff(ir, ic) << " vs " << b.coeff(ir, ic));
                REQUIRE((Approx(a.coeff(ir, ic))
                        .epsilon(rel_error_tolerance)
                        .scale(1.0)
                        == b.coeff(ir, ic)));
            }
        }
    }
};

}

#endif // TROPTER_TESTING_OPTIMALCONTROL_H
