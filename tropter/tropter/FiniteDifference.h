#ifndef TROPTER_FINITEDIFFERENCE_H
#define TROPTER_FINITEDIFFERENCE_H
// ----------------------------------------------------------------------------
// tropter: FiniteDifference.h
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

#include "common.h"
#include <Eigen/SparseCore>
#include "SparsityPattern.h"

/// The functions here are for internal use when computing derivatives via
/// finite differences; accordingly, this header is not included in tropter.h.
namespace tropter {


// TODO print dimensions dimensions.
inline void write_sparsity(const Eigen::SparseMatrix<bool>& mat,
        const std::string& filename) {
    std::ofstream file(filename);
    file << "row_indices,column_indices" << std::endl;
    for (int i = 0; i < mat.outerSize(); ++i) {
        for (Eigen::SparseMatrix<bool>::InnerIterator it(mat, i); it; ++it) {
            file << it.row() << "," << it.col() << std::endl;
        }
    }
    file.close();
}

template <typename T>
Eigen::SparseMatrix<bool>
calc_gradient_sparsity_with_nan(const Eigen::VectorXd& x0,
        std::function<T(const VectorX<T>&)>& function) {
    using std::isnan;
    using tropter::isnan;
    Eigen::SparseMatrix<bool> sparsity(1, x0.size());
    VectorX<T> x = x0.cast<T>();
    T f;
    for (int i = 0; i < (int)x.size(); ++i) {
        x[i] = std::numeric_limits<double>::quiet_NaN();
        f = function(x);
        x[i] = x0[i];
        if (isnan(f)) sparsity.insert(0, i) = 1;
    }
    return sparsity;
}

/// Detect the sparsity pattern of a Jacobian matrix by setting an element of x
/// to NaN and examining which constraint equations end up as NaN (and therefore
/// depend on that element of x).
template <typename T>
SparsityPattern
calc_jacobian_sparsity_with_nan(const Eigen::VectorXd& x0,
        int num_outputs,
        std::function<void(const VectorX<T>&, VectorX<T>&)> function) {
    using std::isnan;
    using tropter::isnan;
    SparsityPattern sparsity(num_outputs, (int)x0.size());
    VectorX<T> x = x0.cast<T>();
    VectorX<T> output(num_outputs);
    for (int j = 0; j < (int)x0.size(); ++j) {
        output.setZero();
        x[j] = std::numeric_limits<double>::quiet_NaN();
        function(x, output);
        x[j] = x0[j];
        for (int i = 0; i < (int)num_outputs; ++i) {
            if (isnan(output[i])) sparsity.set_nonzero(i, j);
        }
    }
    return sparsity;
}

// TODO upper triangular. TODO return compressed row sparsity?
/* TODO
template <typename T>
Eigen::SparseMatrix<bool>
calc_hessian_sparsity_with_nan(
        const Eigen::VectorXd& x0,
        std::function<T(const VectorX<T>&)>& function) {
    // TODO this is a conservative estimate.
    Eigen::SparseMatrix<bool> grad_sparsity =
            calc_gradient_sparsity_with_nan(x0, function);
    return (grad_sparsity.transpose() * grad_sparsity)
            .triangularView<Eigen::Upper>();
}
*/

/// Detect the sparsity pattern of a Hessian matrix by perturbing x and
/// examining if the function value is affected by the perturbation.
template <typename T>
SymmetricSparsityPattern
calc_hessian_sparsity_with_perturbation(
        const Eigen::VectorXd& x0,
        const std::function<T(const VectorX<T>&)>& function) {
    SymmetricSparsityPattern sparsity((int)x0.size());
    VectorX<T> x = x0.cast<T>();
    double eps = 1e-5;
    T f0 = function(x);
    // TODO cache evaluations.
    for (int i = 0; i < (int)x0.size(); ++i) {
        for (int j = i; j < (int)x0.size(); ++j) {
            x[i] += eps;
            T f_i = function(x);
            x[j] += eps;
            T f_ij = function(x);
            x[i] = x0[i];
            T f_j = function(x);
            x[j] = x0[j];
            // Finite difference numerator.
            //std::cout << "DEBUG " << i << " " << j << " " <<
            //        f_ij - f_i - f_j + f0 << std::endl;
            if ((f_ij - f_i - f_j + f0) != 0)
                sparsity.set_nonzero(i, j);
        }
    }
    // TODO std::cout << "DEBUG " << sparsity << std::endl;
    return sparsity;
}

} //namespace tropter

#endif // TROPTER_FINITEDIFFERENCE_H_
