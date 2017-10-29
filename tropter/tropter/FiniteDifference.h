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

namespace tropter {

using CompressedRowSparsity = std::vector<std::vector<unsigned int>>;

Eigen::SparseMatrix<bool> convert_to_Eigen_SparseMatrix(
        const CompressedRowSparsity& sparsity);

// TODO only takes the upper triangle.
CompressedRowSparsity convert_to_CompressedRowSparsity(
        const Eigen::SparseMatrix<bool>& mat);

// TODO
inline void add_in_sparsity(
        std::vector<Eigen::Triplet<bool>>& triplets,
        const CompressedRowSparsity& sub,
        int irowstart, int icolstart) {
    for (int isubrow = 0; isubrow < (int)sub.size(); ++isubrow) {
        for (const auto& isubcol : sub[isubrow]) {
            triplets.push_back({isubrow + irowstart, isubcol + icolstart, 1});
        }
    }
}

inline void add_in_sparsity(
        std::vector<Eigen::Triplet<bool>>& triplets,
        const Eigen::SparseMatrix<bool>& sub,
        int irowstart, int icolstart) {
    for (int i = 0; i < sub.outerSize(); ++i) {
        for (Eigen::SparseMatrix<bool>::InnerIterator it(sub, i); it; ++it) {
            triplets.push_back(
                    {(int)it.row() + irowstart, (int)it.col() + icolstart, 1});
        }
    }
}

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

template <typename T>
Eigen::SparseMatrix<bool>
calc_jacobian_sparsity_with_nan(const Eigen::VectorXd& x0,
        int num_outputs,
        std::function<void(const VectorX<T>&, VectorX<T>&)> function) {
    using std::isnan;
    using tropter::isnan;
    Eigen::SparseMatrix<bool> sparsity(num_outputs, x0.size());
    VectorX<T> x = x0.cast<T>();
    VectorX<T> output(num_outputs);
    for (int j = 0; j < (int)x0.size(); ++j) {
        output.setZero();
        x[j] = std::numeric_limits<double>::quiet_NaN();
        function(x, output);
        x[j] = x0[j];
        for (int i = 0; i < (int)num_outputs; ++i) {
            if (isnan(output[i])) sparsity.insert(i, j) = 1;
        }
    }
    return sparsity;
}

// TODO upper triangular. TODO return compressed row sparsity?
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

template <typename T>
Eigen::SparseMatrix<bool>
calc_hessian_sparsity_with_perturbation(
        const Eigen::VectorXd& x0,
        const std::function<T(const VectorX<T>&)>& function) {
    Eigen::SparseMatrix<bool> sparsity(x0.size(), x0.size());
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
            if ((f_ij - f_i - f_j + f0) != 0)
                sparsity.insert(i, j) = 1;
        }
    }
    std::cout << "DEBUG " << sparsity << std::endl;
    return sparsity;
}

} //namespace tropter

#endif // TROPTER_FINITEDIFFERENCE_H_
