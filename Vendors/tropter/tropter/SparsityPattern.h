#ifndef TROPTER_SPARSITYPATTERN_H
#define TROPTER_SPARSITYPATTERN_H
// ----------------------------------------------------------------------------
// tropter: SparsityPattern.h
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
#include <vector>
#include <set>

namespace tropter {

/// A sparsity pattern in ADOL-C's compressed row format.
/// This format is a 2-Dish array. The length of the first dimension is
/// the number of rows in the Hessian. Each element represents a row
/// and is a vector of the column indices of the nonzeros in that row.
/// The length of each row (the inner dimension) is the number of
/// nonzeros in that row. Indices start at 0 (e.g., the top left element of a
/// matrix has indices (0, 0)). More information about this format can be found
/// in ADOL-C's manual.
using CompressedRowSparsity = std::vector<std::vector<unsigned int>>;


/// This represents the sparsity pattern of a matrix.
class SparsityPattern {
public:
    SparsityPattern(int num_rows, int num_cols)
        : m_num_rows(num_rows), m_num_cols(num_cols) {}
    SparsityPattern(int num_rows, int num_cols,
        const std::vector<unsigned int>& row_indices,
        const std::vector<unsigned int>& col_indices);
    /// Construct a sparsity pattern of a matrix with dimensions 1 x
    /// num_cols, whose nonzero entries are specified by nonzero_col_indices.
    SparsityPattern(int num_cols,
        const std::vector<unsigned int>& nonzero_col_indices);
    /// Set all elements to be nonzero.
    virtual void set_dense();
    /// Set a single entry of the matrix as nonzero.
    /// This function has the same effect if called one or more times with the
    /// same arguments.
    virtual void set_nonzero(unsigned int row_index, unsigned int col_index);
    /// Add in nonzeros from `other`'s nonzeros. This matrix and `other` must
    /// have the same dimensions.
    void add_in_nonzeros(const SparsityPattern& other);

    int get_num_rows() const { return m_num_rows; }
    int get_num_cols() const { return m_num_cols; }
    int get_num_nonzeros() const { return (int)m_sparsity.size(); }

    CompressedRowSparsity convert_to_CompressedRowSparsity() const;

    /// Write the sparsity pattern to a file, which can be plotted with the
    /// plot_sparsity.py script that comes with tropter.
    void write(const std::string& filename);

protected:
    int m_num_rows;
    int m_num_cols;
    friend class SymmetricSparsityPattern;
    std::set<std::pair<unsigned int, unsigned int>> m_sparsity;
};


/// This class represents the sparsity pattern of a (square) symmetric
/// matrix (the nonzeros are only provided for the upper triangle).
class SymmetricSparsityPattern : public SparsityPattern {
public:
    /// N is the number of rows and number of columns.
    explicit SymmetricSparsityPattern(int N) : SparsityPattern(N, N) {}

    void set_dense() override;

    /// Only upper triangular elements can be appended.
    void set_nonzero(unsigned int row_index, unsigned int col_index) override;
    /// Add in a nonzero block, placing the block's upper left corner at
    /// (startindex, startindex) in this matrix.
    /// Note, no nonzeros are "removed", only added.
    void set_nonzero_block(unsigned int startindex,
        SymmetricSparsityPattern& block);

    /// Create a non-symmetric sparsity pattern of this matrix where the
    /// lower triangle is filled in by mirroring the upper triangle.
    SparsityPattern convert_full() const;

    /// If jac_sparsity holds the sparsity pattern for the first derivatives
    /// (Jacobian) of a function, then this function returns a conservative
    /// estimate of the sparsity pattern for the Hessian of that function. Let f
    /// be the function, J be the Jacobian of f, and S1 be a binary matrix
    /// containing the sparsity pattern of J. This function returns S1^T S1.
    /// See Patterson, Michael A., and Anil V. Rao. "GPOPS-II: A MATLAB
    /// software for solving multiple-phase optimal control problems using
    /// hp-adaptive Gaussian quadrature collocation methods and sparse
    /// nonlinear programming." ACM Transactions on Mathematical Software
    /// (TOMS) 41.1 (2014): 1.
    static SymmetricSparsityPattern create_from_jacobian_sparsity(
        const SparsityPattern& jac_sparsity);
};


/// Detect the sparsity pattern of a gradient vector by setting an element of
/// x to NaN and examining if the function value ends up as NaN.
template <typename T>
SparsityPattern
    calc_gradient_sparsity_with_nan(const Eigen::VectorXd& x0,
        std::function<T(const VectorX<T>&)>& function) {
    using std::isnan;
    using tropter::isnan;
    SparsityPattern sparsity(1, (int)x0.size());
    VectorX<T> x = x0.cast<T>();
    T f;
    for (int i = 0; i < (int)x.size(); ++i) {
        x[i] = std::numeric_limits<double>::quiet_NaN();
        f = function(x);
        x[i] = x0[i];
        if (isnan(f)) sparsity.set_nonzero(0, i);
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

/// Detect the sparsity pattern of a gradient vector by perturbing x and
/// examining if the function value is affected by the perturbation.
template <typename T>
SparsityPattern
    calc_gradient_sparsity_with_perturbation(const Eigen::VectorXd& x0,
        std::function<T(const VectorX<T>&)>& function) {
    using std::isnan;
    using tropter::isnan;
    SparsityPattern sparsity(1, (int)x0.size());
    VectorX<T> x = x0.cast<T>();
    double eps = 1e-5;
    T f0 = function(x0);
    T f;
    for (int i = 0; i < (int)x.size(); ++i) {
        x[i] += eps;
        f = function(x);
        x[i] = x0[i];
        if ((f - f0) != 0) sparsity.set_nonzero(0, i);
    }
    return sparsity;
}

/// Detect the sparsity pattern of a Jacobian matrix by perturbing x and
/// examining how the constraint values are affected.
/// A warning is given if NaNs are encountered, in which case row_names and
/// col_names, if provided, are used to make the warning more informative.
template <typename T>
SparsityPattern
    calc_jacobian_sparsity_with_perturbation(const Eigen::VectorXd& x0,
        int num_outputs,
        std::function<void(const VectorX<T>&, VectorX<T>&)> function,
        const std::vector<std::string>& row_names = {},
        const std::vector<std::string>& col_names = {}) {
    using std::isnan;
    using tropter::isnan;
    SparsityPattern sparsity(num_outputs, (int)x0.size());
    VectorX<T> x = x0.cast<T>();
    double eps = 1e-5;
    VectorX<T> output0(num_outputs);
    function(x, output0);
    VectorX<T> output(num_outputs);
    VectorX<T> diff(num_outputs);
    for (int j = 0; j < (int)x0.size(); ++j) {
        output.setZero();
        x[j] += eps;
        function(x, output);
        x[j] = x0[j];
        diff = output - output0;
        for (int i = 0; i < (int)num_outputs; ++i) {
            if (std::isnan(diff[i])) {
                std::cout << "[tropter] Warning: NaN encountered when "
                    "detecting sparsity of Jacobian; entry (";
                if (col_names.empty() || row_names.empty())
                    std::cout << i << ", " << j;
                else
                    std::cout << row_names[i] << ", "
                    << col_names[j];
                std::cout << ")." << std::endl;
                // Set non-zero here just in case this Jacobian element is
                // important. 
                sparsity.set_nonzero(i, j);
            }
            if (diff[i] != 0) sparsity.set_nonzero(i, j);
        }
        diff.setZero();
    }
    return sparsity;
}

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

/// This struct can hold the sparsity pattern of a matrix in "coordinate
/// format": two vectors holding the row and column indicies of nonzeros in a
/// matrix. For example, (row[0], col[0]) is location the first nonzero.
struct SparsityCoordinates {
    std::vector<unsigned int> row;
    std::vector<unsigned int> col;
};

} // tropter

#endif // TROPTER_SPARSITYPATTERN_H
