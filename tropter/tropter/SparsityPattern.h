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
    /// Construct a sparsity pattern of a matrix with dimensions 1 x
    /// num_cols, whose nonzero entries are specified by nonzero_col_indices.
    SparsityPattern(int num_cols,
            const std::vector<unsigned int>& nonzero_col_indices);
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

    /// Only upper triangular elements can be appended.
    void set_nonzero(unsigned int row_index, unsigned int col_index) override;
    /// Add in a nonzero block, placing the block's upper left corner at
    /// (irowstart, icolstart) in this matrix.
    /// Note, no nonzeros are "removed", only added.
    void set_nonzero_block(unsigned int irowstart, unsigned int icolstart,
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

} // tropter

#endif // TROPTER_SPARSITYPATTERN_H
