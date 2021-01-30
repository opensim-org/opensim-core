// ----------------------------------------------------------------------------
// tropter: SparsityPattern.cpp
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

#include "SparsityPattern.h"

#include "Exception.hpp"

#include <Eigen/SparseCore>

using namespace tropter;

SparsityPattern::SparsityPattern(int num_rows, int num_cols,
    const std::vector<unsigned int>& row_indices,
    const std::vector<unsigned int>& col_indices)
    : m_num_rows(num_rows), m_num_cols(num_cols) {
    TROPTER_THROW_IF(row_indices.size() != col_indices.size(),
        "Expected row_indices and col_indices to have the same size.");
    for (int inz = 0; inz < (int)row_indices.size(); ++inz)
        set_nonzero(row_indices[inz], col_indices[inz]);
}

SparsityPattern::SparsityPattern(int num_cols,
    const std::vector<unsigned int>& nonzero_col_indices)
    : m_num_rows(1), m_num_cols(num_cols) {
    for (const auto& icol : nonzero_col_indices)
        set_nonzero(0, icol);
}

void SparsityPattern::set_dense() {
    for (int irow = 0; irow < m_num_rows; ++irow) {
        for (int icol = 0; icol < m_num_cols; ++icol) {
            set_nonzero(irow, icol);
        }
    }
}

void SparsityPattern::set_nonzero(unsigned int row_index,
    unsigned int col_index) {
    TROPTER_THROW_IF(row_index >= (unsigned)m_num_rows,
        "Expected row_index to be in [0, %i), but it's %i.",
        m_num_rows, row_index);
    TROPTER_THROW_IF(col_index >= (unsigned)m_num_cols,
        "Expected col_index to be in [0, %i), but it's %i.",
        m_num_cols, col_index);
    m_sparsity.emplace(row_index, col_index);
}

void SparsityPattern::add_in_nonzeros(const SparsityPattern& other) {
    TROPTER_THROW_IF(get_num_rows() != other.get_num_rows(),
        "Expected the same number of rows.");
    TROPTER_THROW_IF(get_num_cols() != other.get_num_cols(),
        "Expected the same number of columns.");
    for (const auto& other_entry : other.m_sparsity)
        set_nonzero(other_entry.first, other_entry.second);
}

CompressedRowSparsity
SparsityPattern::convert_to_CompressedRowSparsity() const {
    CompressedRowSparsity crs(m_num_rows);
    for (int irow = 0; irow < m_num_rows; ++irow) {
        std::vector<unsigned int> nonzeros_this_row;
        for (const auto& nonzero : m_sparsity) {
            if ((int)nonzero.first == irow)
                nonzeros_this_row.push_back(nonzero.second);
        }
        std::sort(nonzeros_this_row.begin(), nonzeros_this_row.end());
        crs[irow] = nonzeros_this_row;
    }
    return crs;
}

void SparsityPattern::write(const std::string& filename) {
    std::ofstream file(filename);
    file << "num_rows=" << m_num_rows << std::endl;
    file << "num_cols=" << m_num_cols << std::endl;
    file << "row_indices,column_indices" << std::endl;
    for (const auto& entry : m_sparsity)
        file << entry.first << "," << entry.second << std::endl;
    file.close();
}



SparsityPattern SymmetricSparsityPattern::convert_full() const {
    SparsityPattern full(*this);
    for (const auto& entry : m_sparsity)
        // Swap row and col indicies.
        full.set_nonzero(entry.second, entry.first);
    return full;
}

SymmetricSparsityPattern
SymmetricSparsityPattern::create_from_jacobian_sparsity(
    const SparsityPattern& jac_sparsity) {
    Eigen::SparseMatrix<bool> S2;
    {
        // Use 'short' instead of 'bool' to avoid MSVC warning C4804.
        Eigen::SparseMatrix<short> S1(
            jac_sparsity.get_num_rows(), jac_sparsity.get_num_cols());
        S1.reserve(jac_sparsity.get_num_nonzeros());
        std::vector<Eigen::Triplet<short>> triplets;
        for (const auto& entry : jac_sparsity.m_sparsity)
            triplets.emplace_back(entry.first, entry.second, 1);
        S1.setFromTriplets(triplets.begin(), triplets.end());
        S1.makeCompressed();

        S2 = (S1.transpose() * S1).triangularView<Eigen::Upper>().cast<bool>();
    }

    SymmetricSparsityPattern output((int)S2.rows());
    for (int i = 0; i < S2.outerSize(); ++i) {
        for (Eigen::SparseMatrix<bool>::InnerIterator it(S2, i); it; ++it) {
            if (it.value())
                output.set_nonzero((int)it.row(), (int)it.col());
        }
    }
    return output;
}

void SymmetricSparsityPattern::set_dense() {
    for (int irow = 0; irow < m_num_rows; ++irow) {
        for (int icol = irow; icol < m_num_cols; ++icol) {
            set_nonzero(irow, icol);
        }
    }
}

void SymmetricSparsityPattern::set_nonzero(unsigned int row_index,
    unsigned int col_index) {
    TROPTER_THROW_IF(row_index > col_index, "Nonzeros must be in the "
        "upper triangle, but indices (%i, %i) were provided.",
        row_index, col_index);
    SparsityPattern::set_nonzero(row_index, col_index);
}

void SymmetricSparsityPattern::set_nonzero_block(
        unsigned int startindex,
        SymmetricSparsityPattern& block) {
    TROPTER_THROW_IF((int)startindex + block.get_num_rows() > get_num_rows(),
        "Block does not fit within this matrix (number of rows: %i, "
        "required number of rows to set block: %i).", get_num_rows(),
            (int)startindex + block.get_num_rows());
    TROPTER_THROW_IF((int)startindex + block.get_num_cols() > get_num_cols(),
        "Block does not fit within this matrix (number of columns: %i, "
        "required number of columns to set block: %i).", get_num_cols(),
            (int)startindex + block.get_num_cols());
    for (const auto& block_entries : block.m_sparsity) {
        set_nonzero(startindex + block_entries.first,
            startindex + block_entries.second);
    }
}





