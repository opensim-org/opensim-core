// ----------------------------------------------------------------------------
// tropter: FiniteDifference.cpp
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

#include "FiniteDifference.h"

namespace tropter {

Eigen::SparseMatrix<bool> convert_to_Eigen_SparseMatrix(
        const CompressedRowSparsity& sparsity) {
    Eigen::SparseMatrix<bool> mat(sparsity.size(), sparsity.size());
    int num_nonzeros = 0;
    for (const auto& row : sparsity) num_nonzeros += (int)row.size();
    mat.reserve(num_nonzeros);

    for (int i = 0; i < (int)sparsity.size(); ++i) {
        for (const auto& j : sparsity[i]) mat.insert(i, j) = 1;
    }
    mat.makeCompressed();
    return mat;
}
CompressedRowSparsity convert_to_CompressedRowSparsity(
        const Eigen::SparseMatrix<bool>& mat) {
    CompressedRowSparsity sparsity(mat.rows());
    for (int i = 0; i < mat.outerSize(); ++i) {
        for (Eigen::SparseMatrix<bool>::InnerIterator it(mat, i); it; ++it) {
            if (it.row() <= it.col()) // Upper triangle only.
                sparsity[it.row()].push_back((unsigned)it.col());
        }
    }
    return sparsity;
}

} // namespace tropter
