// ----------------------------------------------------------------------------
// tropter: GraphColoring.cpp
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

#include "GraphColoring.h"
#include <tropter/Exception.hpp>
#include <ColPack/ColPackHeaders.h>

using namespace tropter;
using namespace tropter::optimization;

// ----------------------------------------------------------------------------
// JacobianColoring
// ----------------------------------------------------------------------------

// Initially, we store the sparsity structure in ADOL-C's compressed row
// format, since this is what ColPack accepts.
// This format, as described in the ADOL-C manual, is a 2-Dish array.
// The length of the first dimension is the number of rows in the Jacobian.
// The first element of each row is the number of nonzeros in that row of
// the matrix. The remaining elements are the column indices of those
// nonzeros. The length of each row (the second dimension) is
// num_nonzeros_in_the_row + 1.
void convert_sparsity_format(
        const SparsityPattern& sparsity0,
        internal::UnsignedInt2DPtr& ADOLC_format) {
    auto sparsity = sparsity0.convert_to_CompressedRowSparsity();
    int num_rows = (int)sparsity.size();
    //std::cout << "DEBUG sparsity\n" << std::endl;
    //for (int i = 0; i < (int)num_rows; ++i) {
    //    std::cout << i << ":";
    //    for (const auto& elem : sparsity[i]) {
    //        std::cout << " " << elem;
    //    }
    //    std::cout << std::endl;
    //}
    // Create a lambda that deletes the 2D C array.
    auto unsigned_int_2d_deleter = [num_rows](unsigned** x) {
        std::for_each(x, x + num_rows, std::default_delete<unsigned[]>());
        delete [] x;
    };
    ADOLC_format = internal::UnsignedInt2DPtr(new unsigned*[num_rows],
            unsigned_int_2d_deleter);
    for (int i = 0; i < (int)num_rows; ++i) {
        const auto& col_idx_for_nonzeros = sparsity[i];
        const auto num_nonzeros_this_row = col_idx_for_nonzeros.size();
        ADOLC_format[i] = new unsigned[num_nonzeros_this_row+1];
        ADOLC_format[i][0] = (unsigned)num_nonzeros_this_row;
        std::copy(col_idx_for_nonzeros.begin(), col_idx_for_nonzeros.end(),
                // Skip over the first element.
                ADOLC_format[i] + 1);
    }
}

// We must implement the destructor in a context where ColPack's coloring
// class is complete (since it's used in a unique ptr member variable.).
JacobianColoring::~JacobianColoring() {}

JacobianColoring::JacobianColoring(SparsityPattern sparsity)
        : m_sparsity(std::move(sparsity)),
          m_num_rows(m_sparsity.get_num_rows()),
          m_num_cols(m_sparsity.get_num_cols()),
          m_num_nonzeros(m_sparsity.get_num_nonzeros()) {

    // TODO m_sparsity.check();
    convert_sparsity_format(m_sparsity, m_sparsity_ADOLC_format);

    // Determine the efficient perturbation directions.
    // ------------------------------------------------
    m_coloring.reset(new ColPack::BipartiteGraphPartialColoringInterface(
                    SRC_MEM_ADOLC, // We're using the ADOLC sparsity format.
                    m_sparsity_ADOLC_format.get(), // Sparsity.
                    m_num_rows, m_num_cols));

    // ColPack will allocate and store the seed matrix in jacobian_seed_raw.
    double** seed_raw = nullptr;
    int seed_num_rows; // Should be num_cols.
    int seed_num_cols; // Number of seeds.
    m_coloring->GenerateSeedJacobian_unmanaged(&seed_raw,
            &seed_num_rows, &seed_num_cols, // Outputs.
            // Copied from what ADOL-C uses in generate_seed_jac():
            "SMALLEST_LAST", "COLUMN_PARTIAL_DISTANCE_TWO");
    assert(seed_num_rows == m_num_cols);
    // Convert the seed matrix into an Eigen Matrix for ease of use; delete
    // the memory that ColPack created for the seed matrix.
    const int num_seeds = seed_num_cols;
    m_seed.resize(seed_num_rows, seed_num_cols);
    for (int i = 0; i < seed_num_rows; ++i) {
        for (int j = 0; j < seed_num_cols; ++j) {
            m_seed(i, j) = seed_raw[i][j];
        }
        delete [] seed_raw[i];
    }
    delete [] seed_raw;


    // Obtain sparsity pattern format to return.
    // -----------------------------------------
    // Get Jacobian row and column indices in the same order that ColPack
    // will use in recover() when recovering the sparse Jacobian from the
    // dense compressed Jacobian.
    // Allocate the recovery object used in recover().
    m_recovery.reset(new ColPack::JacobianRecovery1D());
    // Allocate memory for the compressed jacobian; we use this in recover().
    // Create a lambda that deletes the 2D C array.
    const int num_rows = m_num_rows;
    auto double_2d_deleter = [num_rows](double** x) {
        std::for_each(x, x + num_rows, std::default_delete<double[]>());
        delete [] x;
    };
    m_jacobian_compressed = internal::Double2DPtr(new double*[m_num_rows],
            double_2d_deleter);
    for (int i = 0; i < (int)m_num_rows; ++i) {
        // We don't actually care about the value of m_jacobian_compressed here;
        // no need to set values.
        m_jacobian_compressed[i] = new double[num_seeds];
    }
    // Our objective here is to set these vectors from the recovery routine.
    m_recovered_row_indices.resize(m_num_nonzeros);
    m_recovered_col_indices.resize(m_num_nonzeros);

    // Again, we don't actually need Jacobian values right now.
    std::vector<double> jacobian_values_dummy(m_num_nonzeros);
    double* jacobian_values_dummy_ptr = jacobian_values_dummy.data();

    // This will set m_recovered_(row|col)_indices.
    recover_internal(jacobian_values_dummy_ptr);
}

void JacobianColoring::get_coordinate_format(
        SparsityCoordinates& sparsity) const {
    sparsity.row = m_recovered_row_indices;
    sparsity.col = m_recovered_col_indices;
}

void JacobianColoring::recover(const Eigen::MatrixXd& jacobian_compressed,
        double* jacobian_sparse_coordinate_format) {
    assert(jacobian_compressed.cols() == m_seed.cols());

    // Convert jacobian_compressed into the format ColPack accepts.
    for (Eigen::Index iseed = 0; iseed < m_seed.cols(); ++iseed) {
        for (unsigned int i = 0; i < jacobian_compressed.rows(); ++i) {
            m_jacobian_compressed[i][iseed] = jacobian_compressed(i, iseed);
        }
    }

    recover_internal(jacobian_sparse_coordinate_format);
}

void JacobianColoring::recover_internal(
        double* jacobian_sparse_coordinate_format) {
    // Convert the dense compressed Jacobian into the sparse Jacobian layout
    // (specified as triplets {row indices, column indices, values}).
    unsigned int* row_ptr = m_recovered_row_indices.data();
    unsigned int* col_ptr = m_recovered_col_indices.data();
    m_recovery->RecoverD2Cln_CoordinateFormat_usermem(
            m_coloring.get(), // ColPack's graph coloring object.
            m_jacobian_compressed.get(), // Holds the finite differences.
            m_sparsity_ADOLC_format.get(), // Input sparsity pattern.
            &row_ptr, &col_ptr, // Row and col. indices of nonzeros.
            // Corresponding values in the Jacobian.
            &jacobian_sparse_coordinate_format);
}

void JacobianColoring::convert(
        const double* const jacobian_sparse_coordinate_format,
        Eigen::SparseMatrix<double>& mat) const {
    mat.resize(m_num_rows, m_num_cols);
    mat.reserve(m_num_nonzeros);
    for (int ijacnz = 0; ijacnz < m_num_nonzeros; ++ijacnz) {
        mat.insert(m_recovered_row_indices[ijacnz],
                m_recovered_col_indices[ijacnz]) =
                jacobian_sparse_coordinate_format[ijacnz];
    }
    mat.makeCompressed();
}


// ----------------------------------------------------------------------------
// HessianColoring
// ----------------------------------------------------------------------------

HessianColoring::HessianColoring(const SymmetricSparsityPattern& sparsity)
        : m_num_vars(sparsity.get_num_cols()),
          m_num_nonzeros(sparsity.get_num_nonzeros()) {

    convert_sparsity_format(sparsity.convert_full(), m_sparsity_ADOLC_format);

    // Determine the efficient perturbation directions.
    // ------------------------------------------------
    m_coloring.reset(new ColPack::GraphColoringInterface(
            SRC_MEM_ADOLC, // We're using the ADOLC sparsity format.
            m_sparsity_ADOLC_format.get(), // Sparsity.
            m_num_vars));

    // ColPack will allocate and store the seed matrix in seed_raw.
    double** seed_raw = nullptr;
    int seed_num_rows; // Should be num_vars.
    int seed_num_cols; // Number of seeds.
    std::string coloringVariant;
    if (m_mode == Mode::Indirect) {
        coloringVariant = "ACYCLIC_FOR_INDIRECT_RECOVERY";
    } else if (m_mode == Mode::Direct) {
        coloringVariant = "STAR";
    } else {
        assert(false);
    }
    m_coloring->GenerateSeedHessian_unmanaged(&seed_raw,
            &seed_num_rows, &seed_num_cols, // Outputs.
            "SMALLEST_LAST", coloringVariant);
    assert(seed_num_rows == m_num_vars);
    // Convert the seed matrix into an Eigen Matrix for ease of use; delete
    // the memory that ColPack created for the seed matrix.
    const int num_seeds = seed_num_cols;
    m_seed.resize(seed_num_rows, seed_num_cols);
    for (int i = 0; i < seed_num_rows; ++i) {
        for (int j = 0; j < seed_num_cols; ++j) {
            m_seed(i, j) = seed_raw[i][j];
        }
        delete [] seed_raw[i];
    }
    delete [] seed_raw;


    // Obtain sparsity pattern format to return.
    // -----------------------------------------
    // Get Hessian row and column indices in the same order that ColPack
    // will use in recover() when recovering the sparse Hessian from the
    // dense compressed Hessian.
    // Allocate the recovery object used in recover().
    m_recovery.reset(new ColPack::HessianRecovery());
    // Allocate memory for the compressed Hessian; we use this in recover().
    // Create a lambda that deletes the 2D C array.
    const int num_vars = m_num_vars;
    auto double_2d_deleter = [num_vars](double** x) {
        std::for_each(x, x + num_vars, std::default_delete<double[]>());
        delete [] x;
    };
    m_hessian_compressed = internal::Double2DPtr(new double*[num_vars],
            double_2d_deleter);
    for (int i = 0; i < (int)num_vars; ++i) {
        // We don't actually care about the value of m_hessian_compressed here;
        // no need to set values.
        m_hessian_compressed[i] = new double[num_seeds];
    }
    // Our objective here is to set these vectors from the recovery routine.
    m_recovered_row_indices.resize(m_num_nonzeros);
    m_recovered_col_indices.resize(m_num_nonzeros);

    // Again, we don't actually need Hessian values right now.
    std::vector<double> hessian_values_dummy(m_num_nonzeros);
    double* hessian_values_dummy_ptr = hessian_values_dummy.data();

    // This will set m_recovered_(row|col)_indices.
    recover_internal(hessian_values_dummy_ptr);

}

HessianColoring::~HessianColoring() {}

void HessianColoring::get_coordinate_format(
        SparsityCoordinates& sparsity) const {
    sparsity.row = m_recovered_row_indices;
    sparsity.col = m_recovered_col_indices;
}

void HessianColoring::recover(const Eigen::MatrixXd& hessian_compressed,
        double* hessian_sparse_coordinate_format) {
    assert(hessian_compressed.cols() == m_seed.cols());

    // Convert hessian_compressed into the format ColPack accepts.
    for (Eigen::Index iseed = 0; iseed < m_seed.cols(); ++iseed) {
        for (unsigned int i = 0; i < hessian_compressed.rows(); ++i) {
            m_hessian_compressed[i][iseed] = hessian_compressed(i, iseed);
        }
    }

    recover_internal(hessian_sparse_coordinate_format);
}

void HessianColoring::recover(const Eigen::MatrixXd& hessian_compressed,
        Eigen::SparseMatrix<double>& hessian_recovered) {
    // TODO allocate this vector as working memory.
    Eigen::VectorXd hessian_coordinate_format(m_num_nonzeros);
    recover(hessian_compressed, hessian_coordinate_format.data());
    convert(hessian_coordinate_format.data(), hessian_recovered);
}

void HessianColoring::recover_internal(
        double* hessian_sparse_coordinate_format) {
    // Convert the dense compressed Jacobian into the sparse Jacobian layout
    // (specified as triplets {row indices, column indices, values}).
    unsigned int* row_ptr = m_recovered_row_indices.data();
    unsigned int* col_ptr = m_recovered_col_indices.data();
    if (m_mode == Mode::Indirect) {
        m_recovery->IndirectRecover_CoordinateFormat_usermem(
                m_coloring.get(), // ColPack's graph coloring object.
                m_hessian_compressed.get(), // Holds the finite differences.
                m_sparsity_ADOLC_format.get(), // Input sparsity pattern.
                &row_ptr, &col_ptr, // Row and col. indices of nonzeros.
                // Corresponding values in the Jacobian.
                &hessian_sparse_coordinate_format);
    } else if (m_mode == Mode::Direct) {
        m_recovery->DirectRecover_CoordinateFormat_usermem(
                m_coloring.get(), // ColPack's graph coloring object.
                m_hessian_compressed.get(), // Holds the finite differences.
                m_sparsity_ADOLC_format.get(), // Input sparsity pattern.
                &row_ptr, &col_ptr, // Row and col. indices of nonzeros.
                // Corresponding values in the Jacobian.
                &hessian_sparse_coordinate_format);
    }
}

void HessianColoring::convert(
        const double* const hessian_sparse_coordinate_format,
        Eigen::SparseMatrix<double>& mat) const {
    mat.resize(m_num_vars, m_num_vars);
    mat.setZero();
    mat.reserve(m_num_nonzeros);
    std::vector<Eigen::Triplet<double>> triplets(m_num_nonzeros);
    for (int ijacnz = 0; ijacnz < m_num_nonzeros; ++ijacnz) {
        triplets[ijacnz] = Eigen::Triplet<double>(
                m_recovered_row_indices[ijacnz],
                m_recovered_col_indices[ijacnz],
                hessian_sparse_coordinate_format[ijacnz]);
    }
    mat.setFromTriplets(triplets.begin(), triplets.end());
    mat.makeCompressed();
}

void HessianColoring::convert(const Eigen::SparseMatrix<double>& mat,
        double* hessian_sparse_coordinate_format) const {
    for (int inz = 0; inz < m_num_nonzeros; ++inz) {
        const auto& i = m_recovered_row_indices[inz];
        const auto& j = m_recovered_col_indices[inz];
        hessian_sparse_coordinate_format[inz] = mat.coeff(i,j);
    }
}
