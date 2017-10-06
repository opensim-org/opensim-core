
#include "GraphColoring.h"
#include <ColPack/ColPackHeaders.h>

using namespace tropter;

// We must implement the destructor in a context where ColPack's coloring
// class is complete (since it's used in a unique ptr member variable.).
JacobianColoring::~JacobianColoring() {}

JacobianColoring::JacobianColoring(int num_rows, int num_cols,
        const std::vector<std::vector<unsigned int>>& sparsity) {

    assert((int)sparsity.size() == num_rows);

    // Initially, we store the sparsity structure in ADOL-C's compressed row
    // format, since this is what ColPack accepts.
    // This format, as described in the ADOL-C manual, is a 2-Dish array.
    // The length of the first dimension is the number of rows in the Jacobian.
    // The first element of each row is the number of nonzeros in that row of
    // the Jacobian. The remaining elements are the column indices of those
    // nonzeros. The length of each row (the second dimension) is
    // num_nonzeros_in_the_row + 1.
    // Create a lambda that deletes the 2D C array.
    auto unsigned_int_2d_deleter = [num_rows](unsigned** x) {
        std::for_each(x, x + num_rows, std::default_delete<unsigned[]>());
        delete [] x;
    };
    m_sparsity_ADOLC_format = UnsignedInt2DPtr(
            new unsigned*[num_rows], unsigned_int_2d_deleter);
    for (int i = 0; i < (int)num_rows; ++i) {
        const auto& col_idx_for_nonzeros = sparsity[i];
        const auto num_nonzeros_this_row = col_idx_for_nonzeros.size();
        m_sparsity_ADOLC_format[i] = new unsigned[num_nonzeros_this_row+1];
        m_sparsity_ADOLC_format[i][0] = (unsigned)num_nonzeros_this_row;
        std::copy(col_idx_for_nonzeros.begin(), col_idx_for_nonzeros.end(),
                // Skip over the first element.
                m_sparsity_ADOLC_format[i] + 1);
        m_num_nonzeros += num_nonzeros_this_row;
    }

    // Determine the efficient perturbation directions.
    // ------------------------------------------------
    m_coloring.reset(new ColPack::BipartiteGraphPartialColoringInterface(
                    SRC_MEM_ADOLC, // We're using the ADOLC sparsity format.
                    m_sparsity_ADOLC_format.get(), // Sparsity.
                    num_rows, num_cols));

    // ColPack will allocate and store the seed matrix in jacobian_seed_raw.
    double** jacobian_seed_raw = nullptr;
    int jacobian_seed_num_rows; // Should be num_cols.
    int jacobian_seed_num_cols; // Number of seeds.
    m_coloring->GenerateSeedJacobian_unmanaged(&jacobian_seed_raw,
            &jacobian_seed_num_rows, &jacobian_seed_num_cols, // Outputs.
            // Copied from what ADOL-C uses in generate_seed_jac():
            "SMALLEST_LAST", "COLUMN_PARTIAL_DISTANCE_TWO");
    assert(jacobian_seed_num_rows == num_cols);
    // Convert the seed matrix into an Eigen Matrix for ease of use; delete
    // the memory that ColPack created for the seed matrix.
    const int num_jacobian_seeds = jacobian_seed_num_cols;
    m_seed.resize(jacobian_seed_num_rows, jacobian_seed_num_cols);
    for (int i = 0; i < jacobian_seed_num_rows; ++i) {
        for (int j = 0; j < jacobian_seed_num_cols; ++j) {
            m_seed(i, j) = jacobian_seed_raw[i][j];
        }
        delete [] jacobian_seed_raw[i];
    }
    delete [] jacobian_seed_raw;


    // Obtain sparsity pattern format to return.
    // -----------------------------------------
    // Get Jacobian row and column indices in the same order that ColPack
    // will use in recover() when recovering the sparse Jacobian from the
    // dense compressed Jacobian.
    // Allocate the recovery object used in recover().
    m_recovery.reset(new ColPack::JacobianRecovery1D());
    // Allocate memory for the compressed jacobian; we use this in recover().
    // Create a lambda that deletes the 2D C array.
    auto double_2d_deleter = [num_rows](double** x) {
        std::for_each(x, x + num_rows, std::default_delete<double[]>());
        delete [] x;
    };
    m_jacobian_compressed = Double2DPtr(new double*[num_rows],
            double_2d_deleter);
    for (int i = 0; i < (int)num_rows; ++i) {
        // We don't actually care about the value of m_jacobian_compressed here;
        // no need to set values.
        m_jacobian_compressed[i] = new double[num_jacobian_seeds];
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
        std::vector<unsigned int>& row_indices,
        std::vector<unsigned int>& col_indices) const {
    row_indices = m_recovered_row_indices;
    col_indices = m_recovered_col_indices;
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
