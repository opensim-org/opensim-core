#ifndef TROPTER_GRAPHCOLORING_H
#define TROPTER_GRAPHCOLORING_H

#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace ColPack {
class BipartiteGraphPartialColoringInterface;
class JacobianRecovery1D;
}

namespace tropter {

/// This class determines the directions in which to perturb
/// This class supports computing sparse finite differences of a Jacobian
/// matrix. It is implemented using the ColPack graph coloring library.
/// The class can determine the directions in which to perturb the unknown
/// variables so as to minimize the number perturbations necessary to obtain
/// all nonzero entries of the Jacobian. The result is a "compressed" dense
/// Jacobian containing the derivative in each of the perturbation directions.
/// This class also supports recovering the sparse Jacobian from the compressed
/// dense Jacobian.
/// This is an internal class (not available from the interface).
class JacobianColoring {
public:

    /// Compute a graph coloring for from the given Jacobian sparsity pattern.
    ///
    /// @param num_rows
    ///     The number of rows in the Jacobian.
    /// @param num_cols
    ///     The number of columms in the Jacobian.
    /// @param sparsity
    ///     The sparsity pattern should be in ADOL-C's compressed row format.
    ///     This format is a 2-Dish array. The length of the first dimension is
    ///     the number of rows in the Jacobian. Each element represents a row
    ///     and is a vector of the column indices of the nonzeros in that row.
    ///     The length of each row (the inner dimension) is the number of
    ///     nonzeros in that row. More information about this format can be
    ///     found in ADOL-C's manual.
    JacobianColoring(int num_rows, int num_cols,
            const std::vector<std::vector<unsigned int>>& sparsity);

    ~JacobianColoring();

    /// This matrix has dimensions num_columns x num_seeds, where num_seeds is
    /// the ("minimal") number of directions in which to perturb (num_columns
    /// is the number of variables). Each column of this matrix is a
    /// perturbation direction. The compressed Jacobian is computed by
    /// perturbing in each of these directions.
    const Eigen::MatrixXd& get_seed_matrix() const { return m_seed; }

    /// Get the sparsity pattern in coordinate (row, column) format.
    /// The length of both arguments will be the number of nonzeros.
    void get_coordinate_format(std::vector<unsigned int>& row_indices,
            std::vector<unsigned int>& col_indices) const;

    /// Given a compressed dense Jacobian (probably computed using finite
    /// differences by perturbing by each of the seed's columns), recover the
    /// entries of the sparse Jacobian corresponding to the coordinate
    /// format with (row, col) coordinates given by get_coordinate_format().
    void recover(const Eigen::MatrixXd& jacobian_compressed,
            double* jacobian_sparse_coordinate_format);

private:

    /// Recover from m_jacobian_compressed.
    void recover_internal(double* jacobian_sparse_coordinate_format);

    int m_num_nonzeros = 0;

    // ColPack objects for (a) determining the directions in which to perturb
    // the variables to compute the Jacobian and (b) recovering the sparse
    // Jacobian (to pass to the optimization solver) after computing finite
    // differences.
    mutable std::unique_ptr<ColPack::BipartiteGraphPartialColoringInterface>
            m_coloring;
    mutable std::unique_ptr<ColPack::JacobianRecovery1D> m_recovery;

    // This has dimensions num_variables x num_perturbation_directions. All
    // entries are either 0 or 1, and each column is a direction in which we
    // will perturb the variables. This matrix is computed for us by
    // ColPack's graph coloring algorithms.
    Eigen::MatrixXd m_seed;

    // We determine the sparsity structure of the Jacobian (by propagating
    // NaNs through the constraint function) and hold the result in this
    // variable to pass to ColPack methods.
    // We resort to using a unique_ptr with a custom deleter because ColPack
    // requires that we provide the sparsity structure as two-dimensional C
    // array.
    using UnsignedInt2DPtr =
            std::unique_ptr<unsigned*[], std::function<void(unsigned**)>>;
    UnsignedInt2DPtr m_sparsity_ADOLC_format;

    // Working memory to hold onto the compressed Jacobian calculation to pass
    // to ColPack.
    // We resort to using a unique_ptr with a custom deleter because ColPack
    // requires that we provide the data as a two-dimensional C array.
    using Double2DPtr =
            std::unique_ptr<double*[], std::function<void(double**)>>;
    mutable Double2DPtr m_jacobian_compressed;

    // These variables store the output of ColPack's recovery routine.
    mutable std::vector<unsigned int> m_recovered_row_indices;
    mutable std::vector<unsigned int> m_recovered_col_indices;

};

} // namespace tropter

#endif // TROPTER_GRAPHCOLORING_H
