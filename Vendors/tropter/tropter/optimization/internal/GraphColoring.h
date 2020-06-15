#ifndef TROPTER_OPTIMIZATION_INTERNAL_GRAPHCOLORING_H
#define TROPTER_OPTIMIZATION_INTERNAL_GRAPHCOLORING_H
// ----------------------------------------------------------------------------
// tropter: GraphColoring.h
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

#include <tropter/SparsityPattern.h>

#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <vector>
#include <memory>

namespace ColPack {
class BipartiteGraphPartialColoringInterface;
class JacobianRecovery1D;
class GraphColoringInterface;
class HessianRecovery;
}

namespace tropter {
namespace optimization {

namespace internal {
// We resort to using a unique_ptr with a custom deleter because ColPack
// requires that we provide the sparsity structure as two-dimensional C
// array.
using UnsignedInt2DPtr =
        std::unique_ptr<unsigned* [], std::function<void(unsigned**)>>;
using Double2DPtr =
        std::unique_ptr<double*[], std::function<void(double**)>>;
} // namespace internal

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
    /// @param sparsity
    ///     The sparsity pattern should be in ADOL-C's compressed row format.
    ///     This format is a 2-Dish array. The length of the first dimension is
    ///     the number of rows in the Jacobian. Each element represents a row
    ///     and is a vector of the column indices of the nonzeros in that row.
    ///     The length of each row (the inner dimension) is the number of
    ///     nonzeros in that row. More information about this format can be
    ///     found in ADOL-C's manual.
    ///     TODO update comment.
    JacobianColoring(SparsityPattern sparsity);

    ~JacobianColoring();

    /// The number of nonzero entries in the Jacobian.
    int get_num_nonzeros() const { return m_num_nonzeros; }

    const SparsityPattern& get_sparsity() const { return m_sparsity; }

    /// This matrix has dimensions num_columns x num_seeds, where num_seeds is
    /// the ("minimal") number of directions in which to perturb (num_columns
    /// is the number of variables). Each column of this matrix is a
    /// perturbation direction. The compressed Jacobian is computed by
    /// perturbing in each of these directions.
    const Eigen::MatrixXd& get_seed_matrix() const { return m_seed; }

    /// Get the sparsity pattern in coordinate (row, column) format.
    /// The length of both arguments will be the number of nonzeros.
    /// This is the coordinate format used by ColPack in recover().
    void get_coordinate_format(SparsityCoordinates& sparsity) const;

    /// Given a compressed dense Jacobian (probably computed using finite
    /// differences by perturbing by each of the seed's columns), recover the
    /// entries of the sparse Jacobian corresponding to the coordinate
    /// format with (row, col) coordinates given by get_coordinate_format().
    void recover(const Eigen::MatrixXd& jacobian_compressed,
            double* jacobian_sparse_coordinate_format);

    /// Utility function to create an Eigen sparse matrix from the coordinate
    /// format (perhaps as returned from recover()).
    void convert(const double* const jacobian_sparse_coordinate_format,
             Eigen::SparseMatrix<double>& mat) const;

private:

    /// Recover from m_jacobian_compressed.
    void recover_internal(double* jacobian_sparse_coordinate_format);

    const SparsityPattern m_sparsity;
    const int m_num_rows = 0;
    const int m_num_cols = 0;
    const int m_num_nonzeros = 0;

    // ColPack objects for (a) determining the directions in which to perturb
    // the variables to compute the Jacobian and (b) recovering the sparse
    // Jacobian (to pass to the optimization solver) after computing finite
    // differences.
    mutable std::unique_ptr<ColPack::BipartiteGraphPartialColoringInterface>
            m_coloring;
    mutable std::unique_ptr<ColPack::JacobianRecovery1D> m_recovery;

    // We determine the sparsity structure of the Jacobian (by propagating
    // NaNs through the constraint function) and hold the result in this
    // variable to pass to ColPack methods.
    internal::UnsignedInt2DPtr m_sparsity_ADOLC_format;

    // This has dimensions num_variables x num_perturbation_directions. All
    // entries are either 0 or 1, and each column is a direction in which we
    // will perturb the variables.
    Eigen::MatrixXd m_seed;

    // Working memory to hold onto the compressed Jacobian calculation to pass
    // to ColPack.
    mutable internal::Double2DPtr m_jacobian_compressed;

    // These variables store the output of ColPack's recovery routine.
    mutable std::vector<unsigned int> m_recovered_row_indices;
    mutable std::vector<unsigned int> m_recovered_col_indices;

};

/// [1] Bohme TJ, Frank B. Hybrid Systems, Optimal Control and Hybrid
/// Vehicles: Theory, Methods and Applications. Springer 2017.
/// [2] Gebremedhin, Assefaw Hadish, Fredrik Manne, and Alex Pothen. "What color
/// is your Jacobian? Graph coloring for computing derivatives." SIAM review
/// 47.4 (2005): 629-705.
class HessianColoring {
public:
    HessianColoring(const SymmetricSparsityPattern& sparsity);

    ~HessianColoring();

    /// This matrix has dimensions num_variables x num_seeds, where num_seeds is
    /// the ("minimal") number of directions in which to perturb. Each column of
    /// this matrix is a perturbation direction.
    const Eigen::MatrixXd& get_seed_matrix() const { return m_seed; }

    /// Get the sparsity pattern in coordinate (row, column) format.
    /// The length of both arguments will be the number of nonzeros.
    /// This is the coordinate format used by ColPack in recover().
    void get_coordinate_format(SparsityCoordinates& sparsity) const;

    /// Convert a compressed Hessian (num_variables x num_seeds) into the
    /// corresponding coordinate format with (row, col) coordinates given by
    /// get_coordinate_format().
    void recover(const Eigen::MatrixXd& hessian_compressed,
            double* hessian_sparse_coordinate_format);

    /// Convert a compressed Hessian (num_variables x num_seeds) into an
    /// Eigen SparseMatrix.
    void recover(const Eigen::MatrixXd& hessian_compressed,
            Eigen::SparseMatrix<double>& hessian_recovered);

    /// Utility function to create an Eigen sparse matrix from the coordinate
    /// format (perhaps as returned from recover()). Only the upper triangle
    /// is filled.
    void convert(const double* const hessian_sparse_coordinate_format,
            Eigen::SparseMatrix<double>& mat) const;

    /// Utility function to convert an Eigen sparse matrix into coordinate
    /// format. Only the elements known to this class as nonzeros are taken
    /// from `mat`. If `mat` doesn't contain an element for what we know to
    /// be a nonzero, then that element is set to 0.
    void convert(const Eigen::SparseMatrix<double>& mat,
            double* hessian_sparse_coordinate_format) const;

private:

    /// Recover from m_hessian_compressed.
    void recover_internal(double* hessian_sparse_coordinate_format);

    enum class Mode {
        /// Results in fewer seeds but requires solving a linear system.
        Indirect,
        /// Star coloring. Results in more seeds.
        Direct
    };

    // See ADOL-C documentation, Bohme and Frank 2017, and Gebremedhin 2005
    // for guidance on choosing the mode.
    Mode m_mode = Mode::Indirect;

    const int m_num_vars;
    int m_num_nonzeros = 0;

    // ColPack objects for (a) determining the directions in which to perturb
    // the variables to compute the Hessian and (b) recovering the sparse
    // Jacobian (to pass to the optimization solver) after computing finite
    // differences.
    mutable std::unique_ptr<ColPack::GraphColoringInterface> m_coloring;
    mutable std::unique_ptr<ColPack::HessianRecovery> m_recovery;

    internal::UnsignedInt2DPtr m_sparsity_ADOLC_format;

    // This has dimensions num_variables x num_perturbation_directions. All
    // entries are either 0 or 1, and each column is a direction in which we
    // will perturb the variables.
    Eigen::MatrixXd m_seed;

    mutable internal::Double2DPtr m_hessian_compressed;

    // These variables store the output of ColPack's recovery routine.
    mutable std::vector<unsigned int> m_recovered_row_indices;
    mutable std::vector<unsigned int> m_recovered_col_indices;

};

} // namespace optimization
} // namespace tropter

#endif // TROPTER_OPTIMIZATION_INTERNAL_GRAPHCOLORING_H
