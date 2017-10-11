// ----------------------------------------------------------------------------
// tropter: sandbox_finite_diff_colpack.cpp
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

#include <ColPack/ColPackHeaders.h>
#include <vector>
#include <limits>
#include <functional>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <ctime>

using namespace Eigen;

VectorXd constraints(const VectorXd& x) {
    VectorXd y(x.size());
    for (int i = 0; i < x.size(); ++i) {
        y[i] = x[i] * x[i];
        if (i < x.size() - 1) {
            y[i] += x[i + 1] * x[i + 1];
        }
        //if (i < x.size() - 3) {
        //    y[i] += x[i + 1] * x[i + 1] + x[i + 2] * x[i + 2] + x[i + 3] *
        //            x[i + 3];
        //}
    }
    return y;
}

double objective(const VectorXd& x) {
    double f = 0;
    for (int i = 0; i < x.size(); ++i) {
        f += x[i] * x[i] + log(x[0]) * x[i];
    }
    return f;
}

VectorXd compute_gradient(const std::function<double (const VectorXd&)> f,
        const VectorXd& x0) {
    VectorXd gradient(x0.size());
    VectorXd x = x0;
    const double y0 = f(x);
    double eps = std::sqrt(Eigen::NumTraits<double>::epsilon());
    double two_eps = 2 * eps;
    for (int i = 0; i < x.size(); ++i) {
        x[i] += eps;
        const double ypos = f(x);
        x[i] = x0[i] - eps;
        const double yneg = f(x);
        x[i] = x0[i];
        gradient[i] = (ypos - yneg) / two_eps;
    }
    return gradient;
}

static const double NaN = std::numeric_limits<double>::quiet_NaN();

// Based off ADOL-C example
// examples/additional_examples/sparse/sparse_jacobian.cpp
class SparsityPattern {
public:
    SparsityPattern(unsigned int num_rows, unsigned int num_columns) :
            m_num_rows(num_rows), m_num_columns(num_columns) {
    }
    void add_nonzero(unsigned int row_index, unsigned int column_index) {
        m_row_indices.push_back(row_index);
        m_column_indices.push_back(column_index);
        m_num_nonzeros++;
    }
    // TODO use smart pointers to handle this memory.
    std::unique_ptr<unsigned int*[], std::function<void(unsigned int**)>>
    convert_to_ADOLC_compressed_row_format() const {
        //unsigned int** pattern = new unsigned int*[m_num_rows];
        const unsigned int num_rows = m_num_rows;
        auto double_array_deleter = [num_rows](unsigned int** x) {
            std::for_each(x, x + num_rows,
                    std::default_delete<unsigned int[]>());
            delete [] x;
        };
        using DoubleArrayUnsignedIntPtr =
        std::unique_ptr<unsigned int*[], std::function<void(unsigned int**)>>;
        DoubleArrayUnsignedIntPtr pattern(
                new unsigned int*[m_num_rows], double_array_deleter);
        for (int i = 0; i < m_num_rows; ++i) {
            std::vector<unsigned int> col_indices_for_nonzeros_in_this_row;
            for (int inz = 0; inz < m_num_nonzeros; ++inz) {
                if (m_row_indices[inz] == i) {
                   col_indices_for_nonzeros_in_this_row.push_back
                           (m_column_indices[inz]);
                }
            }
            size_t num_nonzeros_in_this_row =
                    col_indices_for_nonzeros_in_this_row.size();
            pattern[i] = new unsigned int[num_nonzeros_in_this_row + 1];
            pattern[i][0] = num_nonzeros_in_this_row;
            for (size_t inzrow = 0; inzrow < num_nonzeros_in_this_row;
                 ++inzrow) {
                pattern[i][inzrow + 1] =
                        col_indices_for_nonzeros_in_this_row[inzrow];

            }
        }

        return pattern;
    }
    void print() const {
        for (size_t i = 0; i < m_num_nonzeros; ++i) {
            std::cout << "(" << m_row_indices[i] << "," <<
                    m_column_indices[i] << ")" <<
                    std::endl;
        }
    }

    std::vector<std::vector<unsigned int>> convert_to_compressed_column_format()
            const {
        std::vector<std::vector<unsigned int>> pattern(m_num_columns);
        for (int j = 0; j < m_num_columns; ++j) {
            for (int inz = 0; inz < m_num_nonzeros; ++inz) {
                if (m_column_indices[inz] == j) {
                    pattern[j].push_back(m_row_indices[inz]);
                }
            }
        }
        return pattern;
    }
    unsigned int num_nonzeros() const { return m_num_nonzeros; }
    const std::vector<unsigned int>& row_indices() const { return
                m_row_indices; }
    const std::vector<unsigned int>& col_indices() const { return
                m_column_indices; }

private:
    unsigned int m_num_rows;
    unsigned int m_num_columns;
    unsigned int m_num_nonzeros = 0;
    std::vector<unsigned int> m_row_indices;
    std::vector<unsigned int> m_column_indices;
};

SparsityPattern compute_jacobian_sparsity(int m, int n,
        const std::function<VectorXd (const VectorXd&)> f) {
    SparsityPattern sparsity(m, n);
    VectorXd x = VectorXd::Zero(n);
    for (int j = 0; j < n; ++j) {
        VectorXd y = VectorXd::Zero(m);
        x[j] = NaN;
        y = f(x);
        x[j] = 0;
        //std::cout << std::endl;
        for (int i = 0; i < m; ++i) {
            //std::cout << y[i] << std::endl;
            if (std::isnan(y[i])) {
                sparsity.add_nonzero(i, j);
            }
        }
    }
    return sparsity;
}

SparsityPattern compute_hessian_sparsity(int n,
        const std::function<double (const VectorXd&)> f,
        const VectorXd& x0) {
    SparsityPattern sparsity(n, n);
    double eps = std::sqrt(Eigen::NumTraits<double>::epsilon());
    double epsSquared = Eigen::NumTraits<double>::epsilon();
    double y;
    VectorXd x = x0;
    const double y0 = f(x0);
    for (int i = 0; i < n; ++i) {
        for (int j = i; j < n; ++j) {
            x[i] += eps;
            double perturb_i = f(x);
            x[j] += eps;
            double perturb_i_j = f(x);
            x[i] = x0[i];
            double perturb_j = f(x);
            x[j] = x0[j];
            double second_deriv = (perturb_i_j + y0 - perturb_i - perturb_j)
                    / epsSquared;
            //if ((perturb_i_j != y0) {
            //std::cout << i << " " << j << " " << second_deriv << std::endl;
            if (second_deriv > eps) { // TODO test condition.
                sparsity.add_nonzero(i, j);
            }
        }
    }
    return sparsity;
}


//class SparseJacobian {
//
//public:
//    void print() const {
//
//    }
//private:
//    std::vector<unsigned int> row_indices;
//    std::vector<unsigned int> column_indices;
//    std::vector<double> values;
//};

void compute_jacobian(
        const std::function<VectorXd (const VectorXd&)> f,
        const VectorXd& x) {
    const int n = x.size();
    VectorXd y0 = f(x);
    const int m = y0.size();
    auto sparsity = compute_jacobian_sparsity(m, n, f);
    //sparsity.print();

    auto sparsity_cr = sparsity.convert_to_ADOLC_compressed_row_format();
    //for (int i = 0; i < n; ++i) {
    //    std::cout << i << ":";
    //    for (int j = 1; j <= sparsity_cr[i][0]; ++j) {
    //        std::cout << " " << sparsity_cr[i][j];
    //    }
    //    std::cout << std::endl;
    //}
    //std::cout << std::endl;
    //std::unique_ptr<unsigned*[]> testing;
    //std::cout << typeid(testing).name() << std::endl;
    //std::cout << typeid(testing.get()).name() << std::endl;
    ColPack::BipartiteGraphPartialColoringInterface bgpci(SRC_MEM_ADOLC,
            sparsity_cr.get(), m, n);
    double** seed = nullptr;
    int seed_row_count;
    int seed_column_count;
    bgpci.GenerateSeedJacobian_unmanaged(&seed, &seed_row_count,
            &seed_column_count,
            "SMALLEST_LAST", "COLUMN_PARTIAL_DISTANCE_TWO");
    Eigen::MatrixXd seed_mat(seed_row_count, seed_column_count);
    for (int i = 0; i < seed_row_count; ++i) {
        for (int j = 0; j < seed_column_count; ++j) {
            seed_mat(i, j) = seed[i][j];
        }
        delete [] seed[i];
    }
    delete [] seed;
    //std::cout << "Eigen seed mat \n" << seed_mat << std::endl;

    int num_seeds = seed_column_count;

    clock_t dense_begin = clock();

    //Eigen::SparseMatrix<double> jacobian;
    Eigen::MatrixXd jacobian(m, n);

    // Compute finite difference.
    VectorXd x_working = x;

//    for (int iseed = 0; iseed < num_seeds; ++iseed) {
    double eps = std::sqrt(Eigen::NumTraits<double>::epsilon());
    //std::cout << eps << std::endl;
    for (int icol = 0; icol < n; ++icol) {
        double h = eps * std::abs(x_working[icol]);
        x_working[icol] += h;
        VectorXd y1 = f(x_working);
        x_working[icol] = x[icol];
        jacobian.col(icol) = (y1 - y0) / h;
    }
    std::cout << "jacobian " << std::endl;
    std::cout << jacobian << std::endl;
    std::cout << "duration for dense jac: " << double(clock() -
            dense_begin) / CLOCKS_PER_SEC << std::endl;


    /*
    std::vector<std::vector<unsigned int>> seed_info(num_seeds);
    for (int iseed = 0; iseed < num_seeds; ++iseed) {
        for (int j = 0; j < n; ++j) {
            if (seed_mat(j, iseed) == 1) {
                seed_info[iseed].push_back(j);
            }
        }
    }

    std::cout << "number of seeds " << num_seeds << std::endl;
    std::cout << "SPARSE JAC CALC " << std::endl;
    const auto sparsity_cc = sparsity.convert_to_compressed_column_format();
    */


    clock_t sparse_begin = clock();

    std::vector<Eigen::Triplet<double>> jacobian_triplets;
    Eigen::MatrixXd compressed_jacobian(m, num_seeds);
    for (int iseed = 0; iseed < num_seeds; ++iseed) {
        const VectorXd& direction = seed_mat.col(iseed);

        VectorXd p = eps * direction;
        VectorXd y1 = f(x + p);
        compressed_jacobian.col(iseed) = (y1 - y0) / eps;

        /*
        const auto& columns_in_this_seed = seed_info[iseed];
        for (const auto& ijaccol : columns_in_this_seed) {


            for (const auto& ijacrow : sparsity_cc[ijaccol]) {
                jacobian_triplets.push_back(
                        {(int)ijacrow, (int)ijaccol, deriv[ijacrow]});
            }
        }
         */
    }
    double** compressed_jacobian_raw = new double*[m];
    for (int i = 0; i < m; ++i) {
        compressed_jacobian_raw[i] = new double[num_seeds];
        // TODO replace with std::copy(compressed_jacobian.)
        for (int j = 0; j < num_seeds; ++j) {
            compressed_jacobian_raw[i][j] = compressed_jacobian(i, j);
        }
    }
    std::cout << "compressed jac \n" << compressed_jacobian << std::endl;
    ColPack::JacobianRecovery1D jacobian_recovery;
    std::vector<unsigned int> jacobian_row_indices_recovered(
            sparsity.num_nonzeros());
    //std::unique_ptr<unsigned int[]> jacobian_row_indices_recovered(
    //        new unsigned int[sparsity.num_nonzeros()]);
    std::vector<unsigned int> jacobian_col_indices_recovered(
            sparsity.num_nonzeros());
    std::vector<double> jacobian_recovered(sparsity.num_nonzeros());
    unsigned int* jricd = jacobian_row_indices_recovered.data();
    unsigned int* jcicd = jacobian_col_indices_recovered.data();
    double* jrd = jacobian_recovered.data();
    //unsigned int* jricd = new unsigned int[sparsity.num_nonzeros()];
    //unsigned int* jcicd = new unsigned int[sparsity.num_nonzeros()];
    //double* jrd = new double[sparsity.num_nonzeros()];
    //unsigned int** jricd = new unsigned int*;
    //unsigned int** jcicd = new unsigned int*;
    //double** jrd = new double*;
    jacobian_recovery.RecoverD2Cln_CoordinateFormat_usermem(&bgpci,
            compressed_jacobian_raw, sparsity_cr.get(),
            &jricd, &jcicd, &jrd);
    displayMatrix(compressed_jacobian_raw, m, num_seeds);
    //displayVector(*jricd, bgpci.GetEdgeCount());
    //displayVector(*jcicd, bgpci.GetEdgeCount());
    //displayVector(*jrd, bgpci.GetEdgeCount());

    for (int inz = 0; inz < sparsity.num_nonzeros(); ++inz) {
        std::cout << sparsity.row_indices()[inz] << " " <<
                jacobian_row_indices_recovered[inz] << " " <<
                sparsity.col_indices()[inz] << " " <<
                jacobian_col_indices_recovered[inz] << " " <<
                jacobian_recovered[inz] << std::endl;
    }

    std::cout << "duration for sparse jac: " << double(clock() -
            sparse_begin) / CLOCKS_PER_SEC << std::endl;
    /*
    Eigen::SparseMatrix<double> jacobian_sparse(m, n);
    jacobian_sparse.setFromTriplets(jacobian_triplets.begin(),
            jacobian_triplets.end());
    */
    //std::cout << "sparse jac\n" << jacobian_sparse << std::endl;

}

int main() {
    const int n = 5;
    VectorXd x(n);
    for (int i = 0; i < n; ++i) {
        //x[i] = rand();
        //x[i] = i; //rand();
        x[i] = 0.5 * double(i) + 0.105; //rand();
    }
    VectorXd y = constraints(x);
    std::cout << "y " << y << std::endl;
    //for (int i = 0; i < n; ++i) {
    //    std::cout << y[i] << std::endl;
    //}

    //SparseJacobian jac = compute_jacobian(constraints, x);
    compute_jacobian(constraints, x);
    //jac.print();


    // std::cout << x << std::endl;
    //clock_t hess_sparse_begin = clock();
    //SparsityPattern hessian_sparsity =
    //        compute_hessian_sparsity(n, objective, x);
    //std::cout << "duration for sparse hess: " << double(clock() -
    //        hess_sparse_begin) / CLOCKS_PER_SEC << std::endl;
    //hessian_sparsity.print();
    //std::cout << compute_gradient(objective, x) << std::endl;

    std::cout << "DEBUG END" << std::endl;
    return 0;
}



// Parallelize the FD calculation: this avoids issues with ADOL-C's
// non-parallelizability, and the fact that the constraint eval should happen
// for all time points at once (to handle constant parameters).
// However, there is an opportunity for greater parallelization if we also
// parallelize each contraint eval.

