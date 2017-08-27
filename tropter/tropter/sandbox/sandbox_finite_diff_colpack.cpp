#include <ColPack/ColPackHeaders.h>
#include <vector>
#include <limits>
#include <functional>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <ctime>

using namespace Eigen;

VectorXd func(const VectorXd& x) {
    VectorXd y(x.size());
    for (int i = 0; i < x.size(); ++i) {
        y[i] = x[i] * x[i];
        if (i < x.size() - 3) {
            y[i] += x[i + 1] * x[i + 1] + x[i + 2] * x[i + 2] + x[i + 3] *
                    x[i + 3];
        }
    }
    return y;
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
    unsigned int** convert_to_ADOLC_compressed_row_format() const {
        unsigned int** pattern = new unsigned int*[m_num_rows];
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

private:
    unsigned int m_num_rows;
    unsigned int m_num_columns;
    unsigned int m_num_nonzeros = 0;
    std::vector<unsigned int> m_row_indices;
    std::vector<unsigned int> m_column_indices;
};

SparsityPattern compute_sparsity(int m, int n,
        const std::function<VectorXd (const VectorXd&)> f) {
    SparsityPattern sparsity(m, n);
    VectorXd x = VectorXd::Zero(n);
    for (int j = 0; j < n; ++j) {
        VectorXd y = VectorXd::Zero(m);
        x[j] = NaN;
        y = func(x);
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
    VectorXd y0 = func(x);
    const int m = y0.size();
    auto sparsity = compute_sparsity(m, n, func);
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
    ColPack::BipartiteGraphPartialColoringInterface bgpci(SRC_MEM_ADOLC,
            sparsity_cr, m, n);
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
        VectorXd y1 = func(x_working);
        x_working[icol] = x[icol];
        jacobian.col(icol) = (y1 - y0) / h;
    }
    std::cout << "jacobian " << std::endl;
    //std::cout << jacobian << std::endl;
    std::cout << "duration for dense jac: " << double(clock() -
            dense_begin) / CLOCKS_PER_SEC << std::endl;


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

    clock_t sparse_begin = clock();

    std::vector<Eigen::Triplet<double>> jacobian_triplets;
    for (int iseed = 0; iseed < num_seeds; ++iseed) {
        const VectorXd& direction = seed_mat.col(iseed);

        VectorXd p = eps * direction;
        VectorXd y1 = func(x + p);
        VectorXd deriv = (y1 - y0) / eps;

        const auto& columns_in_this_seed = seed_info[iseed];
        for (const auto& ijaccol : columns_in_this_seed) {


            for (const auto& ijacrow : sparsity_cc[ijaccol]) {
                jacobian_triplets.push_back(
                        {(int)ijacrow, (int)ijaccol, deriv[ijacrow]});
            }
        }
    }
    std::cout << "duration for sparse jac: " << double(clock() -
            sparse_begin) / CLOCKS_PER_SEC << std::endl;
    Eigen::SparseMatrix<double> jacobian_sparse(m, n);
    jacobian_sparse.setFromTriplets(jacobian_triplets.begin(),
            jacobian_triplets.end());
    //std::cout << "sparse jac\n" << jacobian_sparse << std::endl;

}

int main() {
    const int n = 1000;
    VectorXd x(n);
    for (int i = 0; i < n; ++i) {
        x[i] = rand();
    }
    VectorXd y = func(x);
    //for (int i = 0; i < n; ++i) {
    //    std::cout << y[i] << std::endl;
    //}

    //SparseJacobian jac = compute_jacobian(func, x);
    compute_jacobian(func, x);
    //jac.print();

    std::cout << "DEBUG END" << std::endl;
    return 0;
}



