#include <ColPack/ColPackHeaders.h>
#include <vector>
#include <limits>
#include <functional>
#include <cmath>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

//void func(int n, int /*m*/, double* x, double* values) {
VectorXd func(const VectorXd& x) {
    VectorXd y(x.size());
    for (int i = 0; i < x.size(); ++i) {
        y[i] = x[i] * x[i];
        if (i < x.size() - 1) {
            y[i] += x[i + 1] * x[i + 1];
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

private:
    unsigned int m_num_rows;
    unsigned int m_num_columns;
    unsigned int m_num_nonzeros = 0;
    std::vector<unsigned int> m_row_indices;
    std::vector<unsigned int> m_column_indices;
};

SparsityPattern compute_sparsity(int n, int m,
        const std::function<VectorXd (const VectorXd&)> f) {
        //const std::function<void (int, int, double*, double*)> f) {
    SparsityPattern sparsity(m, n);
    //std::vector<double> x(n, 0);
    VectorXd x = VectorXd::Zero(n);
    for (int j = 0; j < n; ++j) {
        VectorXd y = VectorXd::Zero(m);
        x[j] = NaN;
        y = func(x);
        x[j] = 0;
        std::cout << std::endl;
        for (int i = 0; i < m; ++i) {
            std::cout << y[i] << std::endl;
            if (std::isnan(y[i])) {
                sparsity.add_nonzero(i, j);
            }
        }
    }
    return sparsity;
}

class SparseJacobian {

public:
    void print() const {

    }
private:
    std::vector<unsigned int> row_indices;
    std::vector<unsigned int> column_indices;
    std::vector<double> values;
};

SparseJacobian compute_jacobian(
        const std::function<VectorXd (const VectorXd&)> f,
        //const std::function<void (int, int, double*, double*)> f,
        const VectorXd& x) {
    const int n = x.size();
    auto sparsity = compute_sparsity(n, n, func);
    sparsity.print();

    auto sparsity_cr = sparsity.convert_to_ADOLC_compressed_row_format();
    for (int i = 0; i < n; ++i) {
        std::cout << i << ":";
        for (int j = 1; j <= sparsity_cr[i][0]; ++j) {
            std::cout << " " << sparsity_cr[i][j];
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
    ColPack::BipartiteGraphPartialColoringInterface bgpci(SRC_MEM_ADOLC,
            sparsity_cr, n, n);
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
    std::cout << "Eigen seed mat \n" << seed_mat << std::endl;

    int num_seeds = seed_column_count;


    VectorXd y0(n);
}

int main() {
    const int n = 4;
    VectorXd x(n);
    x << 2, 1, 5, 3;
    //std::vector<double> y(n);
    VectorXd y = func(x);
    //func(n, n, x.data(), y.data());
    for (int i = 0; i < n; ++i) {
        std::cout << y[i] << std::endl;
    }

    SparseJacobian jac = compute_jacobian(func, x);
    jac.print();

    std::cout << "DEBUG END" << std::endl;
    return 0;
}



