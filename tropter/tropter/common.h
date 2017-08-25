#ifndef TROPTER_COMMON_H
#define TROPTER_COMMON_H

#ifdef _MSC_VER
    // Ignore warnings from ADOL-C headers.
    #pragma warning(push)
    // 'argument': conversion from 'size_t' to 'locint', possible loss of data.
    #pragma warning(disable: 4267)
#endif
#include <adolc/adouble.h>
#ifdef _MSC_VER
    #pragma warning(pop)
#endif

#include <Eigen/Dense>
#include <fstream>

// Eigen support for adouble as a scalar type. This allows Eigen to use
// epsilon() etc. on adouble.
// https://eigen.tuxfamily.org/dox/TopicCustomizing_CustomScalar.html
namespace Eigen {

template<> struct NumTraits<adouble> : NumTraits<double>
{
    typedef adouble Real;
    typedef adouble NonInteger;
    typedef adouble Nested;

    enum {
        IsComplex = 0,
        IsInteger = 0,
        IsSigned  = 1,
        RequireInitialization = 1,
        ReadCost = 1, // TODO
        AddCost = 3, // TODO
        MulCost = 3 // TODO
    };
};

} // namespace Eigen

namespace tropter {

using VectorXa = Eigen::Matrix<adouble, Eigen::Dynamic, 1>;
using MatrixXa = Eigen::Matrix<adouble, Eigen::Dynamic, Eigen::Dynamic>;

template<typename T>
using VectorX = Eigen::Matrix<T, Eigen::Dynamic, 1>;
template<typename T>
using MatrixX = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

template<typename T>
using Vector2 = Eigen::Matrix<T, 2, 1>;
template<typename T>
using Matrix2 = Eigen::Matrix<T, 2, 2>;


/// Write an Eigen matrix to a file in a CSV (comma-separated values) format.
/// If you specify column labels, you must specify one for each column of the
/// matrix.
template<typename Derived>
void write(const Eigen::DenseBase<Derived>& matrix, const std::string& filepath,
           const std::vector<std::string>& columnLabels = {})
{
    std::ofstream f(filepath);
    if (matrix.cols() == 0) {
        f.close();
        return;
    }
    
    // Column labels.
    if (!columnLabels.empty()) {
        if (columnLabels.size() != size_t(matrix.cols())) {
            throw std::runtime_error("Number of column labels ("
                + std::to_string(columnLabels.size()) + ") does not match "
                "the number of columns in the matrix ("
                + std::to_string(matrix.cols()) + ").");
        }

        f << columnLabels[0];
        for (size_t i_col = 1; i_col < columnLabels.size(); ++i_col) {
            f << "," << columnLabels[i_col];
        }
        f << std::endl;
    }
    
    // Data.
    // Separate coefficients with a comma.
    Eigen::IOFormat format(Eigen::StreamPrecision, 0, ",");
    // This function only exists on DenseBase (not on EigenBase).
    f << matrix.format(format) << std::endl;
}

/// Similar to the other write() function, but this one also includes a time
/// column. Additionally, the matrix should have time along its columns;
/// it will be transposed in the file so that time is along the rows. The
/// number of columns in matrix must match the number of rows in time.
template<typename Derived>
void write(const Eigen::VectorXd& time,
           const Eigen::DenseBase<Derived>& matrix,
           const std::string& filepath,
           const std::vector<std::string>& columnLabels = {}) {
    auto labels = columnLabels;
    if (!labels.empty()) {
        labels.insert(labels.begin(), "time");
    }

    Eigen::MatrixXd with_time(matrix.cols(), matrix.rows() + 1);
    with_time << time, matrix.transpose();

    write(with_time, filepath, labels);
}

} // namespace tropter

#endif // TROPTER_COMMON_H
