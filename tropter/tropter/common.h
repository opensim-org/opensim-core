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

} // namespace tropter

#endif // TROPTER_COMMON_H
