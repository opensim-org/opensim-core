#ifndef TROPTER_COMMON_H
#define TROPTER_COMMON_H
// ----------------------------------------------------------------------------
// tropter: common.h
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

#include <tropter/optional-lite/optional.hpp>

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

// optional-lite
// -------------
template <typename T>
using Optional = nonstd::optional_lite::optional<T>;

// ADOL-C
// ------
// TODO move elsewhere.
inline bool isnan(const adouble& v) { return std::isnan(v.value()); }

// Eigen
// -----

using VectorXa = Eigen::Matrix<adouble, Eigen::Dynamic, 1>;
using MatrixXa = Eigen::Matrix<adouble, Eigen::Dynamic, Eigen::Dynamic>;

template<typename T>
using VectorX = Eigen::Matrix<T, Eigen::Dynamic, 1>;
template<typename T>
using RowVectorX = Eigen::Matrix<T, 1, Eigen::Dynamic>;
template<typename T>
using MatrixX = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

template<typename T>
using Vector2 = Eigen::Matrix<T, 2, 1>;
template<typename T>
using Matrix2 = Eigen::Matrix<T, 2, 2>;

} // namespace tropter

#endif // TROPTER_COMMON_H
