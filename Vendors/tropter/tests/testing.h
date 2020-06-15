#ifndef TROPTER_TESTING_H
#define TROPTER_TESTING_H
// ----------------------------------------------------------------------------
// tropter: testing.h
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

#include <catch.hpp>
#include <Eigen/Dense>

/// Compare any two Eigen matrices (their dimensions and each element)
/// using Catch, with a given relative error tolerance.
// Extra parentheses are to avoid a warning from GCC 5.4.

namespace tropter {

#define TROPTER_REQUIRE_EIGEN(actual, expected, rel_error_tolerance)         \
do {                                                                         \
    REQUIRE((actual.rows() == expected.rows()));                             \
    REQUIRE((actual.cols() == expected.cols()));                             \
    for (int ir = 0; ir < actual.rows(); ++ir) {                             \
        for (int ic = 0; ic < actual.cols(); ++ic) {                         \
            INFO("(" << ir << "," << ic << "): " <<                          \
                    actual(ir, ic) << " vs " << expected(ir, ic));           \
            REQUIRE((Approx(actual(ir, ic))                                  \
                    .epsilon(rel_error_tolerance).scale(1.0)                 \
                    == expected(ir, ic)));                                   \
        }                                                                    \
    }                                                                        \
} while (0)

/// Similar to TROPTER_REQUIRE_EIGEN, but using an absolute error tolerance.
#define TROPTER_REQUIRE_EIGEN_ABS(actual, expected, abs_error_tolerance)     \
do {                                                                         \
    REQUIRE((actual.rows() == expected.rows()));                             \
    REQUIRE((actual.cols() == expected.cols()));                             \
    for (int ir = 0; ir < actual.rows(); ++ir) {                             \
        for (int ic = 0; ic < actual.cols(); ++ic) {                         \
            INFO("(" << ir << "," << ic << "): " <<                          \
                    actual(ir, ic) << " vs " << expected(ir, ic));           \
            REQUIRE((Approx(actual(ir, ic)).margin(abs_error_tolerance)      \
                    == expected(ir, ic)));                                   \
        }                                                                    \
    }                                                                        \
} while (0)

template <typename TActual, typename TExpected>
void testAlmostEqual(const Eigen::EigenBase<TActual>& a,
        const Eigen::EigenBase<TExpected>& e,
        double rel_error_tolerance) {
    TROPTER_REQUIRE_EIGEN(a.derived(), e.derived(), rel_error_tolerance);
}

} // namespace tropter

#endif // TROPTER_TESTING_H
