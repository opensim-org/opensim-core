#ifndef TROPTER_TESTING_H
#define TROPTER_TESTING_H

#include <catch.hpp>

/// Compare any two Eigen matrices (their dimensions and each element)
/// using Catch.
#define TROPTER_REQUIRE_EIGEN(actual, expected, abs_error_tolerance)         \
do {                                                                         \
    REQUIRE(actual.rows() == expected.rows());                               \
    REQUIRE(actual.cols() == expected.cols());                               \
    for (int ir = 0; ir < actual.rows(); ++ir) {                             \
        for (int ic = 0; ic < actual.cols(); ++ic) {                         \
            INFO("Checking row " << ir << " column " << ic << ".");          \
            REQUIRE(Approx(actual(ir, ic)).epsilon(abs_error_tolerance)      \
                    == expected(ir, ic));                                    \
        }                                                                    \
    }                                                                        \
} while (0)

#endif // TROPTER_TESTING_H
