#ifndef MESH_TESTING_H
#define MESH_TESTING_H

#include <catch.hpp>

/// TODO
#define REQUIRE_EIGEN(actual, expected, abs_error_tolerance)                 \
do {                                                                         \
    REQUIRE(actual.rows() == expected.rows());                               \
    REQUIRE(actual.cols() == expected.cols());                               \
    for (int ir = 0; ir < actual.rows(); ++ir) {                             \
        for (int ic = 0; ic < actual.cols(); ++ic) {                         \
            REQUIRE(Approx(actual(ir, ic)).epsilon(abs_error_tolerance)      \
                    == expected(ir, ic));                                    \
        }                                                                    \
    }                                                                        \
} while (0)

#endif // MESH_TESTING_H
