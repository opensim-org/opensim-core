
#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#include <Eigen/Dense>
#include <adolc/adolc.h>

using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Dynamic;

typedef Matrix<adouble, Dynamic, 1> VectorXa;
TEST_CASE("Eigen can use adouble scalar type in reverse mode.", "[autodiff]") {

    // TODO
    SECTION("Trying without Eigen first.") {
        short int tag = 0;
        trace_on(tag);
        double px = 1.5;
        adouble x;
        double py;
        x <<= px;
        adouble y = x * x;
        y >>= py;
        trace_off();
        CAPTURE(py);
        double** J = myalloc(1, 1);
        int success = jacobian(tag, 1, 1, &px, J);
        REQUIRE(success == 3);
        REQUIRE(J[0][0] == 3.0);
    }

    SECTION("Now with Eigen.") {
        // TODO
        short int tag = 0;
        trace_on(tag);
        VectorXd px(1); px[0] = 1.5;
        VectorXa x(1);
        VectorXd py(1);
        x[0] <<= px[0];
        VectorXa y = x * x;
        y[0] >>= py[0];
        trace_off();
        CAPTURE(py);
        double** J = myalloc(1, 1);
        int success = jacobian(tag, 1, 1, &px[0], J);
        REQUIRE(success == 3);
        REQUIRE(J[0][0] == 3.0);
        //VectorXd v(2);
        //v[0] = 1;
        //v[1] = 5;
        //INFO(v);
    }
}