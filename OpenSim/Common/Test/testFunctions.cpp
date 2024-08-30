/* -------------------------------------------------------------------------- *
 *                       OpenSim:  testFunctions.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "ComponentsForTesting.h"

#include <OpenSim/Common/CommonUtilities.h>
#include <OpenSim/Common/MultivariatePolynomialFunction.h>
#include <OpenSim/Common/Reporter.h>
#include <OpenSim/Common/SignalGenerator.h>
#include <OpenSim/Common/Sine.h>
#include <OpenSim/Common/ExpressionBasedFunction.h>

#include <catch2/catch_all.hpp>

#include <OpenSim/Common/PolynomialFunction.h>

using namespace OpenSim;
using namespace SimTK;
using Catch::Approx;
using Catch::Matchers::ContainsSubstring;


TEST_CASE("SignalGenerator") {
    // Feed a SignalGenerator's output into a TableReporter, and make sure the
    // reported value is correct.

    // Build the model.
    RootComponent world;
    auto* signalGen = new SignalGenerator();
    signalGen->setName("sinusoid");
    const double amplitude = 1.5;
    const double omega = 3.1;
    const double phase = 0.3;
    const double offset = 0.12345;
    signalGen->set_function(Sine(amplitude, omega, phase, offset));
    world.addComponent(signalGen);
    
    auto* reporter = new TableReporter();
    reporter->addToReport(signalGen->getOutput("signal"));
    world.addComponent(reporter);

    // Build the system.
    MultibodySystem system;
    world.buildUpSystem(system);
    State s = system.realizeTopology();
    system.realize(s, Stage::Model);

    // "Simulate."
    const int numTimePoints = 5;
    for (int i = 0; i < numTimePoints; ++i) {
        s.setTime(0.1 * i);
        system.realize(s, Stage::Report);
    }

    // Check that the SignalGenerator produced the correct values.
    const TimeSeriesTable_<Real>& results = reporter->getTable();
    REQUIRE(results.getNumRows() == numTimePoints);
    for (int i = 0; i < numTimePoints; ++i) {
        const double time = 0.1 * i;
        system.realize(s, Stage::Report);
        CHECK(results.getRowAtIndex(i)[0] ==
                Approx(amplitude * std::sin(omega * time + phase) + offset));
    }
}

TEST_CASE("Interpolate using PiecewiseLinearFunction") {
    SECTION("New X out of original range") {
        SimTK::Vector x = createVector({0, 1});
        SimTK::Vector y = createVector({1, 0});
        SimTK::Vector newX = createVector({-1, 0.25, 0.75, 1.5});
        SimTK::Vector newY = OpenSim::interpolate(x, y, newX);

        SimTK_TEST(SimTK::isNaN(newY[0]));
        SimTK_TEST_EQ(newY[1], 0.75);
        SimTK_TEST_EQ(newY[2], 0.25);
        SimTK_TEST(SimTK::isNaN(newY[3]));
    }
    SECTION("New X out of original range, extrapolate") {
        SimTK::Vector x = createVector({0, 1});
        SimTK::Vector y = createVector({1, 0});
        SimTK::Vector newX = createVector({-1, 0.25, 0.75, 1.5});
        SimTK::Vector newY = OpenSim::interpolate(x, y, newX, false, true);

        SimTK_TEST(!SimTK::isNaN(newY[0]));
        SimTK_TEST_EQ(newY[1], 0.75);
        SimTK_TEST_EQ(newY[2], 0.25);
        SimTK_TEST(!SimTK::isNaN(newY[3]));
    }
    SECTION("First and last Y are NaN, extrapolate") {
        SimTK::Vector x = createVector({0, 1, 2, 3, 4, 5});
        SimTK::Vector y = createVector({SimTK::NaN, 0, 3, 4, SimTK::NaN, SimTK::NaN});
        SimTK::Vector newY = OpenSim::interpolate(x, y, x, true, true);

        for (int i = 0; i < newY.size(); ++i) {
            SimTK_TEST(!SimTK::isNaN(newY[i]));
        }
    }

    SimTK::Vector x = createVector({0, 1, 2, 3});
    SimTK::Vector y = createVector({SimTK::NaN, 1, 2, SimTK::NaN});
    SECTION("Ignore NaNs, extrapolate") {
        SimTK::Vector newY = OpenSim::interpolate(x, y, x, true, true);
        SimTK_TEST_EQ(newY[0], 0);
        SimTK_TEST_EQ(newY[1], 1);
        SimTK_TEST_EQ(newY[2], 2);
        SimTK_TEST_EQ(newY[3], 3);
    }
    SECTION("Don't ignore NaNs, extrapolate") {
        SimTK::Vector newY = OpenSim::interpolate(x, y, x, false, true);
        SimTK_TEST(SimTK::isNaN(newY[0]));
        SimTK_TEST_EQ(newY[1], 1);
        SimTK_TEST(SimTK::isNaN(newY[2]));
        SimTK_TEST(SimTK::isNaN(newY[3]));
    }
    SECTION("Ignore NaNs, don't extrapolate") {
        SimTK::Vector newY = OpenSim::interpolate(x, y, x, true, false);
        SimTK_TEST(SimTK::isNaN(newY[0]));
        SimTK_TEST_EQ(newY[1], 1);
        SimTK_TEST_EQ(newY[2], 2);
        SimTK_TEST(SimTK::isNaN(newY[3]));
    }
}

TEST_CASE("MultivariatePolynomialFunction") {
    SECTION("Input errors") {
        {
            MultivariatePolynomialFunction f(createVector({1}), -1, 1);
            CHECK_THROWS_WITH(f.calcValue(SimTK::Vector()),
                    ContainsSubstring("Expected dimension"));
        }
        {
            MultivariatePolynomialFunction f(createVector({1}), 1, -1);
            CHECK_THROWS_WITH(f.calcValue(SimTK::Vector()),
                    ContainsSubstring("Expected order"));
        }
        {
            SimTK::Vector coefficients(19);
            MultivariatePolynomialFunction f(coefficients, 3, 3);
            CHECK_THROWS_WITH(f.calcValue(createVector({1, 2, 3})),
                    ContainsSubstring("Expected 20 coefficients but got 19"));
        }
        {
            SimTK::Vector coefficients(21);
            MultivariatePolynomialFunction f(coefficients, 3, 3);
            CHECK_THROWS_WITH(f.calcValue(createVector({1, 2, 3})),
                    ContainsSubstring("Expected 20 coefficients but got 21"));
        }
    }
    SECTION("Consistent with PolynomialFunction") {
        PolynomialFunction univariate(createVector({2, 1, 3}));
        MultivariatePolynomialFunction multivariate(
                createVector({3, 1, 2}), 1, 2);
        SimTK::Vector x = createVector({0.338});
        SimTK_TEST_EQ(univariate.calcValue(x), multivariate.calcValue(x));
        SimTK_TEST_EQ(univariate.calcDerivative({0}, x),
                multivariate.calcDerivative({0}, x));
    }
    SECTION("Test 3-dimensional 3rd order polynomial") {
        SimTK::Vector c = SimTK::Test::randVector(20);
        MultivariatePolynomialFunction f(c, 3, 3);
        SimTK::Vector input = createVector({0.3, 7.3, 0.8});
        const double x = input[0];
        const double y = input[1];
        const double z = input[2];
        double expected = c[0] + c[1]*z + c[2]*z*z + c[3] * z*z*z +
                          c[4]*y + c[5]*y*z + c[6]*y*z*z + c[7]*y*y +
                          c[8]*y*y*z + c[9]*y*y*y + c[10]*x + c[11]*x*z +
                          c[12]*x*z*z + c[13]*x*y + c[14]*x*y*z + c[15]*x*y*y +
                          c[16]*x*x + c[17]*x*x*z + c[18]*x*x*y + c[19]*x*x*x;
        SimTK_TEST_EQ(f.calcValue(input), expected);
        SimTK_TEST_EQ(f.calcDerivative({0}, input), 
                c[10] + c[11]*z + c[12]*z*z +
                c[13]*y + c[14]*y*z + c[15]*y*y + 2*c[16]*x + 2*c[17]*x*z +
                2*c[18]*x*y + 3*c[19]*x*x);
    }
    SECTION("Test 4-dimensional 1st order polynomial") {
        SimTK::Vector c = SimTK::Test::randVector(5);
        MultivariatePolynomialFunction f(c, 4, 1);
        SimTK::Vector input = createVector({0.3, 7.3, 0.8, 6.4});
        double expected = c[0] + c[1] * input[3] + c[2] * input[2] +
                          c[3] * input[1] + c[4] * input[0];
        SimTK_TEST_EQ(f.calcValue(input), expected);
        SimTK_TEST_EQ(f.calcDerivative({0}, input), c[4]);
    }
    SECTION("Test 6-dimensional 1st order polynomial") {
        SimTK::Vector c = SimTK::Test::randVector(7);
        MultivariatePolynomialFunction f(c, 6, 1);
        SimTK::Vector input = createVector({0.3, 7.3, 0.8, 6.4, 1.2, 3.4});
        double expected = c[0] + c[1] * input[5] + c[2] * input[4] +
                          c[3] * input[3] + c[4] * input[2] + c[5] * input[1] +
                          c[6] * input[0];
        SimTK_TEST_EQ(f.calcValue(input), expected);
        SimTK_TEST_EQ(f.calcDerivative({0}, input), c[6]);
    }
    SECTION("Test calcDerivativeCoefficients()") {
        // 2-DOF polynomial function.
        // f = 1 + 2*q_y + 3*q_y^2 + 4*q_x + 5*q_x*q_y + 6*q_x^2
        MultivariatePolynomialFunction f(
                createVector({1.0, 2.0, 3.0, 4.0, 5.0, 6.0}), 2, 2);

        // Partial derivatives.
        // f_x = dl/dq_x = 4 + 5*q_y + 12*q_x
        // f_y = dl/dq_y = 2 + 6*q_y + 5*q_x
        MultivariatePolynomialFunction f_x(
                createVector({4.0, 5.0, 12.0}), 2, 1);
        MultivariatePolynomialFunction f_y(
                createVector({2.0, 6.0, 5.0}), 2, 1);

        MultivariatePolynomialFunction f_x_test = 
                f.generateDerivativeFunction(0);
        MultivariatePolynomialFunction f_y_test = 
                f.generateDerivativeFunction(1);

        SimTK::Vector q = SimTK::Test::randVector(2);
        SimTK_TEST_EQ(f.calcDerivative({0}, q), f_x.calcValue(q));
        SimTK_TEST_EQ(f.calcDerivative({1}, q), f_y.calcValue(q));
        SimTK_TEST_EQ(f_x.calcValue(q), f_x_test.calcValue(q));
        SimTK_TEST_EQ(f_y.calcValue(q), f_y_test.calcValue(q));
    }
}

TEST_CASE("solveBisection()") {

    auto calcResidual = [](const SimTK::Real& x) { return x - 3.78; };
    {
        const auto root = solveBisection(calcResidual, -5, 5, 1e-6);
        SimTK_TEST_EQ_TOL(root, 3.78, 1e-6);
        // Make sure the tolerance has an effect.
        SimTK_TEST_NOTEQ_TOL(root, 3.78, 1e-10);
    }
    {
        const auto root = solveBisection(calcResidual, -5, 5, 1e-10);
        SimTK_TEST_EQ_TOL(root, 3.78, 1e-10);
    }

    // Multiple roots.
    {
        auto parabola = [](const SimTK::Real& x) {
            return SimTK::square(x - 2.5);
        };
        REQUIRE_THROWS_AS(solveBisection(parabola, -5, 5), OpenSim::Exception);
    }
}

TEST_CASE("ExpressionBasedFunction") {
    const SimTK::Real x = SimTK::Test::randReal();
    const SimTK::Real y = SimTK::Test::randReal();
    const SimTK::Real z = SimTK::Test::randReal();

    SECTION("Square-root function") {
        ExpressionBasedFunction f("sqrt(x)", {"x"});
        REQUIRE_THAT(f.calcValue(createVector({x})), 
                Catch::Matchers::WithinAbs(std::sqrt(x), 1e-10));
        REQUIRE_THAT(f.calcDerivative({0}, createVector({x})), 
                Catch::Matchers::WithinAbs(0.5 / std::sqrt(x), 1e-10));
    }

    SECTION("Exponential function") {
        ExpressionBasedFunction f("exp(x)", {"x"});
        REQUIRE_THAT(f.calcValue(createVector({x})), 
                Catch::Matchers::WithinAbs(std::exp(x), 1e-10));
        REQUIRE_THAT(f.calcDerivative({0}, createVector({x})), 
                Catch::Matchers::WithinAbs(std::exp(x), 1e-10));
    }

    SECTION("Multivariate function") {
        ExpressionBasedFunction f("2*x^3 + 3*y*z^2", {"x", "y", "z"});
        REQUIRE_THAT(f.calcValue(createVector({x, y, z})), 
                Catch::Matchers::WithinAbs(2*x*x*x + 3*y*z*z, 1e-10));
        REQUIRE_THAT(f.calcDerivative({0}, createVector({x, y, z})), 
                Catch::Matchers::WithinAbs(6*x*x, 1e-10));
        REQUIRE_THAT(f.calcDerivative({1}, createVector({x, y, z})), 
                Catch::Matchers::WithinAbs(3*z*z, 1e-10));
        REQUIRE_THAT(f.calcDerivative({2}, createVector({x, y, z})), 
                Catch::Matchers::WithinAbs(6*y*z, 1e-10));
    }


    SECTION("Sinusoidal function") {
        ExpressionBasedFunction f("x*sin(y*z^2)", {"x", "y", "z"});
        REQUIRE_THAT(f.calcValue(createVector({x, y, z})), 
                Catch::Matchers::WithinAbs(x * std::sin(y*z*z), 1e-10));
        REQUIRE_THAT(f.calcDerivative({0}, createVector({x, y, z})), 
                Catch::Matchers::WithinAbs(std::sin(y*z*z), 1e-10));
        REQUIRE_THAT(f.calcDerivative({1}, createVector({x, y, z})), 
                Catch::Matchers::WithinAbs(x*z*z*std::cos(y*z*z), 1e-10));  
    }

    SECTION("Undefined variable in expression") {
        ExpressionBasedFunction f("x*y", {"x"});
        REQUIRE_THROWS_WITH(f.calcValue(createVector({x, y})), 
                Catch::Matchers::ContainsSubstring(
                        "Variable 'y' is not defined."));
    }

    SECTION("Extra variables should have zero derivative") {
        ExpressionBasedFunction f("x*y", {"x", "y", "z"});
        REQUIRE_THAT(f.calcValue(createVector({x, y, z})), 
                Catch::Matchers::WithinAbs(x*y, 1e-10));
        REQUIRE_THAT(f.calcDerivative({0}, createVector({x, y, z})), 
                Catch::Matchers::WithinAbs(y, 1e-10));
        REQUIRE_THAT(f.calcDerivative({1}, createVector({x, y, z})), 
                Catch::Matchers::WithinAbs(x, 1e-10));
        REQUIRE_THAT(f.calcDerivative({2}, createVector({x, y, z})), 
                Catch::Matchers::WithinAbs(0.0, 1e-10));
    }

    SECTION("Derivative of nonexistent variable") {
        ExpressionBasedFunction f("x*y", {"x", "y"});
        REQUIRE_THAT(f.calcDerivative({2}, createVector({x, y})),
                Catch::Matchers::WithinAbs(0.0, 1e-10));
    }

    SECTION("Variable defined multiple times") {
        ExpressionBasedFunction f("x", {"x", "x"});
        REQUIRE_THROWS_WITH(f.calcValue(createVector({x})), 
                Catch::Matchers::ContainsSubstring(
                        "Variable 'x' is defined more than once."));
    }

    SECTION("Non-alphabetic variable names") {
        ExpressionBasedFunction f("@^2 + %*cos(&)", {"@", "%", "&"});
        REQUIRE_THAT(f.calcValue(createVector({x, y, z})), 
                Catch::Matchers::WithinAbs(x*x + y*std::cos(z), 1e-10));
        REQUIRE_THAT(f.calcDerivative({0}, createVector({x, y, z})), 
                Catch::Matchers::WithinAbs(2*x, 1e-10));
        REQUIRE_THAT(f.calcDerivative({1}, createVector({x, y, z})), 
                Catch::Matchers::WithinAbs(std::cos(z), 1e-10));
        REQUIRE_THAT(f.calcDerivative({2}, createVector({x, y, z})), 
                Catch::Matchers::WithinAbs(-y*std::sin(z), 1e-10));
    }
}