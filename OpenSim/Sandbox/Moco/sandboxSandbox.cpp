/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxSandbox.cpp                                           *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

// This file provides a way to easily prototype or test temporary snippets of
// code during development.

#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Common/MultivariatePolynomialFunction.h>

using namespace OpenSim;

class HornerSchemeNode {
public:
    virtual SimTK::Real calcValue(const SimTK::Vector& x) = 0;
};
class Factor : public HornerSchemeNode {
public:
    Factor(HornerSchemeNode* left, HornerSchemeNode* right, int index) : 
            m_left(left), m_right(right), m_index(index) {}

    SimTK::Real calcValue(const SimTK::Vector& x) override {
        return m_left->calcValue(x) + x[m_index] * m_right->calcValue(x);
    }
private:
    HornerSchemeNode* m_left;
    HornerSchemeNode* m_right;
    int m_index;
};

class Monomial : public HornerSchemeNode {
public:
    Monomial(SimTK::Real constant, const SimTK::Vector& coefficients) : 
            m_constant(constant), m_coefficients(coefficients), 
            m_numCoefficients(coefficients.size()) {}

    SimTK::Real calcValue(const SimTK::Vector& x) override {
        SimTK::Real result = m_constant;
        for (int i = 0; i < m_numCoefficients; ++i) {
            result += m_coefficients[i] * x[i];
        }
        return result;
    }
private:
    SimTK::Real m_constant;
    SimTK::Vector m_coefficients;
    int m_numCoefficients;
};

std::unique_ptr<HornerSchemeNode> createHornerScheme(
        const std::vector<std::string>& vars,
        const MultivariatePolynomial& poly) {

    int order = poly.calcOrder();
    if (order == 0) {
        return std::make_unique<Monomial>(poly.at({}), SimTK::Vector());
    } else if (order == 1) {
        int dimension = vars.size();
        SimTK::Vector coefficients = 
                poly.calcCoefficients(dimension, order, vars);
        
        // We need to reverse the coefficients of the monomial to match the
        // convention of the MultivariatePolynomialFunction.
        SimTK::Vector x_coefs(dimension);
        int index = 0;
        for (int i = dimension; i > 0; --i) {
            x_coefs[index] = coefficients[i];
            ++index;
        }
        
        return std::make_unique<Monomial>(coefficients[0], x_coefs);
    } else {
        // Find the variable in "vars" that appears in the most terms.
        int index = 0;
        int maxCount = 0;
        for (int i = 0; i < static_cast<int>(vars.size()); ++i) {
            int count = 0;
            for (const auto& term : poly) {
                if (term.first.find(vars[i]) != term.first.end()) {
                    ++count;
                }
            }
            if (count > maxCount) {
                maxCount = count;
                index = i;
            }
        }

        std::string var = vars[index];
        auto factors = poly.factorVariable(var);
        std::unique_ptr<HornerSchemeNode> left =
                createHornerScheme(vars, factors.first);
        std::unique_ptr<HornerSchemeNode> right =
                createHornerScheme(vars, factors.second);
        return std::make_unique<Factor>(left.release(), right.release(), index); 
    }
}

void testHornerScheme() {
    int order = 5;
    int dimension = 3;
    std::vector<std::string> vars = { "x1", "x2", "x3" };
    int numCoeffs = MultivariatePolynomial::choose(dimension + order, order);
    // SimTK::Vector coefficients = SimTK::Test::randVector(numCoeffs);
    SimTK::Vector coefficients(numCoeffs);
    for (int i = 0; i < numCoeffs; ++i) {
        coefficients[i] = i+1;
    }

    MultivariatePolynomialFunction mvpFunc(coefficients, dimension, order);

    MultivariatePolynomial mvp(mvpFunc, vars);
    std::unique_ptr<HornerSchemeNode> hornerScheme = 
            createHornerScheme(vars, mvp);

    std::cout << "poly = " << mvp << std::endl;
    auto factors = mvp.factorVariable("x1");

    // Test the Horner scheme.
    SimTK::Vector x = SimTK::Test::randVector(dimension);

    // Evaluate time elapsed for the Horner scheme.
    int numTrials = 1000000;
    SimTK::Real sum = 0;
    SimTK::Real startTime = SimTK::realTime();
    for (int i = 0; i < numTrials; ++i) {
        sum += hornerScheme->calcValue(x);
    }
    SimTK::Real endTime = SimTK::realTime();
    std::cout << "Time elapsed for Horner scheme: " << endTime - startTime << std::endl;

    // Evaluate time elapsed for the original function.
    sum = 0;
    startTime = SimTK::realTime();
    for (int i = 0; i < numTrials; ++i) {
        sum += mvpFunc.calcValue(x);
    }
    endTime = SimTK::realTime();
    std::cout << "Time elapsed for original function: " << endTime - startTime << std::endl;

    for (int i = 0; i < numTrials; ++i) {
        SimTK::Vector x = SimTK::Test::randVector(dimension);
        SimTK_TEST_EQ(hornerScheme->calcValue(x), mvpFunc.calcValue(x));
    }
}

void testHelperFunctions() {
    // 2-DOF polynomial path function.
    // length = 1 + 2*q_y + 3*q_y^2 + 4*q_x + 5*q_x*q_y + 6*q_x^2
    MultivariatePolynomialFunction lengthFunc(
            createVector({1.0, 2.0, 3.0, 4.0, 5.0, 6.0}), 2, 2);

    // Moment arm functions.
    // momentArm_x = -dl/dq_x = -4 - 5*q_y - 12*q_x
    // momentArm_y = -dl/dq_y = -2 - 6*q_y - 5*q_x
    bool negateCoefficients = true;
    MultivariatePolynomialFunction momentArmFunc_x = 
        lengthFunc.generateDerivativeFunction(0, negateCoefficients);
    MultivariatePolynomialFunction momentArmFunc_y =
        lengthFunc.generateDerivativeFunction(1, negateCoefficients);

    // Speed function.
    // speed = -qdot_x * momentArm_x - qdot_y * momentArm_y
    //       = qdot_x * (4 + 5*q_y + 12*q_x) + qdot_y * (2 + 5*q_x + 6*q_y)
    //       = 4*qdot_x + 5*qdot_x*q_y + 12*qdot_x*q_x + 2*qdot_y +
    //         5*qdot_y*q_x + 6*qdot_y*q_y
    // 
    // See the documentation for MultivariatePolynomialFunction for an
    // explanation of the coefficients.
    MultivariatePolynomialFunction speedFunc = 
            lengthFunc.generateChainRuleFunction();

    int dimension = 4;
    SimTK::Vector x = SimTK::Test::randVector(dimension);
    speedFunc.calcValue(x);

}

int main() {
    testHelperFunctions();
    return EXIT_SUCCESS;
}
