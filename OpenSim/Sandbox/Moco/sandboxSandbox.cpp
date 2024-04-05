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


typedef std::map<std::string, int> Term;
typedef std::map<Term, double> Polynomial;

int choose(int n, int k) {
    if (k == 0) { return 1; }
    return (n * choose(n - 1, k - 1)) / k;
}

Polynomial constructPolynomial(int m_order, int m_dimension,
        const std::vector<std::string>& varNames) {
    OPENSIM_ASSERT(m_dimension == varNames.size());
    std::array<int, 6> nq{{0, 0, 0, 0, 0, 0}};
    int nqNonNegative;
    int numCoefficients = choose(m_dimension + m_order, m_order);
    std::vector<Term> terms;
    terms.resize(numCoefficients);

    // Cycle through all possible combinations of powers for the polynomial
    // terms.
    int coeffIndex = 0;
    for (nq[0] = 0; nq[0] < m_order + 1; ++nq[0]) {
        int nq2_s;
        if (m_dimension < 2)
            nq2_s = 0;
        else
            nq2_s = m_order - nq[0];
        for (nq[1] = 0; nq[1] < nq2_s + 1; ++nq[1]) {
            int nq3_s;
            if (m_dimension < 3)
                nq3_s = 0;
            else
                nq3_s = m_order - nq[0] - nq[1];
            for (nq[2] = 0; nq[2] < nq3_s + 1; ++nq[2]) {
                int nq4_s;
                if (m_dimension < 4)
                    nq4_s = 0;
                else
                    nq4_s = m_order - nq[0] - nq[1] - nq[2];
                for (nq[3] = 0; nq[3] < nq4_s + 1; ++nq[3]) {
                    int nq5_s;
                    if (m_dimension < 5)
                        nq5_s = 0;
                    else
                        nq5_s = m_order - nq[0] - nq[1] - nq[2] - nq[3];
                    for (nq[4] = 0; nq[4] < nq5_s + 1; ++nq[4]) {
                        int nq6_s;
                        if (m_dimension < 6)
                            nq6_s = 0;
                        else
                            nq6_s = m_order - nq[0] - nq[1] - nq[2] -
                                    nq[3] - nq[4];
                        for (nq[5] = 0; nq[5] < nq6_s + 1; ++nq[5]) {

                            // Construct the polynomial term.
                            Term term;
                            for (int i = 0; i < m_dimension; ++i) {
                                if (nq[i] > 0) {
                                    term[varNames[i]]= nq[i];
                                }
                            }

                            // Add the term to the polynomial. Use a zero value
                            // for now.
                            terms.at(coeffIndex) = term;

                            // Increment the number of coefficients.
                            ++coeffIndex;
                        }
                    }
                }
            }
        }
    }

    Polynomial poly;
    for (int i = 0; i < numCoefficients; ++i) {
        poly[terms[i]] = i;
    }

    return poly;
}

void printPolynomial(const Polynomial& poly) {
    for (auto it = poly.begin(); it != poly.end(); it++) {
        std::cout << it->second;
        for (auto it2 = it->first.begin(); it2 != it->first.end(); it2++) {
            std::cout << it2->first << "^" << it2->second;
        }
        if (std::next(it) != poly.end())
            std::cout << " + ";
    }
    std::cout << std::endl;
}

Polynomial calcDerivative(const Polynomial& poly, const std::string& var) {
    Polynomial deriv;
    for (auto it = poly.begin(); it != poly.end(); it++) {
        Term t = it->first;
        double c = it->second;
        for (auto it2 = t.begin(); it2 != t.end(); it2++) {
            Term t2 = t;
            if (it2->first == var) {
                t2.erase(it2->first);
                if (it2->second > 1) {
                    t2[it2->first] = it2->second - 1;
                }
                deriv[t2] = c * it2->second;
            }
        }
    }
    return deriv;
}

Polynomial multiplyPolynomialByVar(const Polynomial& poly, 
        const std::string& var) {
    Polynomial multPoly;
    // Multiply each term by the variable. Modify each term in place.
    for (auto it = poly.begin(); it != poly.end(); it++) {
        Term t = it->first;
        double c = it->second;
        t[var] = 1;
        multPoly[t] = c;
    }
    return multPoly;
}

Polynomial sumPolynomials(const std::vector<Polynomial>& polys) {
    Polynomial sum;
    for (const auto& poly : polys) {
        for (const auto& term : poly) {
            sum[term.first] += term.second;
        }
    }
    return sum;
}

SimTK::Vector getDerivativeCoefficients(
        const MultivariatePolynomialFunction& mvpoly,
        int derivComponent) {

    std::string var = "q" + std::to_string(derivComponent);
            
    SimTK::Vector coefficients = mvpoly.getCoefficients();
    int order = mvpoly.getOrder();
    int dimension = mvpoly.getDimension();

    std::vector<std::string> varNames;
    for (int i = 0; i < dimension; ++i) {
        varNames.push_back("q" + std::to_string(i));
    }
    Polynomial poly = constructPolynomial(order, dimension, varNames);
    // Set the coefficients of the polynomial to the coefficients of the
    // MultivariatePolynomialFunction.
    for (auto it = poly.begin(); it != poly.end(); it++) {
        it->second = coefficients[it->second];
    }
    std::cout << "poly = ";
    printPolynomial(poly);

    Polynomial deriv_q0 = calcDerivative(poly, var);
    std::cout << "deriv_" + var + " = ";
    printPolynomial(deriv_q0);

    Polynomial deriv_q0_temp = constructPolynomial(order - 1, dimension, varNames);
    std::cout << "deriv_" + var + "_temp = ";
    printPolynomial(deriv_q0_temp);

    // Use the order of terms in deriv_q0_temp to construct a new vector of 
    // coefficients for the derivative function.
    int numDerivCoeffs = choose(dimension + order - 1, order - 1);
    SimTK::Vector derivCoefficients(numDerivCoeffs, 0.0);
    for (auto it2 = deriv_q0_temp.begin(); it2 != deriv_q0_temp.end(); ++it2) {
        auto it = deriv_q0.find(it2->first);
        if (it != deriv_q0.end()) {
            derivCoefficients[it2->second] = it->second;
        }
    }

    return derivCoefficients;
}

SimTK::Vector getCompositeFunctionCoefficients(
        const MultivariatePolynomialFunction& mvpoly) {

    SimTK::Vector coefficients = mvpoly.getCoefficients();
    int order = mvpoly.getOrder();
    int dimension = mvpoly.getDimension();

    std::vector<std::string> varNames;
    for (int i = 0; i < dimension; ++i) {
        varNames.push_back("q" + std::to_string(i));
    }
    Polynomial poly = constructPolynomial(order, dimension, varNames);
    // Set the coefficients of the polynomial to the coefficients of the
    // MultivariatePolynomialFunction.
    for (auto it = poly.begin(); it != poly.end(); it++) {
        it->second = coefficients[it->second];
    }
    std::cout << "poly = ";
    printPolynomial(poly);

    std::vector<Polynomial> composite_polys;
    for (int i = 0; i < dimension; ++i) {
        Polynomial f_qi = calcDerivative(poly, "q" + std::to_string(i));
        Polynomial f_qi_qdot = 
            multiplyPolynomialByVar(f_qi, "qdot" + std::to_string(i));
        composite_polys.push_back(f_qi_qdot);
    }

    Polynomial composite = sumPolynomials(composite_polys);
    std::cout << "composite = ";
    printPolynomial(composite);

    std::vector<std::string> compositeVarNames;
    for (int i = 0; i < dimension; ++i) {
        compositeVarNames.push_back("q" + std::to_string(i));
    }
    for (int i = 0; i < dimension; ++i) {
        compositeVarNames.push_back("qdot" + std::to_string(i));
    }

    Polynomial composite_temp = constructPolynomial(order, 2*dimension, compositeVarNames);
    std::cout << "composite_temp = ";
    printPolynomial(composite_temp);

    // Use the order of terms in composite_temp to construct a new vector of
    // coefficients for the composite function.
    int numCompositeCoeffs = choose(2*dimension + order, order);
    SimTK::Vector compositeCoefficients(numCompositeCoeffs, 0.0);
    for (auto it2 = composite_temp.begin(); it2 != composite_temp.end(); ++it2) {
        auto it = composite.find(it2->first);
        if (it != composite.end()) {
            compositeCoefficients[it2->second] = it->second;
        }
    }

    return compositeCoefficients;
}

void testPolynomial() {
    // f(x,y,z) = 1 + 2x + 3y + 4z + 5xy + 6xz + 7yz + 8x^2 + 9y^2 + 10z^2
    // df/dx = 2 + 5y + 6z + 16x
    // df/dy = 3 + 5x + 7z + 18y
    // df/dz = 4 + 6x + 7y + 20z

    Polynomial f = 
    {
        { { }, 1 },
        { { { "x", 1 } }, 2 },
        { { { "y", 1 } }, 3 },
        { { { "z", 1 } }, 4 },
        { { { "x", 1 }, { "y", 1 } }, 5 },
        { { { "x", 1 }, { "z", 1 } }, 6 },
        { { { "y", 1 }, { "z", 1 } }, 7 },
        { { { "x", 2 } }, 8 },
        { { { "y", 2 } }, 9 },
        { { { "z", 2 } }, 10 }
    };

    std::cout << "f(x,y,z) = ";
    printPolynomial(f);

    Polynomial dfdx = calcDerivative(f, "x");
    Polynomial dfdy = calcDerivative(f, "y");
    Polynomial dfdz = calcDerivative(f, "z");

    std::cout << "df/dx = ";
    printPolynomial(dfdx);

    std::cout << "df/dy = ";
    printPolynomial(dfdy);

    std::cout << "df/dz = ";
    printPolynomial(dfdz);
}

void testFunctionBasedPathWorkflow() {

    // Test workflow
    // -------------
    // 1. Start with a MultivariatePolynomialFunction with known coefficients.
    // 2. Need a new polynomial that is a derivative of the original polynomial
    //    with respect to one of the independent components.
    // 3. Start by constructing a polynomial with the same order and dimension
    //    as the original polynomial.

    int order = 3;
    int dimension = 2;
    int numCoeffs = choose(dimension + order, order);


    SimTK::Vector coefficients(numCoeffs);
    for (int i = 0; i < numCoeffs; ++i) {
        coefficients[i] = i;
    }
    std::cout << "coefficients = " << coefficients << std::endl;
    std::cout << std::endl;

    
    MultivariatePolynomialFunction mvpoly(coefficients, dimension, order);
    
    SimTK::Vector derivCoeffs0 = getDerivativeCoefficients(mvpoly, 0);
    std::cout << "derivCoefficients, q0 = " << derivCoeffs0 << std::endl;
    std::cout << std::endl;

    SimTK::Vector derivCoeffs1 = getDerivativeCoefficients(mvpoly, 1);
    std::cout << "derivCoefficients, q1 = " << derivCoeffs1 << std::endl;
    std::cout << std::endl;

    // SimTK::Vector derivCoeffs2 = getDerivativeCoefficients(mvpoly, 2);
    // std::cout << "derivCoefficients, q2 = " << derivCoeffs2 << std::endl;

    // TODO not yet possible for all paths, since we don't support multivariate 
    // polynomials of dimension > 6.
    SimTK::Vector compositeCoefs = getCompositeFunctionCoefficients(mvpoly);
    std::cout << "compositeCoefs = " << compositeCoefs << std::endl;

}

void testCreateDerivativeFunction() {
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

    SimTK::Vector f_x_coefs = f.calcDerivativeCoefficients(f, 0);
    SimTK::Vector f_y_coefs = f.calcDerivativeCoefficients(f, 1);

    std::cout << "f_x_coefs: " << f_x_coefs << std::endl;
    std::cout << "f_y_coefs: " << f_y_coefs << std::endl;

}

void testCompositeFunctionCreation() {

    Polynomial f = 
    {
        { { }, 1 },
        { { { "x", 1 } }, 2 },
        { { { "y", 1 } }, 3 },
        { { { "x", 1 }, { "y", 1 } }, 5 },
        { { { "x", 2 } }, 8 },
        { { { "y", 2 } }, 9 },
    };

    std::cout << "f(x,y) = ";
    printPolynomial(f);

    Polynomial f_xdot = multiplyPolynomialByVar(f, "xdot");
    std::cout << "f(xdot,x,y) = ";
    printPolynomial(f_xdot);

    Polynomial f_ydot = multiplyPolynomialByVar(f, "ydot");
    std::cout << "f(ydot,x,y) = ";
    printPolynomial(f_ydot);

    Polynomial sum = sumPolynomials({f_xdot, f_ydot});
    std::cout << "f(xdot,ydot,x,y) = ";
    printPolynomial(sum);

}



int main() {
    testFunctionBasedPathWorkflow();
    // testCreateDerivativeFunction();
    // testCompositeFunctionCreation();
    return EXIT_SUCCESS;
}
