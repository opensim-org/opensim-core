/* -------------------------------------------------------------------------- *
 * OpenSim: MultivariatePolynomialFunction.cpp                                *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017-19 Stanford University and the Authors                  *
 *                                                                            *
 * Author(s): Antoine Falisse                                                 *
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

#include "MultivariatePolynomialFunction.h"

#include "Exception.h"

using namespace OpenSim;

template <class T>
class SimTKMultivariatePolynomial : public SimTK::Function_<T> {
public:
    SimTKMultivariatePolynomial(const SimTK::Vector_<T>& coefficients,
            const int& dimension, const int& order)
            : m_coefficients(coefficients), m_dimension(dimension),
              m_order(order) {
        OPENSIM_THROW_IF(dimension < 0 || dimension > 6, Exception,
                "Expected dimension >= 0 && <=6 but got {}.", dimension);
        OPENSIM_THROW_IF(order < 0, Exception,
                "Expected order >= 0 but got {}.", order);

        SimTK::Vector tmp;
        int numCoefficients = calcPolynomialHelper({},
                SimTK::Vector(dimension, 1), tmp, false);

        OPENSIM_THROW_IF(coefficients.size() != numCoefficients, Exception,
                "Expected {} coefficients but got {}.", numCoefficients,
                coefficients.size());
    }

    SimTK::Vector_<T> calcTermValues(const SimTK::Vector& x) const {
        std::array<int, 6> nq{{0, 0, 0, 0, 0, 0}};
        SimTK::Vector_<T> values(m_coefficients.size(), T(0.0));
        calcPolynomialHelper({}, x, values);
        return values;
    }
    
    SimTK::Vector_<T> calcTermDerivatives(
            const SimTK::Array_<int>& derivComponent,
            const SimTK::Vector& x) const {
        SimTK::Vector_<T> derivatives(m_coefficients.size(), T(0.0));
        calcPolynomialHelper(derivComponent, x, derivatives);
        return derivatives;
    }

    // A helper function for computing polynomial term values, term derivatives,
    // and the total number of coefficients. If 'derivComponent' is empty, then
    // the function computes the polynomial term values. Otherwise, it computes
    // the polynomial term derivatives with respect to the component specified
    // by 'derivComponent'. The function returns the total number of
    // coefficients. If 'calcInfo' is false, then the function only returns the
    // total number of coefficients.
    int calcPolynomialHelper(const SimTK::Array_<int>& derivComponent,
            const SimTK::Vector& x, SimTK::Vector_<T>& info,
            bool calcInfo = true) const {
        std::array<int, 6> nq{{0, 0, 0, 0, 0, 0}};
        int nqNonNegative;
        int numCoefficients = 0;

        // Cycle through all possible combinations of powers for the polynomial
        // terms.
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
                                if (!derivComponent.empty() && calcInfo) {
                                    // Calculate the derivative terms.
                                    if (derivComponent[0] == 0) {
                                        nqNonNegative = nq[0] - 1;
                                        if (nqNonNegative < 0)
                                            nqNonNegative = 0;
                                        T derivP =
                                                nq[0] *
                                                std::pow(x[0], nqNonNegative);
                                        for (int i = 0; i < m_dimension; ++i) {
                                            if (i == derivComponent[0])
                                                continue;
                                            derivP *= std::pow(x[i], nq[i]);
                                        }
                                        info[numCoefficients] = derivP;
                                    } else if (derivComponent[0] == 1) {
                                        nqNonNegative = nq[1] - 1;
                                        if (nqNonNegative < 0)
                                            nqNonNegative = 0;
                                        T derivP =
                                                nq[1] *
                                                std::pow(x[1], nqNonNegative);
                                        for (int i = 0; i < m_dimension; ++i) {
                                            if (i == derivComponent[0])
                                                continue;
                                            derivP *= std::pow(x[i], nq[i]);
                                        }
                                        info[numCoefficients] = derivP;
                                    } else if (derivComponent[0] == 2) {
                                        nqNonNegative = nq[2] - 1;
                                        if (nqNonNegative < 0)
                                            nqNonNegative = 0;
                                        T derivP =
                                                nq[2] *
                                                std::pow(x[2], nqNonNegative);
                                        for (int i = 0; i < m_dimension; ++i) {
                                            if (i == derivComponent[0])
                                                continue;
                                            derivP *= std::pow(x[i], nq[i]);
                                        }
                                        info[numCoefficients] = derivP;
                                    } else if (derivComponent[0] == 3) {
                                        nqNonNegative = nq[3] - 1;
                                        if (nqNonNegative < 0)
                                            nqNonNegative = 0;
                                        T derivP =
                                                nq[3] *
                                                std::pow(x[3], nqNonNegative);
                                        for (int i = 0; i < m_dimension; ++i) {
                                            if (i == derivComponent[0])
                                                continue;
                                            derivP *= std::pow(x[i], nq[i]);
                                        }
                                        info[numCoefficients] = derivP;
                                    } else if (derivComponent[0] == 4) {
                                        nqNonNegative = nq[4] - 1;
                                        if (nqNonNegative < 0)
                                            nqNonNegative = 0;
                                        T derivP =
                                                nq[4] *
                                                std::pow(x[4], nqNonNegative);
                                        for (int i = 0; i < m_dimension; ++i) {
                                            if (i == derivComponent[0])
                                                continue;
                                            derivP *= std::pow(x[i], nq[i]);
                                        }
                                        info[numCoefficients] = derivP;
                                    } else if (derivComponent[0] == 5) {
                                        nqNonNegative = nq[5] - 1;
                                        if (nqNonNegative < 0)
                                            nqNonNegative = 0;
                                        T derivP =
                                                nq[5] *
                                                std::pow(x[5], nqNonNegative);
                                        for (int i = 0; i < m_dimension; ++i) {
                                            if (i == derivComponent[0])
                                                continue;
                                            derivP *= std::pow(x[i], nq[i]);
                                        }
                                        info[numCoefficients] = derivP;
                                    }
                                } else if (calcInfo) {
                                    // Calculate the value terms.
                                    T valueP = static_cast<T>(1);
                                    for (int i = 0; i < m_dimension; ++i) {
                                        valueP *= std::pow(x[i], nq[i]);
                                    }
                                    info[numCoefficients] = valueP;
                                }

                                // Increment the number of coefficients.
                                ++numCoefficients;
                            }
                        }
                    }
                }
            }
        }
        return numCoefficients;
    }
    
    T calcValue(const SimTK::Vector& x) const override {
        SimTK::Vector_<T> values = calcTermValues(x);
        return ~m_coefficients * values;
    }
    
    T calcDerivative(const SimTK::Array_<int>& derivComponent,
            const SimTK::Vector& x) const override {
        SimTK::Vector_<T> derivatives = calcTermDerivatives(derivComponent, x);
        return ~m_coefficients * derivatives;
    }
    
    int getArgumentSize() const override { return m_dimension; }
    int getMaxDerivativeOrder() const override { return 1; }
    SimTKMultivariatePolynomial* clone() const override {
        return new SimTKMultivariatePolynomial(*this);
    }

private:
    SimTK::Vector_<T> m_coefficients;
    int m_dimension;
    int m_order;
};

SimTK::Function* MultivariatePolynomialFunction::createSimTKFunction() const {
    return new SimTKMultivariatePolynomial<SimTK::Real>(
            get_coefficients(), get_dimension(), getOrder());
}

SimTK::Vector MultivariatePolynomialFunction::getTermValues(
        const SimTK::Vector& x) const {
    if (!_function) {
        _function = createSimTKFunction();
    }
    return dynamic_cast<const SimTKMultivariatePolynomial<SimTK::Real>*>(
                    _function)->calcTermValues(x);
}

SimTK::Vector MultivariatePolynomialFunction::getTermDerivatives(
        const std::vector<int>& derivComponent, const SimTK::Vector& x) const {
    if (!_function) {
        _function = createSimTKFunction();
    }
    return dynamic_cast<const SimTKMultivariatePolynomial<SimTK::Real>*>(
                    _function)->calcTermDerivatives(
                        SimTK::ArrayViewConst_<int>(derivComponent), x);
}

class PolynomialDerivativeHelper {

public:
    SimTK::Vector calcDerivativeCoefficients(
            const MultivariatePolynomialFunction& mvpoly, int derivComponent) {

        std::string var = "q" + std::to_string(derivComponent);
                
        SimTK::Vector coefficients = mvpoly.getCoefficients();
        int order = mvpoly.getOrder();
        int dimension = mvpoly.getDimension();

        Polynomial poly = constructPolynomial(order, dimension);
        // Set the coefficients of the polynomial to the coefficients of the
        // MultivariatePolynomialFunction.
        for (auto it = poly.begin(); it != poly.end(); it++) {
            it->second = coefficients[it->second];
        }

        Polynomial deriv_q0 = calcDerivative(poly, var);
        Polynomial deriv_q0_temp = constructPolynomial(order - 1, dimension);

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

private:
    typedef std::map<std::string, int> Term;
    typedef std::map<Term, double> Polynomial;

    int choose(int n, int k) {
        if (k == 0) { return 1; }
        return (n * choose(n - 1, k - 1)) / k;
    }

    Polynomial constructPolynomial(int m_order, int m_dimension) {
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
                                        term["q" + std::to_string(i)] = nq[i];
                                    }
                                }

                                // Add the term to the polynomial.
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

    

};

SimTK::Vector MultivariatePolynomialFunction::calcDerivativeCoefficients(
        const MultivariatePolynomialFunction& mvpoly, int derivComponent) {
    PolynomialDerivativeHelper helper;
    return helper.calcDerivativeCoefficients(mvpoly, derivComponent);
}
