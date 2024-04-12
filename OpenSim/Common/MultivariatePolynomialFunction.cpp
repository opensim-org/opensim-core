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

//=============================================================================
//                        MULTIVARIATE POLYNOMIAL
//=============================================================================
MultivariatePolynomial::MultivariatePolynomial(
        int dimension, int order, const std::vector<std::string>& vars) {

    int numCoefficients = choose(dimension + order, order);
    std::vector<Term> terms;
    terms.resize(numCoefficients);

    OPENSIM_THROW_IF(static_cast<int>(vars.size()) != dimension, 
            Exception,
            "Expected the number of variable names ({}) to match the "
            "polynomial dimension ({}), but it does not.", 
            vars.size(), dimension)

    std::vector<std::vector<int>> combinations;
    generateCombinations(
            std::vector<int>(), 0, 0, dimension, order, combinations);
    for (int i = 0; i < numCoefficients; ++i) {
        Term term;
        for (int j = 0; j < dimension; ++j) {
            if (combinations[i][j] > 0) {
                term[vars[j]] = combinations[i][j];
            }
        }
        terms.at(i) = term;
    }
    
    for (int i = 0; i < numCoefficients; ++i) {
        (*this)[terms[i]] = i;
    }
}

MultivariatePolynomial::MultivariatePolynomial(
        const MultivariatePolynomialFunction& mvp, 
        const std::vector<std::string>& vars) {

    int dimension = mvp.getDimension();
    int order = mvp.getOrder();
    SimTK::Vector coefficients = mvp.getCoefficients();

    OPENSIM_THROW_IF(
            coefficients.size() != choose(dimension + order, order), 
            Exception,
            "Expected the number of coefficients ({}) to match the "
            "number of coefficients in a polynomial of dimension {} and order "
            "{}, but it does not.", 
            coefficients.size(), dimension, order);

    std::vector<Term> terms;
    terms.resize(coefficients.size());

    OPENSIM_THROW_IF(static_cast<int>(vars.size()) != dimension, 
            Exception,
            "Expected the number of variable names ({}) to match the "
            "polynomial dimension ({}), but it does not.", 
            vars.size(), dimension)

    std::vector<std::vector<int>> combinations;
    generateCombinations(
            std::vector<int>(), 0, 0, dimension, order, combinations);
    for (int i = 0; i < coefficients.size(); ++i) {
        Term term;
        for (int j = 0; j < dimension; ++j) {
            if (combinations[i][j] > 0) {
                term[vars[j]] = combinations[i][j];
            }
        }
        terms.at(i) = term;
    }
    
    for (int i = 0; i < coefficients.size(); ++i) {
        (*this)[terms[i]] = coefficients[i];
    }
}

//=============================================================================
//                               CALC METHODS
//=============================================================================
int MultivariatePolynomial::calcOrder() const {
    int order = 0;
    for (auto it = begin(); it != end(); it++) {
        int sum = 0;
        for (auto it2 = it->first.begin(); it2 != it->first.end(); it2++) {
            sum += it2->second;
        }
        if (sum > order) {
            order = sum;
        }
    }
    return order;
}

SimTK::Vector MultivariatePolynomial::calcCoefficients(int dimension, int order, 
        const std::vector<std::string>& vars) const {

    OPENSIM_THROW_IF(static_cast<int>(vars.size()) != dimension, 
            Exception,
            "Expected the number of variable names ({}) to match the "
            "polynomial dimension ({}), but it does not.", 
            vars.size(), dimension)
    
    const MultivariatePolynomial& poly = *this;
    MultivariatePolynomial temp(dimension, order, vars);
    int numCoefficients = choose(dimension + order, order);
    SimTK::Vector coefficients(numCoefficients, 0.0);
    for (auto it2 = temp.begin(); it2 != temp.end(); ++it2) {
        auto it = poly.find(it2->first);
        if (it != poly.end()) {
            coefficients[it2->second] = it->second;
        }
    }

    return coefficients;
}

//=============================================================================
//                        POLYNOMIAL OPERATIONS
//=============================================================================
MultivariatePolynomial MultivariatePolynomial::getDerivative(
        const std::string& var) const {
    const MultivariatePolynomial& poly = *this;
    MultivariatePolynomial deriv;
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

MultivariatePolynomial MultivariatePolynomial::multiplyByVariable(
            const std::string& var, int exponent) const {
    MultivariatePolynomial result;
    const MultivariatePolynomial& poly = *this;
    for (auto it = poly.begin(); it != poly.end(); it++) {
        Term term = it->first;
        if (term.find(var) == term.end()) {
            term[var] = exponent;
        } else {
            term[var] += exponent;
        }
        result[term] = it->second;
    }
    return result;
}

std::pair<MultivariatePolynomial, MultivariatePolynomial> 
MultivariatePolynomial::factorVariable(const std::string& var) const {
    // All the terms that do not contain the variable.
    MultivariatePolynomial left;
    // All the terms that contain the variable.
    MultivariatePolynomial right;

    const MultivariatePolynomial& poly = *this;
    for (auto it = poly.begin(); it != poly.end(); it++) {
        Term term = it->first;
        if (term.find(var) == term.end()) {
            left[term] = it->second;
        } else {
            term[var] -= 1;
            if (term[var] == 0) {
                term.erase(var);
            }
            right[term] = it->second;
        }
    }
    return std::make_pair(left, right);
}

MultivariatePolynomial 
MultivariatePolynomial::sum(const std::vector<MultivariatePolynomial>& polys) {
    MultivariatePolynomial sum;
    for (const auto& poly : polys) {
        for (auto it = poly.begin(); it != poly.end(); it++) {
            sum[it->first] += it->second;
        }
    }
    return sum;
}

//=============================================================================
//                              HELPER METHODS
//=============================================================================
int MultivariatePolynomial::choose(int n, int k) {
    if (k == 0) { return 1; }
    return (n * choose(n - 1, k - 1)) / k;
}

void MultivariatePolynomial::generateCombinations(std::vector<int> current, 
        int level, int sum, int dimension, int order, 
        std::vector<std::vector<int>>& combinations) {
    if (level == dimension) {
        if (sum <= order) {
            combinations.push_back(current);
        }
        return;
    }
    for (int i = 0; i <= order - sum; ++i) {
        current.push_back(i);
        generateCombinations(current, level + 1, sum + i, 
                dimension, order, combinations);
        current.pop_back();
    }
}

std::ostream& operator<<(std::ostream& os, const MultivariatePolynomial& poly) {
    for (auto it = poly.begin(); it != poly.end(); it++) {
        os << it->second;
        for (auto it2 = it->first.begin(); it2 != it->first.end(); it2++) {
            os << it2->first << "^" << it2->second;
        }
        if (std::next(it) != poly.end())
            os << " + ";
    }
    return os;
}

//=============================================================================
//                     MULTIVARIATE POLYNOMIAL FUNCTION
//=============================================================================
template <class T>
class SimTKMultivariatePolynomial : public SimTK::Function_<T> {
public:
    SimTKMultivariatePolynomial(const SimTK::Vector_<T>& coefficients,
            const int& dimension, const int& order)
            : m_coefficients(coefficients), m_dimension(dimension),
              m_order(order) {
        OPENSIM_THROW_IF(dimension < 0, Exception,
                "Expected dimension >= 0 but got {}.", dimension);
        OPENSIM_THROW_IF(order < 0, Exception,
                "Expected order >= 0 but got {}.", order);

        int numCoefs = MultivariatePolynomial::choose(dimension + order, order);
        OPENSIM_THROW_IF(coefficients.size() != numCoefs, Exception,
                "Expected {} coefficients but got {}.", 
                numCoefs, coefficients.size());

        precomputeCombinations();

        // Construct a Horner's scheme for the polynomial.
        MultivariatePolynomial poly;
        std::vector<std::string> vars;
        for (int i = 0; i < dimension; ++i) {
            vars.push_back("x" + std::to_string(i));
        }
        for (int i = 0; i < numCoefs; ++i) {
            Term term;
            for (int j = 0; j < dimension; ++j) {
                if (m_combinations[i][j] > 0) {
                    term[vars[j]] = m_combinations[i][j];
                }
            }
            poly[term] = m_coefficients[i];
        }
        m_value = createHornerScheme(vars, poly);    

        // Construct Horner's schemes for the polynomial derivatives.
        for (int i = 0; i < dimension; ++i) {
            std::string var = "x" + std::to_string(i);
            MultivariatePolynomial deriv = poly.getDerivative(var);
            m_derivatives.push_back(createHornerScheme(vars, deriv));
        }

    }

    // T calcValue(const SimTK::Vector& x) const {
    //     T result = static_cast<T>(0);
    //     for (int i = 0; i < m_coefficients.size(); ++i) {
    //         T term = m_coefficients[i];
    //         for (int j = 0; j < m_dimension; ++j) {
    //             term *= std::pow(x[j], m_combinations[i][j]);
    //         }
    //         result += term;
    //     }
        
    //     return result;
    // }

    T calcValue(const SimTK::Vector& x) const {
        return m_value->calcValue(x);
    }

    SimTK::Vector_<T> calcTermValues(const SimTK::Vector& x) const {
        SimTK::Vector_<T> values(m_coefficients.size(), T(0.0));
        for (int i = 0; i < m_coefficients.size(); ++i) {
            T term = static_cast<T>(1);
            for (int j = 0; j < m_dimension; ++j) {
                term *= std::pow(x[j], m_combinations[i][j]);
            }
            values[i] = term;
        }
        
        return values;
    }

    // T calcDerivative(const SimTK::Array_<int>& derivComponent, 
    //         const SimTK::Vector& x) const {
    //     T result = static_cast<T>(0);
    //     for (int i = 0; i < m_coefficients.size(); ++i) {
    //         if (m_combinations[i][derivComponent[0]] > 0) {
    //             T term = m_coefficients[i];
    //             for (int j = 0; j < m_dimension; ++j) {
    //                 if (j == derivComponent[0]) {
    //                     term *= m_combinations[i][j];
    //                     term *= std::pow(x[j], m_combinations[i][j] - 1);
    //                 } else {
    //                     term *= std::pow(x[j], m_combinations[i][j]);
    //                 }
    //             }
    //             result += term;
    //         }
    //     }
    //     return result;
    // }

    T calcDerivative(const SimTK::Array_<int>& derivComponent, 
            const SimTK::Vector& x) const {
        return m_derivatives[derivComponent[0]]->calcValue(x);
    }

    SimTK::Vector_<T> calcTermDerivatives(
            const SimTK::Array_<int>& derivComponent,
            const SimTK::Vector& x) const {
        SimTK::Vector_<T> derivatives(m_coefficients.size(), T(0.0));
        for (int i = 0; i < m_coefficients.size(); ++i) {
            if (m_combinations[i][derivComponent[0]] > 0) {
                T term = static_cast<T>(1);
                for (int j = 0; j < m_dimension; ++j) {
                    if (j == derivComponent[0]) {
                        term *= m_combinations[i][j];
                        term *= std::pow(x[j], m_combinations[i][j] - 1);
                    } else {
                        term *= std::pow(x[j], m_combinations[i][j]);
                    }
                }
                derivatives[i] = term;
            }
        }
        return derivatives;
    }
    
    int getArgumentSize() const override { return m_dimension; }
    int getMaxDerivativeOrder() const override { return 1; }
    SimTKMultivariatePolynomial* clone() const override {
        return new SimTKMultivariatePolynomial(*this);
    }

private:
    void precomputeCombinations() {
        m_combinations.clear();
        MultivariatePolynomial::generateCombinations(
            std::vector<int>(), 0, 0, m_dimension, m_order, m_combinations);
    }

    SimTK::Vector_<T> m_coefficients;
    int m_dimension;
    int m_order;
    std::vector<std::vector<int>> m_combinations;

    class HornerSchemeNode {
        public:
            virtual T calcValue(const SimTK::Vector_<T>& x) const = 0;
            HornerSchemeNode* clone() const { return nullptr; }
    };

    class Factor : public HornerSchemeNode {
        public:
            Factor(HornerSchemeNode* left, HornerSchemeNode* right, int index) : 
                    m_left(left), m_right(right), m_index(index) {}

            T calcValue(const SimTK::Vector_<T>& x) const override {
                return m_left->calcValue(x) + x[m_index] * m_right->calcValue(x);
            }
        private:
            HornerSchemeNode* m_left;
            HornerSchemeNode* m_right;
            int m_index;
    };

    class Monomial : public HornerSchemeNode {
        public:
            Monomial(T constant, const SimTK::Vector_<T>& coefficients) : 
                    m_constant(constant), m_coefficients(coefficients), 
                    m_numCoefficients(coefficients.size()) {}

            T calcValue(const SimTK::Vector_<T>& x) const override {
                T result = m_constant;
                for (int i = 0; i < m_numCoefficients; ++i) {
                    result += m_coefficients[i] * x[i];
                }
                return result;
            }
        private:
            T m_constant;
            SimTK::Vector_<T> m_coefficients;
            int m_numCoefficients;
    };

    SimTK::ClonePtr<HornerSchemeNode> createHornerScheme(
            const std::vector<std::string>& vars,
            const MultivariatePolynomial& poly) {

        int order = poly.calcOrder();
        if (order == 0) {
            Monomial* monomial = new Monomial(
                    static_cast<T>(poly.at({})), SimTK::Vector_<T>());
            return SimTK::ClonePtr<Monomial>(monomial);

        } else if (order == 1) {
            int dimension = vars.size();
            SimTK::Vector_<T> coefficients = 
                    poly.calcCoefficients(dimension, order, vars);

            // We need to reverse the coefficients of the monomial to match the
            // convention of the MultivariatePolynomialFunction.
            SimTK::Vector_<T> x_coefs(dimension);
            int index = 0;
            for (int i = dimension; i > 0; --i) {
                x_coefs[index] = coefficients[i];
                ++index;
            }
            
            Monomial* monomial = new Monomial(
                    static_cast<T>(coefficients[0]), x_coefs);
            return SimTK::ClonePtr<Monomial>(monomial);

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
            SimTK::ClonePtr<HornerSchemeNode> left =
                    createHornerScheme(vars, factors.first);
            SimTK::ClonePtr<HornerSchemeNode> right =
                    createHornerScheme(vars, factors.second);

            Factor* factor = new Factor(left.release(), right.release(), index);
            return SimTK::ClonePtr<Factor>(factor); 
        }
    }
    
    SimTK::ClonePtr<HornerSchemeNode> m_value;
    std::vector<SimTK::ClonePtr<HornerSchemeNode>> m_derivatives;
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

MultivariatePolynomialFunction 
MultivariatePolynomialFunction::generateFunctionFirstDerivative(
        int derivComponent, bool negateCoefficients) const {
    
    SimTK::Vector coefficients = getCoefficients();
    int order = getOrder();
    int dimension = getDimension();
    std::vector<std::string> vars;
    for (int i = 0; i < dimension; ++i) {
        vars.push_back("x" + std::to_string(i));
    }
    std::string var = "x" + std::to_string(derivComponent);

    // Construct a symbolic representation of the MultivariatePolynomialFunction
    // using MultivariatePolynomial and take the derivative with respect to the
    // specified variable.
    MultivariatePolynomial poly(*this, vars);
    MultivariatePolynomial deriv = poly.getDerivative(var);

    // Get the coefficients of the derivative polynomial.
    SimTK::Vector derivCoeffs = deriv.calcCoefficients(
            dimension, order - 1, vars);
    if (negateCoefficients) {
        derivCoeffs.negateInPlace();
    }

    return MultivariatePolynomialFunction(derivCoeffs, dimension, order - 1);
}

MultivariatePolynomialFunction 
MultivariatePolynomialFunction::generateFunctionChainRule() const {

    SimTK::Vector coefficients = getCoefficients();
    int order = getOrder();
    int dimension = getDimension();
    std::vector<std::string> vars;
    for (int i = 0; i < dimension; ++i) {
        vars.push_back("x" + std::to_string(i));
    }

    // Construct a symbolic representation of the MultivariatePolynomialFunction
    // using MultivariatePolynomial.
    MultivariatePolynomial mvp(*this, vars);

    // Construct the terms of the "chain rule polynomial".
    std::vector<MultivariatePolynomial> polys;
    for (int i = 0; i < dimension; ++i) {
        std::string x = "x" + std::to_string(i);
        std::string xdot = "xdot" + std::to_string(i);
        MultivariatePolynomial deriv = mvp.getDerivative(x);
        MultivariatePolynomial poly = deriv.multiplyByVariable(xdot, 1);
        polys.push_back(poly);
    }

    // Sum the terms.
    MultivariatePolynomial polysSum = MultivariatePolynomial::sum(polys);

    // Get the coefficients. We need to append "xdot0", "xdot1", etc. to the
    // variable names to get the correct dimension of the chain rule polynomial.
    for (int i = 0; i < dimension; ++i) {
        vars.push_back("xdot" + std::to_string(i));
    }
    SimTK::Vector chainRuleCoeffs = polysSum.calcCoefficients(
            2*dimension, order, vars);

    MultivariatePolynomialFunction compositePoly;
    compositePoly.setDimension(2*dimension);
    compositePoly.setOrder(order);
    compositePoly.setCoefficients(chainRuleCoeffs);
 
    return compositePoly;
}
