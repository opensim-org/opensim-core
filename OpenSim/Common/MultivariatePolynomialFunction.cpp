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

/** 
 * A helper class to construct and manipulate multivariate polynomials using 
 * symbolic operations.
 * 
 * A multivariate polynomial is represented as a map of monomials to 
 * coefficients. Monomials are represented as maps of variable names to 
 * exponents. For example, the polynomial
 * 
 * f = 0.1 x^2 y + 0.7 y^3 z + 0.5 x z^5
 * 
 * can be constructed represented as
 *     
 * @code
 * MultivariatePolynomial f; 
 * Monomial m0 = {{"x", 2}, {"y", 1}};
 * Monomial m1 = {{"y", 3}, {"z", 1}};
 * Monomial m2 = {{"x", 1}, {"z", 5}};
 * f[m0] = 0.1;
 * f[m1] = 0.7;
 * f[m2] = 0.5;
 * @endcode
 */ 
using Monomial = std::map<std::string, int>;
class MultivariatePolynomial : public std::map<Monomial, double> {

public:
    MultivariatePolynomial() = default;

    /**
     * Construct a multivariate polynomial of a given dimension and order with 
     * variables named according to the provided vector of strings, `var`. All 
     * possible monomials of the polynomial are generated  in the same order as 
     * `MultivariatePolynomialFunction` and assigned a coefficient corresponding
     * to the index of the monomial in the polynomial.
     */
    MultivariatePolynomial(int dimension, int order,
            const std::vector<std::string>& vars) {

        OPENSIM_THROW_IF(static_cast<int>(vars.size()) != dimension, 
                Exception,
                "Expected the number of variable names ({}) to match the "
                "polynomial dimension ({}), but it does not.", 
                vars.size(), dimension)

        int numCoefficients = choose(dimension + order, order);
        std::vector<Monomial> monomials;
        monomials.resize(numCoefficients);

        std::vector<std::vector<int>> combinations;
        generateCombinations(dimension, order, combinations);
        for (int i = 0; i < numCoefficients; ++i) {
            Monomial monomial;
            for (int j = 0; j < dimension; ++j) {
                if (combinations[i][j] > 0) {
                    monomial[vars[j]] = combinations[i][j];
                }
            }
            monomials.at(i) = monomial;
        }
        
        for (int i = 0; i < numCoefficients; ++i) {
            (*this)[monomials[i]] = i;
        }
    }

    /**
     * Construct a multivariate polynomial from a 
     * MultivariatePolynomialFunction and a vector of variable names. 
     * The polynomial is constructed by assigning the coefficients of the 
     * function to the monomials of the polynomial. The number of variable names
     * must match the dimension of the MultivariatePolynomialFunction.
     */
    MultivariatePolynomial(
            const MultivariatePolynomialFunction& mvp, 
            const std::vector<std::string>& vars) {

        int dimension = mvp.getDimension();
        int order = mvp.getOrder();
        SimTK::Vector coefficients = mvp.getCoefficients();

        OPENSIM_THROW_IF(
                coefficients.size() != choose(dimension + order, order), 
                Exception,
                "Expected the number of coefficients ({}) to match the number "
                "of coefficients in a polynomial of dimension {} and order {}, "
                "but it does not.", coefficients.size(), dimension, order);

        std::vector<Monomial> monomials;
        monomials.resize(coefficients.size());

        OPENSIM_THROW_IF(static_cast<int>(vars.size()) != dimension, 
                Exception,
                "Expected the number of variable names ({}) to match the "
                "polynomial dimension ({}), but it does not.", 
                vars.size(), dimension)

        std::vector<std::vector<int>> combinations;
        generateCombinations(dimension, order, combinations);
        for (int i = 0; i < coefficients.size(); ++i) {
            Monomial monomial;
            for (int j = 0; j < dimension; ++j) {
                if (combinations[i][j] > 0) {
                    monomial[vars[j]] = combinations[i][j];
                }
            }
            monomials.at(i) = monomial;
        }
        
        for (int i = 0; i < coefficients.size(); ++i) {
            (*this)[monomials[i]] = coefficients[i];
        }
    }

    // GET METHODS
    /**
     * Get the coefficient of a monomial in the polynomial. If the monomial is
     * not in the polynomial, the coefficient is zero.
     */
    double getCoefficient(const Monomial& monomial) const {
        auto it = find(monomial);
        if (it != end()) {
            return it->second;
        }
        return 0.0;
    }

    /**
     * Get the exponent of a variable in a monomial. If the variable is not in
     * the monomial, the exponent is zero.
     */
    int getExponent(const Monomial& monomial, const std::string& var) const {
        auto it = monomial.find(var);
        if (it != monomial.end()) {
            return it->second;
        }
        return 0;
    }

    // CALC METHODS
    /**
     * Calculate the order of the polynomial.
     */
    int calcOrder() const {
        int order = 0;
        // Iterate through all monomials.
        for (auto it = begin(); it != end(); it++) {
            // For this monomial, calculate the sum of the exponents of all
            // variables.
            const Monomial& monomial = it->first;
            int sum = 0;
            for (auto itv = monomial.begin(); itv != monomial.end(); itv++) {
                const std::string& var = itv->first;
                sum += getExponent(monomial, var);
            }
            // Update the order if the sum is greater than the current order.
            if (sum > order) {
                order = sum;
            }
        }
        return order;
    }

    /**
     * Calculate the vector of coefficients of the polynomial if it had a given 
     * order and dimension. This vector matches the coefficients vector of 
     * MultivariatePolynomialFunction; therefore it will contain coefficients 
     * for all possible monomials in the same order as a 
     * MultivariatePolynomialFunction. If the MultivariatePolynomial does not 
     * contain a monomial for a corresponding slot in the coefficients vector, 
     * coefficient is set to zero.
     */
    SimTK::Vector calcCoefficients(int dimension, int order, 
            const std::vector<std::string>& vars) const {

        OPENSIM_THROW_IF(static_cast<int>(vars.size()) != dimension, 
                Exception,
                "Expected the number of variable names ({}) to match the "
                "polynomial dimension ({}), but it does not.", 
                vars.size(), dimension)
        
        const MultivariatePolynomial& poly = *this;
        // Create a temporary polynomial with the same dimension and order as
        // the desired polynomial.
        MultivariatePolynomial temp(dimension, order, vars);
        // Initialize the coefficients to zero.
        int numCoefficients = choose(dimension + order, order);
        SimTK::Vector coefficients(numCoefficients, 0.0);
        // If the current polynomial contains a monomial that matches a monomial
        // in the temporary polynomial, set its coefficient in the full 
        // coefficients vector.
        for (auto it2 = temp.begin(); it2 != temp.end(); ++it2) {
            const Monomial& monomial = it2->first;
            int index = static_cast<int>(it2->second);
            coefficients[index] = getCoefficient(monomial);
        }

        return coefficients;
    }

    // POLYNOMIAL OPERATIONS
    /**
     * Get a new polynomial that is the first derivative of the current 
     * polynomial with respect to the specified variable. 
     */
    MultivariatePolynomial getDerivative(const std::string& var) const {
        const MultivariatePolynomial& poly = *this;
        MultivariatePolynomial deriv;
        // Iterate through all monomials in the polynomial.
        for (auto itp = poly.begin(); itp != poly.end(); itp++) {
            Monomial monomial = itp->first;
            double coeff = getCoefficient(monomial);
            // Iterate through all variables in the monomial.
            for (auto itm = monomial.begin(); itm != monomial.end(); itm++) {
                Monomial derivMonomial = monomial;
                // If the variable is in the monomial, take the derivative.
                if (itm->first == var) {
                    // If the exponent is 1, remove the variable from the
                    // monomial. Otherwise, reduce the exponent by 1.
                    int exponent = getExponent(monomial, var);
                    derivMonomial.erase(itm->first);
                    if (exponent > 1) {
                        derivMonomial[itm->first] = exponent - 1;
                    }
                    // Add the derivative term to the derivative polynomial.
                    // The coefficient is the product of the original 
                    // coefficient and previous exponent of the monomial.
                    deriv[derivMonomial] = coeff * exponent;
                }
            }
        }
        return deriv;
    }

    /**
     * Get a new polynomial that is the product of the current polynomial and 
     * a specified variable. The `exponent` argument specifies the power to
     * which the variable is raised.
     */
    MultivariatePolynomial multiplyByVariable(const std::string& var, 
            int exponent) const {
        MultivariatePolynomial result;
        const MultivariatePolynomial& poly = *this;
        for (auto it = poly.begin(); it != poly.end(); it++) {
            Monomial monomial = it->first;
            double coeff = getCoefficient(monomial);
            monomial[var] += exponent;
            result[monomial] = coeff;
        }
        return result;
    }

    /**
     * Given a variable, factor the current polynomial into two polynomials: one 
     * that contains all the terms did not contain the variable, and one that
     * contains all the terms that did contain the variable with the variable 
     * factored out. The former polynomial is returned as the first element of
     * the pair, and the latter polynomial is returned as the second element.
     *
     * The variable is factored out to only the first order. For example, if the
     * variable is X, the second polynomial X^3 + X^2 Y would be factored as 
     * X (X^2 + X Y) (the first polynomial here is 0).
     */
    std::pair<MultivariatePolynomial, MultivariatePolynomial> 
    factorVariable(const std::string& var) const {
        // All the terms that do not contain the variable.
        MultivariatePolynomial left;
        // All the terms that contain the variable.
        MultivariatePolynomial right;

        const MultivariatePolynomial& poly = *this;
        for (auto it = poly.begin(); it != poly.end(); it++) {
            Monomial monomial = it->first;
            // If the variable is not in the monomial, add the term to the left
            // polynomial. Otherwise, factor out the variable and add the term
            // to the right polynomial.
            double coeff = getCoefficient(monomial);
            if (monomial.find(var) == monomial.end()) {
                left[monomial] = coeff;
            } else {
                // Factor out the variable.
                monomial[var] -= 1;
                // Remove the variable if the exponent is zero.
                if (monomial[var] == 0) {
                    monomial.erase(var);
                }
                // Add the term to the right polynomial.
                right[monomial] = coeff;
            }
        }
        return std::make_pair(left, right);
    }

    /**
     * Return the sum of two MultivariatePolynomials.
     */
    static MultivariatePolynomial sum(
            const std::vector<MultivariatePolynomial>& polys) {
        MultivariatePolynomial sum;
        // Iterate through all polynomials.
        for (const auto& poly : polys) {
            // Iterate through all monomials in the polynomial.
            for (auto it = poly.begin(); it != poly.end(); it++) {
                // Add the coefficient of the monomial to the "sum" polynomial.
                const Monomial& monomial = it->first;
                sum[it->first] += poly.getCoefficient(monomial);
            }
        }
        return sum;
    }

    // HELPER METHODS
    /**
     * Generate all possible combinations of `k` elements from a set of `n`
     * total elements.
     */
    static int choose(int n, int k) {
        if (k == 0) { return 1; }
        return (n * choose(n - 1, k - 1)) / k;
    }

    /**
     * Generate all possible combinations of powers representing the terms of a
     * multivariate polynomial. Each combination is represented as a vector of
     * integers, where the `i`-th element is the power of the `i`-th variable.
     * For example, the combination (2, 1, 0) represents the term X^2 Y Z.
     */
    static void generateCombinations(int dimension, int order, 
            std::vector<std::vector<int>>& combinations) {
        combinations.clear();
        generateCombinations(std::vector<int>(), 0, 0, 
                dimension, order, combinations);
    }

private:
    static void generateCombinations(std::vector<int> current, int level, 
            int sum, int dimension, int order, 
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
};

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

        MultivariatePolynomial::generateCombinations(m_dimension, m_order, 
                m_combinations);

        // Construct a Horner's scheme for the polynomial.
        MultivariatePolynomial poly;
        std::vector<std::string> vars;
        for (int i = 0; i < dimension; ++i) {
            vars.push_back("x" + std::to_string(i));
        }
        for (int i = 0; i < numCoefs; ++i) {
            std::map<std::string, int> monomial;
            for (int j = 0; j < dimension; ++j) {
                if (m_combinations[i][j] > 0) {
                    monomial[vars[j]] = m_combinations[i][j];
                }
            }
            poly[monomial] = m_coefficients[i];
        }
        m_value = createHornerScheme(vars, poly);    

        // Construct a Horner's scheme for each polynomial derivatives.
        for (int i = 0; i < dimension; ++i) {
            std::string var = "x" + std::to_string(i);
            MultivariatePolynomial deriv = poly.getDerivative(var);
            m_derivatives.push_back(createHornerScheme(vars, deriv));
        }

    }

    T calcValue(const SimTK::Vector& x) const {
        return m_value->calcValue(x);
    }

    T calcDerivative(const SimTK::Array_<int>& derivComponent, 
            const SimTK::Vector& x) const {
        return m_derivatives[derivComponent[0]]->calcValue(x);
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
    SimTK::Vector_<T> m_coefficients;
    int m_dimension;
    int m_order;
    std::vector<std::vector<int>> m_combinations;

    // HORNER'S SCHEME

    // A base class representing a node for a binary tree representation of 
    // Horner's scheme for a multivariate polynomial.
    class HornerSchemeNode {
        public:
            virtual ~HornerSchemeNode() = default;
            virtual T calcValue(const SimTK::Vector_<T>& x) const = 0;

            // SimTK::ClonePtr requires a clone method.
            virtual HornerSchemeNode* clone() const = 0;
    };

    // A node representing a polynomial factorization. The value of the node is
    // the sum of the left node and the product of the right node and the
    // variable at the specified index.
    class Factor : public HornerSchemeNode {
        public:
            Factor(SimTK::ClonePtr<HornerSchemeNode> left, 
                    SimTK::ClonePtr<HornerSchemeNode> right, int index) : 
                    m_left(std::move(left)), m_right(std::move(right)), 
                    m_index(index) {}

            T calcValue(const SimTK::Vector_<T>& x) const override {
                return m_left->calcValue(x) + x[m_index] * m_right->calcValue(x);
            }

            HornerSchemeNode* clone() const override {
                return new Factor(
                    SimTK::ClonePtr<HornerSchemeNode>(m_left->clone()), 
                    SimTK::ClonePtr<HornerSchemeNode>(m_right->clone()), 
                    m_index);
            }
        private:
            SimTK::ClonePtr<HornerSchemeNode> m_left;
            SimTK::ClonePtr<HornerSchemeNode> m_right;
            int m_index;
    };

    // A node representing a monomial. The value of the node is the constant
    // term plus the sum of the product of the polynomial coefficients and the 
    // variables.
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

            HornerSchemeNode* clone() const override {
                return new Monomial(m_constant, m_coefficients);
            }
        private:
            T m_constant;
            SimTK::Vector_<T> m_coefficients;
            int m_numCoefficients;
    };

    // Construct Horner's scheme for a multivariate polynomial. Uses the 
    // "greedy" algorithm described in Ceberio and Kreinovich (2004), "Greedy 
    // Algorithms for Optimizing Multivariate Horner Schemes". The algorithm is
    // represented by a binary tree where each successive node is constructed 
    // by factoring out the variable that appears in the most terms of the 
    // current polynomial.
    SimTK::ClonePtr<HornerSchemeNode> createHornerScheme(
            const std::vector<std::string>& vars,
            const MultivariatePolynomial& poly) {

        int order = poly.calcOrder();
        if (order == 0) {
            // For a zeroth order polynomial, construct a Monomial node with a 
            // constant term and no coefficients.
            Monomial* monomial = new Monomial(
                    static_cast<T>(poly.at({})), SimTK::Vector_<T>());
            return SimTK::ClonePtr<Monomial>(monomial);

        } else if (order == 1) {
            // For a first order polynomial, construct a Monomial node with a
            // constant term and coefficients equal to the polynomial 
            // coefficients.
            int dimension = vars.size();
            SimTK::Vector_<T> coefficients = 
                    poly.calcCoefficients(dimension, order, vars);

            // Reverse the coefficients of the monomial to match the
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
            // For a polynomial of order greater than 1, factor out the variable
            // that appears in the most terms of the polynomial. Construct two 
            // Factor nodes: the "left" node represents the polynomial terms 
            // that do not contain the variable, and the "right" node represents
            // the polynomial terms that did contain the variable with it
            // factored out to the first order.
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

            // Factor the polynomial. This returns a pair of polynomials: the
            // "left" polynomial and the "right" polynomial described above.
            auto factors = poly.factorVariable(vars[index]);

            // Recursively continue constructing the binary tree.
            SimTK::ClonePtr<HornerSchemeNode> left =
                    createHornerScheme(vars, factors.first);
            SimTK::ClonePtr<HornerSchemeNode> right =
                    createHornerScheme(vars, factors.second);

            // Construct a Factor node.
            Factor* factor = new Factor(left, right, index);
            return SimTK::ClonePtr<Factor>(factor); 
        }
    }
    
    SimTK::ClonePtr<HornerSchemeNode> m_value;
    SimTK::Array_<SimTK::ClonePtr<HornerSchemeNode>> m_derivatives;
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
MultivariatePolynomialFunction::generateDerivativeFunction(
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
MultivariatePolynomialFunction::generatePartialVelocityFunction() const {

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

    // Construct the terms of the "partial velocity" polynomial.
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
    // variable names to get the correct dimension of the partial velocity 
    // polynomial.
    for (int i = 0; i < dimension; ++i) {
        vars.push_back("xdot" + std::to_string(i));
    }
    SimTK::Vector partialVelocityCoeffs = polysSum.calcCoefficients(
            2*dimension, order, vars);

    MultivariatePolynomialFunction partialVelocityPoly;
    partialVelocityPoly.setDimension(2*dimension);
    partialVelocityPoly.setOrder(order);
    partialVelocityPoly.setCoefficients(partialVelocityCoeffs);
 
    return partialVelocityPoly;
}
