#ifndef OPENSIM_MULTIVARIATEPOLYNOMIAL_FUNCTION_H_
#define OPENSIM_MULTIVARIATEPOLYNOMIAL_FUNCTION_H_
/* -------------------------------------------------------------------------- *
 * OpenSim: MultivariatePolynomialFunction.h                                  *
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

#include "osimCommonDLL.h"
#include "Function.h"

namespace OpenSim {

/** A multivariate polynomial function.
 *
 * This implementation allows computation of first-order derivatives only.
 *
 * For a third-order polynomial that is a function of three components 
 * (X, Y, Z), the order is a follows:
 *
 * <pre>
 * Index | X  Y  Z
 * 0     | 0  0  0
 * 1     | 0  0  1
 * 2     | 0  0  2
 * 3     | 0  0  3
 * 4     | 0  1  0
 * 5     | 0  1  1
 * 6     | 0  1  2
 * 7     | 0  2  0
 * 8     | 0  2  1
 * 9     | 0  3  0
 * 10    | 1  0  0
 * 11    | 1  0  1
 * 12    | 1  0  2
 * 13    | 1  1  0
 * 14    | 1  1  1
 * 15    | 1  2  0
 * 16    | 2  0  0
 * 17    | 2  0  1
 * 18    | 2  1  0
 * 19    | 3  0  0
 * </pre>
 * Assuming c6 the index 6 coefficient, the corresponding term is Y Z^2.
 *
 * @param dimension the number of independent components
 * @param order the polynomial order (the largest sum of exponents in a single
 *        term)
 * @param coefficients the polynomial coefficients in order of ascending
 *        powers starting from the last independent component.
 *
 * @note The order of coefficients for this class is the *opposite** from the
 *       order used in the univariate PolynomialFunction.
 */
class OSIMCOMMON_API MultivariatePolynomialFunction : public Function {
    OpenSim_DECLARE_CONCRETE_OBJECT(MultivariatePolynomialFunction, Function);

public:
    OpenSim_DECLARE_PROPERTY(coefficients, SimTK::Vector,
            "Coefficients of a multivariate polynomial function in order of "
            "ascending powers starting from the last independent component.");
    OpenSim_DECLARE_PROPERTY(dimension, int,
            "Number of input dimensions (i.e., independent components).");
    OpenSim_DECLARE_PROPERTY(
            order, int, "The largest sum of exponents in a single term.");

    MultivariatePolynomialFunction() { constructProperties(); }

    MultivariatePolynomialFunction(
            SimTK::Vector coefficients, int dimension, int order) {
        constructProperties();
        set_coefficients(std::move(coefficients));
        set_dimension(dimension);
        set_order(order);
    }

    /**
     * The vector of coefficients for the multivariate polynomial.
     */
    void setCoefficients(SimTK::Vector coefficients) {
        set_coefficients(std::move(coefficients));
    }
    /// @copydoc setCoefficients()
    const SimTK::Vector& getCoefficients() const { return get_coefficients(); }

    /**
     * The number of independent variables in the multivariate polynomial.
     */
    void setDimension(int dimension) { set_dimension(dimension); }
    /// @copydoc setDimension()
    int getDimension() const { return get_dimension(); }

    /**
     * The order (i.e., the largest sum of exponents in a single term) of the
     * multivariate polynomial.
     */
    void setOrder(int order) { set_order(order); }
    /// @copydoc setOrder()
    int getOrder() const { return get_order(); }

    /**
     * Return a pointer to a SimTK::Function object that implements this
     * function.
     */
    SimTK::Function* createSimTKFunction() const override;
    
    /**
     * Get a vector of the terms in the polynomial function.
     */
    SimTK::Vector getTermValues(const SimTK::Vector& x) const;

    /**
     * Get a vector of the derivatives of the terms in the polynomial function.
     */
    SimTK::Vector getTermDerivatives(const std::vector<int>& derivComponent,
            const SimTK::Vector& x) const;

    /**
     * Generate a new MultivariatePolynomialFunction representing the first
     * derivative of the current function with respect to the specified
     * component. Therefore, the resulting function with have the same dimension
     * as the original function, but the order will be reduced by one.
     *
     * The coefficients of the derivative function can be negated using the 
     * `negateCoefficients` argument. This may be useful, for example, if the 
     * current function represents the length of a muscle path and the 
     * derivative is needed for computing muscle moment arms.
     *
     * @param derivComponent The component with respect to which the derivative
     *        is taken.
     * @param negateCoefficients If true, the coefficients of the derivative
     *        function will be negated.
     */
    MultivariatePolynomialFunction generateFunctionFirstDerivative(
            int derivComponent, bool negateCoefficients = false) const;

    /**
     * Generate a new MultivariatePolynomialFunction representing the dot
     * product of the function first derivatives with a new vector of variables.
     * This is useful for generating a function that represents the derivative
     * of the current function with respect to a different independent variable
     * (e.g., time).
     * 
     * For example, if the current function has the arguments 
     * \f$ x = x_0, x_1, \ldots, x_N \f$
     *  
     */
    MultivariatePolynomialFunction generateFunctionChainRule() const;

private:
    void constructProperties() {
        constructProperty_coefficients(SimTK::Vector(0));
        constructProperty_dimension(0);
        constructProperty_order(0);
    }
};


/** 
 * A helper class to construct and manipulate multivariate polynomials using 
 * symbolic operations.
 */ 
class OSIMCOMMON_API MultivariatePolynomial : 
        public std::map<std::map<std::string, int>, double> {

public:
    MultivariatePolynomial() = default;
    MultivariatePolynomial(
            int dimension, int order, const std::vector<std::string>& vars);
    MultivariatePolynomial(
            const MultivariatePolynomialFunction& mvp, 
            const std::vector<std::string>& vars);

    // CALC METHODS
    /**
     * Calculate the order of the polynomial.
     */
    int calcOrder() const;

    /**
     * Calculate the vector of coefficients of the polynomial for a given order
     * and dimension. 
     */
    SimTK::Vector calcCoefficients(int dimension, int order,
            const std::vector<std::string>& vars) const;

    
    // POLYNOMIAL OPERATIONS
    /**
     * Get a new polynomial that is the first derivative of the current 
     * polynomial with respect to the specified variable. 
     */
    MultivariatePolynomial getDerivative(const std::string& var) const;

    /**
     * Get a new polynomial that is the product of the current polynomial and 
     * a specified variable. The `exponent` argument specifies the power to
     * which the variable is raised.
     */
    MultivariatePolynomial multiplyByVariable(
            const std::string& var, int exponent = 1) const;

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
    std::pair<MultivariatePolynomial, MultivariatePolynomial> factorVariable(
            const std::string& var) const;

    /**
     * Return the sum of two MultivariatePolynomials.
     */
    static MultivariatePolynomial sum(
            const std::vector<MultivariatePolynomial>& polys);

    // HELPER METHODS
    /**
     * Generate all possible combinations of `k` elements from a set of `n`
     * total elements.
     */
    static int choose(int n, int k);

    /**
     * Generate all possible combinations of powers representing the terms of a
     * multivariate polynomial. Each combination is represented as a vector of
     * integers, where the `i`-th element is the power of the `i`-th variable.
     * For example, the combination {2, 1, 0} represents the term X^2 Y Z.
     */
    static void generateCombinations(int dimension, int order, 
            std::vector<std::vector<int>>& combinations);

    /**
     * Print a string representation of the polynomial to the output stream.
     */
    friend std::ostream& operator<<(
            std::ostream& os, const MultivariatePolynomial& poly);

private:
    typedef std::map<std::string, int> Term;
    static void generateCombinations(std::vector<int> current, int level, 
            int sum, int dimension, int order, 
            std::vector<std::vector<int>>& combinations);
};

} // namespace OpenSim

#endif // OPENSIM_MULTIVARIATEPOLYNOMIAL_FUNCTION_H_
