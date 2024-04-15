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

/** 
 * A multivariate polynomial function.
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
 * @note The order of coefficients for this class is the opposite from the
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
     */
    MultivariatePolynomialFunction generateFunctionChainRule() const;

private:
    void constructProperties() {
        constructProperty_coefficients(SimTK::Vector(0));
        constructProperty_dimension(0);
        constructProperty_order(0);
    }
};

} // namespace OpenSim

#endif // OPENSIM_MULTIVARIATEPOLYNOMIAL_FUNCTION_H_
