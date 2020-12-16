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
This implementation assumes a maximum of four input dimensions and allows
computation of first-order derivatives only.
@param coefficients the polynomial coefficients in order of ascending
powers starting from the last dependent component.
For a polynomial of third order dependent on three components
(X, Y, Z), the order is a follows:
<pre>
Index | X  Y  Z
0     | 0  0  0
1     | 0  0  1
2     | 0  0  2
3     | 0  0  3
4     | 0  1  0
5     | 0  1  1
6     | 0  1  2
7     | 0  2  0
8     | 0  2  1
9     | 0  3  0
10    | 1  0  0
11    | 1  0  1
12    | 1  0  2
13    | 1  1  0
14    | 1  1  1
15    | 1  2  0
16    | 2  0  0
17    | 2  0  1
18    | 2  1  0
19    | 3  0  0
</pre>
Assuming c6 the index 6 coefficient, the corresponding term is Y Z^2.
@note The order of coefficients for this class is the *opposite** from the order
used in the univariate PolynomialFunction.
@param dimension the number of dependent components
@param order the polynomial order (the largest sum of exponents in a single term) */
class OSIMCOMMON_API MultivariatePolynomialFunction : public Function {
    OpenSim_DECLARE_CONCRETE_OBJECT(MultivariatePolynomialFunction, Function);

public:
    OpenSim_DECLARE_PROPERTY(coefficients, SimTK::Vector,
            "Coefficients of a multivariate polynomial function in order of"
            " ascending powers starting from the last dependent component.");
    OpenSim_DECLARE_PROPERTY(dimension, int,
            "Number of input dimensions (i.e., dependent components).");
    OpenSim_DECLARE_PROPERTY(
            order, int, "The largest sum of exponents in a single term.");

    MultivariatePolynomialFunction() { constructProperties(); }

    MultivariatePolynomialFunction(
            SimTK::Vector coefficients, int dimension, int order) {
        constructProperties();
        set_coefficients(coefficients);
        set_dimension(dimension);
        set_order(order);
    }

    /// Set coefficients
    void setCoefficients(SimTK::Vector coefficients) {
        set_coefficients(coefficients);
    }
    /// Get coefficients
    const SimTK::Vector& getCoefficients() const { return get_coefficients(); }
    /// Set dimension
    void setDimension(int dimension) { set_dimension(dimension); }
    /// Get dimension
    int getDimension() const { return get_dimension(); }
    /// Set order (largest sum of exponents in a single term).
    void setOrder(int order) { set_order(order); }
    /// Get order
    int getOrder() const { return get_order(); }

    /// Return function
    SimTK::Function* createSimTKFunction() const override;

private:
    void constructProperties() {
        constructProperty_coefficients(SimTK::Vector(0));
        constructProperty_dimension(0);
        constructProperty_order(0);
    }
};

} // namespace OpenSim

#endif // OPENSIM_MULTIVARIATEPOLYNOMIAL_FUNCTION_H_
