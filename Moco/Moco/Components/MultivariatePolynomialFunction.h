#ifndef OPENSIM_MULTIVARIATEPOLYNOMIAL_FUNCTION_H_
#define OPENSIM_MULTIVARIATEPOLYNOMIAL_FUNCTION_H_
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MultivariatePolynomialFunction.h                             *
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

#include "../MocoUtilities.h"
#include "../osimMocoDLL.h"

namespace OpenSim {

template <class T>
class SimTKMultivariatePolynomial : public SimTK::Function_<T> {
public:
    /// Create a SimTKMultivariatePolynomial object.
    /// This implementation assumes a maximum of four dimensions and allows
    /// computation of order one derivatives only.
    /// @param coefficients the polynomial coefficients in order of ascending
    /// powers starting from the last dependent component.
    /// For a polynomial of third order dependent on three components
    /// (X, Y, Z), the order is a follows:
    /// <pre>
    /// Index | X  Y  Z
    /// 1     | 0  0  0
    /// 2     | 0  0  1
    /// 3     | 0  0  2
    /// 4     | 0  0  3
    /// 5     | 0  1  0
    /// 6     | 0  1  1
    /// 7     | 0  1  2
    /// 8     | 0  2  0
    /// 9     | 0  2  1
    /// 10    | 0  3  0
    /// 11    | 1  0  0
    /// 12    | 1  0  1
    /// 13    | 1  0  2
    /// 14    | 1  1  0
    /// 15    | 1  1  1
    /// 16    | 1  2  0
    /// 17    | 2  0  0
    /// 18    | 2  0  1
    /// 19    | 2  1  0
    /// 20    | 3  0  0
    /// </pre>
    /// @param dimension the number of dependent components
    /// @param order the polynomial order
    SimTKMultivariatePolynomial(const SimTK::Vector_<T>& coefficients,
            const int& dimension, const int& order) :
            coefficients(coefficients), dimension(dimension), order(order) {}
    /// Calculate the value of this function at a particular point.
    /// @param x the Vector of input arguments (e.g., x[0] is the value of the
    /// first component). The size of x must equal the value returned by
    /// getArgumentSize().
    T calcValue(const SimTK::Vector& x) const override {
        SimTK::Vector y(4);
        y.setToZero();
        for (int i = 0; i < 4; ++i) y[i] = x[i];
        T value = static_cast<T>(0);
        int coeff_nr = 0;
        for (int nq1 = 0; nq1 < order + 1; ++nq1) {
            int nq2_s;
            if (dimension < 2) nq2_s = 0;
            else nq2_s = order - nq1;
            for (int nq2 = 0; nq2 < nq2_s + 1; ++nq2) {
                int nq3_s;
                if (dimension < 3) nq3_s = 0;
                else nq3_s = order - nq1 - nq2;
                for (int nq3 = 0; nq3 < nq3_s + 1; ++nq3) {
                    int nq4_s;
                    if (dimension < 4) nq4_s = 0;
                    else nq4_s = order - nq1 - nq2 - nq3;
                    for (int nq4 = 0; nq4 < nq4_s + 1; ++nq4) {
                        value += (std::pow(y[0], nq1) * std::pow(y[1], nq2) *
                                std::pow(y[2], nq3) * std::pow(y[3], nq4)) *
                                coefficients[coeff_nr];
                        ++coeff_nr;
                    }
                }
            }
        }
        return value;
    }
    /// Calculate a partial derivative of this function at a particular point.
    /// Which derivative to take is specified by listing the input components
    /// with which to take it. This implementation allows computation of order
    /// one derivatives only. For example, if derivComponent=={0}, that
    /// indicates a first derivative with respective to component 0. If
    /// derivComponent=={2}, that indicates a first derivative with
    /// respective to component 2.
    /// @param derivComponent the input component with respect to which the
    /// derivative should be taken.  Its size must be one.
    /// @param x the Vector of input arguments (e.g., x[0] is the value of the
    /// first component). The size of x must equal the value returned by
    /// getArgumentSize().
    T calcDerivative(const SimTK::Array_<int>& derivComponent,
                     const SimTK::Vector& x) const override {
        SimTK::Vector y(4);
        y.setToZero();
        for (int i = 0; i < 4; ++i) y[i] = x[i];
        T value = static_cast<T>(0);
        int nqNonNegative = 0;
        int coeff_nr = 0;
        for (int nq1 = 0; nq1 < order + 1; ++nq1) {
            int nq2_s;
            if (dimension < 2) nq2_s = 0;
            else nq2_s = order - nq1;
            for (int nq2 = 0; nq2 < nq2_s + 1; ++nq2) {
                int nq3_s;
                if (dimension < 3) nq3_s = 0;
                else nq3_s = order - nq1 - nq2;
                for (int nq3 = 0; nq3 < nq3_s + 1; ++nq3) {
                    int nq4_s;
                    if (dimension < 4) nq4_s = 0;
                    else nq4_s = order - nq1 - nq2 - nq3;
                    for (int nq4 = 0; nq4 < nq4_s + 1; ++nq4) {
                        if (derivComponent[0] == 0) {
                            nqNonNegative = nq1 - 1;
                            if (nqNonNegative < 0) nqNonNegative = 0;
                            value += (nq1 * std::pow(y[0], nqNonNegative)*
                                    std::pow(y[1], nq2) *
                                    std::pow(y[2], nq3) *
                                    std::pow(y[3], nq4)) *
                                    coefficients[coeff_nr];
                        }
                        else if (derivComponent[0] == 1) {
                            nqNonNegative = nq2 - 1;
                            if (nqNonNegative < 0) nqNonNegative = 0;
                            value += (std::pow(y[0], nq1) *
                                    nq2 * std::pow(y[1], nqNonNegative) *
                                    std::pow(y[2], nq3) *
                                    std::pow(y[3], nq4)) *
                                    coefficients[coeff_nr];
                        }
                        else if (derivComponent[0] == 2) {
                            nqNonNegative = nq3 - 1;
                            if (nqNonNegative < 0) nqNonNegative = 0;
                            value += (std::pow(y[0], nq1) *
                                    std::pow(y[1], nq2) *
                                    nq3 * std::pow(y[2], nqNonNegative) *
                                    std::pow(y[3], nq4)) *
                                    coefficients[coeff_nr];
                        }
                        else if (derivComponent[0] == 3) {
                            nqNonNegative = nq4 - 1;
                            if (nqNonNegative < 0) nqNonNegative = 0;
                            value += (std::pow(y[0], nq1) *
                                    std::pow(y[1], nq2) *
                                    std::pow(y[2], nq3) *
                                    nq4 * std::pow(y[3], nqNonNegative)) *
                                    coefficients[coeff_nr];
                        }
                        ++coeff_nr;
                    }
                }
            }
        }
        return value;
    }
    /// Get the number of components expected in the input vector.
    int getArgumentSize() const override {
        return dimension;
    }
    /// Get the maximum derivative order this function can calculate.
    int getMaxDerivativeOrder() const override {
        return 1;
    }
    SimTKMultivariatePolynomial* clone() const override {
        return new SimTKMultivariatePolynomial(*this);
    }
    /// This provides compatibility with std::vector without requiring any
    /// copying.
    T calcDerivative(const std::vector<int>& derivComponent,
        const SimTK::Vector& x) const {
        return calcDerivative(SimTK::ArrayViewConst_<int>(derivComponent), x);
    }
private:
    SimTK::Vector_<T> coefficients;
    int dimension;
    int order;
};

class OSIMMOCO_API MultivariatePolynomialFunction : public Function {
OpenSim_DECLARE_CONCRETE_OBJECT(MultivariatePolynomialFunction, Function);

public:
    OpenSim_DECLARE_PROPERTY(coefficients, SimTK::Vector,
            "Coefficients of a multivariate polynomial function in order of"
            " ascending powers starting from the last dependent component ("
            " see SimTKMultivariatePolynomial for example)");
    OpenSim_DECLARE_PROPERTY(dimension, int,
            "Number of dimensions (i.e., dependent components) of a"
            " multivariate polynomial function.");
    OpenSim_DECLARE_PROPERTY(order, int,
            "Order of a multivariate polynomial function.");

    MultivariatePolynomialFunction() { constructProperties(); }

    MultivariatePolynomialFunction(SimTK::Vector coefficients, int dimension,
            int order) {
        constructProperties();
        set_coefficients(coefficients);
        set_dimension(dimension);
        set_order(order);
    }

    virtual ~MultivariatePolynomialFunction() {};

    /// Set coefficients
    void setCoefficients(SimTK::Vector coefficients)
    {   set_coefficients(coefficients); }
    /// Get coefficients
    const SimTK::Vector getCoefficients() const
    {   return get_coefficients(); }
    /// Set dimension
    void setDimension(int dimension)
    {   set_dimension(dimension); }
    /// Get dimension
    const int getDimension() const
    {   return get_dimension(); }
    /// Set order
    void setOrder(int order)
    {   set_order(order); }
    /// Get order
    const int getOrder() const
    {   return get_order(); }

    /// Return function
    SimTK::Function* createSimTKFunction() const override {
        return new SimTKMultivariatePolynomial<SimTK::Real>(get_coefficients(),
                get_dimension(), getOrder());
    }

private:
    void constructProperties() {
        double coefficientDefaultValues[6] = {1,1,1,1,1,1};
        constructProperty_coefficients(
                SimTK::Vector(6, &coefficientDefaultValues[0]));
        constructProperty_dimension(2);
        constructProperty_order(2);
    }

};

} // end of namespace OpenSim

#endif  // OPENSIM_MULTIVARIATEPOLYNOMIAL_FUNCTION_H_
