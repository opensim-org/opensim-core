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
    /// @param coefficients the polynomial coefficients
    /// @param dimension the number of dimensions
    /// @param order the polynomial order
    SimTKMultivariatePolynomial(const SimTK::Vector_<T>& coefficients,
            const int& dimension, const int& order) :
            coefficients(coefficients), dimension(dimension), order(order) {}
    T calcValue(const SimTK::Vector& x) const override {
        // TODO: not sure this is necessary but this implementation assumes 4
        // dimensions so it might be good to make sure the vector has only 0 to
        // start with. I have not been very successful in coming up with a
        // generic expression so far...
        SimTK::Vector y(4);
        y.setToZero();
        for (int i = 0; i < 4; ++i) y[i] = x[i];
        T value = static_cast<T>(0);
        int coeff_nr = 0;
        for (int nq_1 = 0; nq_1 < order+1; ++nq_1) {
            int n_q2_s;
            if (dimension < 2) n_q2_s = 0;
            else n_q2_s = order-nq_1;
            for (int n_q2 = 0; n_q2 < n_q2_s + 1; ++n_q2) {
                int n_q3_s;
                if (dimension < 3) n_q3_s = 0;
                else n_q3_s = order-nq_1-n_q2;
                for (int n_q3 = 0; n_q3 < n_q3_s + 1; ++n_q3) {
                    int n_q4_s;
                    if (dimension < 4) n_q4_s = 0;
                    else n_q4_s = order-nq_1-n_q2-n_q3;
                    for (int n_q4 = 0; n_q4 < n_q4_s + 1; ++n_q4) {
                        value += (std::pow(y[0], nq_1) * std::pow(y[1], n_q2) *
                                std::pow(y[2], n_q3) * std::pow(y[3], n_q4)) *
                                coefficients[coeff_nr];
                        ++coeff_nr;
                    }
                }
            }
        }
        return value;
    }
    T calcDerivative(const SimTK::Array_<int>& derivComponents,
                     const SimTK::Vector& x) const override {
        // TODO: idem as above.
        SimTK::Vector y(4);
        y.setToZero();
        for (int i = 0; i < 4; ++i) y[i] = x[i];
        T value = static_cast<T>(0);
        int temp = 0;
        int coeff_nr = 0;
        for (int nq_1 = 0; nq_1 < order+1; ++nq_1) {
            int n_q2_s;
            if (dimension < 2) n_q2_s = 0;
            else n_q2_s = order-nq_1;
            for (int n_q2 = 0; n_q2 < n_q2_s + 1; ++n_q2) {
                int n_q3_s;
                if (dimension < 3) n_q3_s = 0;
                else n_q3_s = order-nq_1-n_q2;
                for (int n_q3 = 0; n_q3 < n_q3_s + 1; ++n_q3) {
                    int n_q4_s;
                    if (dimension < 4) n_q4_s = 0;
                    else n_q4_s = order-nq_1-n_q2-n_q3;
                    for (int n_q4 = 0; n_q4 < n_q4_s + 1; ++n_q4) {
                        if (derivComponents[0] == 0) {
                            temp = nq_1-1; // TODO dirty fix
                            if (temp < 0) temp = 0;
                            value += (nq_1 * std::pow(y[0], temp)*
                                    std::pow(y[1], n_q2) *
                                    std::pow(y[2], n_q3) *
                                    std::pow(y[3], n_q4)) *
                                    coefficients[coeff_nr];
                        }
                        else if (derivComponents[0] == 1) {
                            temp = n_q2-1; // TODO dirty fix
                            if (temp < 0) temp = 0;
                            value += (std::pow(y[0], nq_1) *
                                    n_q2 * std::pow(y[1], temp) *
                                    std::pow(y[2], n_q3) *
                                    std::pow(y[3], n_q4)) *
                                    coefficients[coeff_nr];
                        }
                        else if (derivComponents[0] == 2) {
                            temp = n_q3-1; // TODO dirty fix
                            if (temp < 0) temp = 0;
                            value += (std::pow(y[0], nq_1) *
                                    std::pow(y[1], n_q2) *
                                    n_q3 * std::pow(y[2], temp) *
                                    std::pow(y[3], n_q4)) *
                                    coefficients[coeff_nr];
                        }
                        else if (derivComponents[0] == 3) {
                            temp = n_q4-1; // TODO dirty fix
                            if (temp < 0) temp = 0;
                            value += (std::pow(y[0], nq_1) *
                                    std::pow(y[1], n_q2) *
                                    std::pow(y[2], n_q3) *
                                    n_q4 * std::pow(y[3], temp)) *
                                    coefficients[coeff_nr];
                        }
                        ++coeff_nr;
                    }
                }
            }
        }
        return value;
    }
    int getArgumentSize() const override {
        return dimension;
    }
    int getMaxDerivativeOrder() const override {
        return std::numeric_limits<int>::max();
    }
    SimTKMultivariatePolynomial* clone() const override {
        return new SimTKMultivariatePolynomial(*this);
    }
    /// This provides compatibility with std::vector without requiring any
    /// copying.
    T calcDerivative(const std::vector<int>& derivComponents,
        const SimTK::Vector& x) const {
        return calcDerivative(SimTK::ArrayViewConst_<int>(derivComponents), x);
    }
private:
    SimTK::Vector_<T> coefficients;
    int dimension;
    int order;
};

class OSIMMOCO_API MultivariatePolynomialFunction : public Function {
OpenSim_DECLARE_CONCRETE_OBJECT(MultivariatePolynomialFunction, Function);

    //=========================================================================
    // PROPERTIES
    //=========================================================================
    // TODO ajust after discussing "format"
    OpenSim_DECLARE_PROPERTY(coefficients, SimTK::Vector,
            "Coefficients of a multivariate polynomial function.");
    OpenSim_DECLARE_PROPERTY(dimension, int,
            "Number of dimensions of a multivariate polynomial function.");
    OpenSim_DECLARE_PROPERTY(order, int,
            "Order of a multivariate polynomial function.");

public:
    //=========================================================================
    // METHODS
    //=========================================================================
    MultivariatePolynomialFunction() { constructProperties(); }

    MultivariatePolynomialFunction(SimTK::Vector coefficients, int dimension,
            int order) {
        constructProperties();
        set_coefficients(coefficients);
        set_dimension(dimension);
        set_order(order);
    }

    virtual ~MultivariatePolynomialFunction() {};

    /// Set coefficients TODO
    void setCoefficients(SimTK::Vector coefficients)
    {   set_coefficients(coefficients); }
    /// Get coefficients TODO
    const SimTK::Vector getCoefficients() const
    {   return get_coefficients(); }
    /// Set dimension TODO
    void setDimension(int dimension)
    {   set_dimension(dimension); }
    /// Get dimension TODO
    const int getDimension() const
    {   return get_dimension(); }
    /// Set order TODO
    void setOrder(int order)
    {   set_order(order); }
    /// Get order TODO
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
                SimTK::Vector(6,coefficientDefaultValues));
        constructProperty_dimension(2);
        constructProperty_order(2);
    }

//=============================================================================
};  // END class MultivariatePolynomialFunction
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif  // OPENSIM_MULTIVARIATEPOLYNOMIAL_FUNCTION_H_
