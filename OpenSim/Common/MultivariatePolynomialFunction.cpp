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
            : coefficients(coefficients), dimension(dimension), order(order) {
        OPENSIM_THROW_IF(dimension < 0 || dimension > 4, Exception,
                "Expected dimension >= 0 && <=4 but got {}.", dimension);
        OPENSIM_THROW_IF(order < 0, Exception,
                "Expected order >= 0 but got {}.", order);
        std::array<int, 4> nq{{0, 0, 0, 0}};
        int coeff_nr = 0;
        for (nq[0] = 0; nq[0] < order + 1; ++nq[0]) {
            int nq2_s;
            if (dimension < 2)
                nq2_s = 0;
            else
                nq2_s = order - nq[0];
            for (nq[1] = 0; nq[1] < nq2_s + 1; ++nq[1]) {
                int nq3_s;
                if (dimension < 3)
                    nq3_s = 0;
                else
                    nq3_s = order - nq[0] - nq[1];
                for (nq[2] = 0; nq[2] < nq3_s + 1; ++nq[2]) {
                    int nq4_s;
                    if (dimension < 4)
                        nq4_s = 0;
                    else
                        nq4_s = order - nq[0] - nq[1] - nq[2];
                    for (nq[3] = 0; nq[3] < nq4_s + 1; ++nq[3]) { ++coeff_nr; }
                }
            }
        }
        OPENSIM_THROW_IF(coefficients.size() != coeff_nr, Exception,
                "Expected {} coefficients but got {}.", coeff_nr,
                coefficients.size());
    }
    T calcValue(const SimTK::Vector& x) const override {
        std::array<int, 4> nq{{0, 0, 0, 0}};
        T value = static_cast<T>(0);
        int coeff_nr = 0;
        for (nq[0] = 0; nq[0] < order + 1; ++nq[0]) {
            int nq2_s;
            if (dimension < 2)
                nq2_s = 0;
            else
                nq2_s = order - nq[0];
            for (nq[1] = 0; nq[1] < nq2_s + 1; ++nq[1]) {
                int nq3_s;
                if (dimension < 3)
                    nq3_s = 0;
                else
                    nq3_s = order - nq[0] - nq[1];
                for (nq[2] = 0; nq[2] < nq3_s + 1; ++nq[2]) {
                    int nq4_s;
                    if (dimension < 4)
                        nq4_s = 0;
                    else
                        nq4_s = order - nq[0] - nq[1] - nq[2];
                    for (nq[3] = 0; nq[3] < nq4_s + 1; ++nq[3]) {
                        T valueP = static_cast<T>(1);
                        for (int i = 0; i < dimension; ++i) {
                            valueP *= std::pow(x[i], nq[i]);
                        }
                        value += valueP * coefficients[coeff_nr];
                        ++coeff_nr;
                    }
                }
            }
        }
        return value;
    }
    T calcDerivative(const SimTK::Array_<int>& derivComponent,
            const SimTK::Vector& x) const override {
        std::array<int, 4> nq{{0, 0, 0, 0}};
        T value = static_cast<T>(0);
        int nqNonNegative;
        int coeff_nr = 0;
        for (nq[0] = 0; nq[0] < order + 1; ++nq[0]) {
            int nq2_s;
            if (dimension < 2)
                nq2_s = 0;
            else
                nq2_s = order - nq[0];
            for (nq[1] = 0; nq[1] < nq2_s + 1; ++nq[1]) {
                int nq3_s;
                if (dimension < 3)
                    nq3_s = 0;
                else
                    nq3_s = order - nq[0] - nq[1];
                for (nq[2] = 0; nq[2] < nq3_s + 1; ++nq[2]) {
                    int nq4_s;
                    if (dimension < 4)
                        nq4_s = 0;
                    else
                        nq4_s = order - nq[0] - nq[1] - nq[2];
                    for (nq[3] = 0; nq[3] < nq4_s + 1; ++nq[3]) {
                        if (derivComponent[0] == 0) {
                            nqNonNegative = nq[0] - 1;
                            if (nqNonNegative < 0) nqNonNegative = 0;
                            T valueP = nq[0] * std::pow(x[0], nqNonNegative);
                            for (int i = 0; i < dimension; ++i) {
                                if (i == derivComponent[0]) continue;
                                valueP *= std::pow(x[i], nq[i]);
                            }
                            value += valueP * coefficients[coeff_nr];
                        } else if (derivComponent[0] == 1) {
                            nqNonNegative = nq[1] - 1;
                            if (nqNonNegative < 0) nqNonNegative = 0;
                            T valueP = nq[1] * std::pow(x[1], nqNonNegative);
                            for (int i = 0; i < dimension; ++i) {
                                if (i == derivComponent[0]) continue;
                                valueP *= std::pow(x[i], nq[i]);
                            }
                            value += valueP * coefficients[coeff_nr];
                        } else if (derivComponent[0] == 2) {
                            nqNonNegative = nq[2] - 1;
                            if (nqNonNegative < 0) nqNonNegative = 0;
                            T valueP = nq[2] * std::pow(x[2], nqNonNegative);
                            for (int i = 0; i < dimension; ++i) {
                                if (i == derivComponent[0]) continue;
                                valueP *= std::pow(x[i], nq[i]);
                            }
                            value += valueP * coefficients[coeff_nr];
                        } else if (derivComponent[0] == 3) {
                            nqNonNegative = nq[3] - 1;
                            if (nqNonNegative < 0) nqNonNegative = 0;
                            T valueP = nq[3] * std::pow(x[3], nqNonNegative);
                            for (int i = 0; i < dimension; ++i) {
                                if (i == derivComponent[0]) continue;
                                valueP *= std::pow(x[i], nq[i]);
                            }
                            value += valueP * coefficients[coeff_nr];
                        }
                        ++coeff_nr;
                    }
                }
            }
        }
        return value;
    }
    int getArgumentSize() const override { return dimension; }
    int getMaxDerivativeOrder() const override { return 1; }
    SimTKMultivariatePolynomial* clone() const override {
        return new SimTKMultivariatePolynomial(*this);
    }
    T calcDerivative(const std::vector<int>& derivComponent,
            const SimTK::Vector& x) const {
        return calcDerivative(SimTK::ArrayViewConst_<int>(derivComponent), x);
    }

private:
    SimTK::Vector_<T> coefficients;
    int dimension;
    int order;
};

SimTK::Function* MultivariatePolynomialFunction::createSimTKFunction() const {
    return new SimTKMultivariatePolynomial<SimTK::Real>(
            get_coefficients(), get_dimension(), getOrder());
}
