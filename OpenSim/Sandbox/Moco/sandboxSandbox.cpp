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
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <casadi/casadi.hpp>
#include <Eigen/Dense>

using namespace OpenSim;

casadi::DM createLegendrePolynomialRoots(int degree) {
    // TODO based on the Benson thesis.

    // Create indices.
    std::vector<int> n;
    for (int i = 1; i < degree; ++i) {
        n.push_back(i);
    }

    // Create the subdiagonals.
    std::vector<double> d(degree-1);
    for (int i = 0; i < degree-1; ++i) {
        d[i] = n[i] / sqrt(4 * pow(n[i],2) - 1);
    }

    // Create the Jacobi matrix.
    Eigen::MatrixXd matrix(degree, degree);
    for (int i = 0; i < degree; ++i) {
        for (int j = 0; j < degree; ++j) {
            if (i == j) {
                matrix(i,j) = 0;
            } else if (i == j+1) {
                matrix(i,j) = d[j];
            } else if (i == j-1) {
                matrix(i,j) = d[i];
            } else {
                matrix(i,j) = 0;
            }
        }
    }

    // Compute the eigenvalues.
    Eigen::EigenSolver<Eigen::MatrixXd> solver(matrix);
    Eigen::VectorXd eigenvalues = solver.eigenvalues().real();
    std::vector<double> stdVector(eigenvalues.data(),
            eigenvalues.data() + eigenvalues.size());
    std::sort(stdVector.begin(), stdVector.end());

    // Convert stdVector to a casadi::DM
    casadi::DM roots(stdVector);

    return roots;
}

// Function to multiply a vector by a scalar
Eigen::VectorXd multiply(const Eigen::VectorXd& vec, const Eigen::VectorXd& x) {
    double scale = vec.prod();
    if (std::isinf(scale)) {
        Eigen::VectorXd result(x.size());
        for (int i = 0; i < x.size(); ++i) {
            Eigen::VectorXd tmp = x(i) * vec;
            result(i) = tmp.prod();
        }
        return result;
    } else {
        return scale * x;
    }
}

//template <typename T>
//int sign(T val) {
//    return (T(0) < val) - (val < T(0));
//}

Eigen::MatrixXd legendre_custom(int n, std::vector<double> x) {

    int numRoots = (int)x.size();
    std::vector<double> rootn(2 * n + 1);
    for (int i = 0; i < 2 * n + 1; ++i) {
        rootn[i] = std::sqrt(i);
    }

    std::vector<double> s(numRoots);
    for (int i = 0; i < numRoots; ++i) {
        s[i] = std::sqrt(1 - x[i] * x[i]);
    }
    Eigen::MatrixXd P(n+3, numRoots);
    P.setZero();

    std::vector<double> twocot(numRoots);
    for (int i = 0; i < numRoots; ++i) {
        twocot[i] = -2 * x[i] / s[i];
    }

    double tol = std::sqrt(std::numeric_limits<double>::min());
    std::vector<double> sn(numRoots);
    for (int i = 0; i < numRoots; ++i) {
        sn[i] = std::pow(-s[i], n);
    }

    std::vector<int> ind;
    for (int i = 0; i < numRoots; ++i) {
        if (s[i] > 0 && std::abs(sn[i]) <= tol) {
            ind.push_back(i);
        }
    }

    if (!ind.empty()) {
        std::cout << "DEBUG!!!! Need 'ind' code..." << std::endl;
    }

//    if (!ind.empty()) {
//        std::vector<double> v((int)ind.size()), w((int)ind.size());
//        std::vector<int> m1((int)ind.size());
//        for (int k = 0; k < (int)ind.size(); ++k) {
//            int col = ind[k];
//            v[k] = 9.2 - std::log(tol) / (n * s[col]);
//            w[k] = 1.0 / std::log(v[k]);
//            m1[k] = (int)std::min(static_cast<double>(n),
//                        std::floor(1.0 + n * s[col] * v[k] * w[k] *
//                                   (1.0058 + w[k] * (3.819 - w[k] * 12.173))));
//
//            for (int i = m1[k]; i <= n + 1; ++i) {
//                P[i, col] = 0;
//            }
//
//            double tstart = std::numeric_limits<double>::epsilon();
//            P[m1[k], col] = sign(static_cast<double>(m1[k] % 2) - 0.5) * tstart;
//            if (x[col] < 0) {
//                P[m1[k], col] =
//                        sign(static_cast<double>((n + 1) % 2) - 0.5) * tstart;
//            }
//
//            double sumsq = tol;
//            for (int m = m1[k] - 2; m >= 0; --m) {
//                P[m + 1, col] = ((m + 1) * twocot[col] * P[m + 2, col] -
//                    rootn[n + m + 3] * rootn[n - m] * P[m + 3, col]) /
//                    (rootn[n + m + 2] * rootn[n - m + 1]);
//                sumsq = P[m + 1, col] * P[m + 1, col] + sumsq;
//            }
//            double scale = 1.0 / std::sqrt(2 * sumsq - P[1, col] * P[1, col]);
//            for (int i = 0; i <= m1[k]; ++i) {
//                P[i, col] *= scale;
//            }
//        }
//    }

    std::vector<int> nind;
    for (int i = 0; i < numRoots; ++i) {
        if (x[i] != 1 && std::abs(sn[i]) >= tol) {
            nind.push_back(i);
        }
    }

    if (!nind.empty()) {
        std::vector<int> d;
        for (int i = 2; i <= 2 * n; i += 2) {
            d.push_back(i);
        }

        double c = 1;
        for (int i : d) {
            c *= 1.0 - 1.0 / i;
        }

        for (int col : nind) {
            P(n, col) = std::sqrt(c) * sn[col];
            P(n - 1, col) = P(n, col) * twocot[col] * n / rootn[2 * n];

            for (int m = n - 2; m >= 0; --m) {
                P(m, col) =
                        (P(m + 1, col) * twocot[col] * (m + 1) -
                         P(m + 2, col) * rootn[n + m + 2] * rootn[n - m - 1]) /
                            (rootn[n + m + 1] * rootn[n - m]);
            }
        }
    }

    Eigen::MatrixXd y(n + 1, numRoots);
    for (int i = 0; i < n+1; ++i) {
        for (int j = 0; j < numRoots; ++j) {
            y(i, j) = P(i, j);
        }
    }

    // Polar argument (x = +-1)
    for (int i = 0; i < (int)s.size(); ++i) {
        if (s[i] == 0) {
            y(0, i) = std::pow(x[i], n);
        }
    }

    for (int m = 1; m <= n - 1; ++m) {
        int start = n - m + 2;
        int end = n + m + 1;
        Eigen::VectorXd rootn_vec(end-start+1);
        int k = 0;
        for (int j = n - m + 2; j <= n + m + 1; ++j) {
            rootn_vec(k) = rootn[j-1];
            ++k;
        }
        y.row(m) = multiply(rootn_vec, y.row(m));
    }

    Eigen::VectorXd rootn_vec((int)rootn.size() - 1);
    for (int i = 0; i < rootn_vec.size(); ++i) {
        rootn_vec(i) = rootn[i+1];
    }
    y.row(n) = multiply(rootn_vec, y.row(n));

    return y;
}

int main() {
//    for (int n = 1; n <= 20; ++n) {
//        std::cout << "n = " << n << std::endl;
//
//        casadi::DM roots = createLegendrePolynomialRoots(n);
//        std::vector<double> x = roots.get_elements();
//        Eigen::MatrixXd result = legendre_custom(n + 1, x);
//
//        const auto Pnp1 = result.row(0);
//
//        std::vector<double> Pndot(Pnp1.size());
//        for (int i = 0; i < (int)Pnp1.size(); ++i) {
//            Pndot[i] = -(n + 1) / (1 - x[i] * x[i]) * Pnp1[i];
//        }
//
//        std::vector<double> w(Pndot.size());
//        for (int i = 0; i < (int)Pndot.size(); ++i) {
//            w[i] = 1 / (Pndot[i] * Pndot[i]) * (2 / (1 - x[i] * x[i]));
//        }
//
//        std::cout << "x: " << x << "\n";
//        std::cout << "w: " << w << "\n";
//        std::cout << std::endl;
//    }

    int degree = 2;
    auto roots = casadi::collocation_points(degree, "legendre");
    casadi::DM C, D, B;
    casadi::collocation_coeff(roots, C, D, B);

    std::cout << "roots: " << roots << std::endl;
    std::cout << "C: " << C << std::endl;
    std::cout << "D: " << D << std::endl;
    std::cout << "B: " << B << std::endl;

    return 0;
}

