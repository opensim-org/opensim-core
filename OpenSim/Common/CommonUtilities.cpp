/* -------------------------------------------------------------------------- *
 *                          OpenSim:  CommonUtilities.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2019 Stanford University and the Authors                *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "CommonUtilities.h"

#include "PiecewiseLinearFunction.h"
#include "STOFileAdapter.h"
#include "TimeSeriesTable.h"
#include <chrono>
#include <ctime>
#include <iomanip>
#include <memory>
#include <sstream>
#include <unordered_map>

#include <SimTKcommon/internal/Pathname.h>

std::string OpenSim::getFormattedDateTime(
        bool appendMicroseconds, std::string format) {
    using namespace std::chrono;
    auto now = system_clock::now();
    auto time_now = system_clock::to_time_t(now);
    struct tm buf;
#if defined(_WIN32)
    localtime_s(&buf, &time_now);
#else
    localtime_r(&time_now, &buf);
#endif
    if (format == "ISO") { format = "%Y-%m-%dT%H:%M:%S"; }

    // To get the date/time in the desired format, we would ideally use
    // std::put_time, but that is not available in GCC < 5.
    // https://stackoverflow.com/questions/30269657/what-is-an-intelligent-way-to-determine-max-size-of-a-strftime-char-array
    int size = 32;
    std::unique_ptr<char[]> formatted(new char[size]);
    while (strftime(formatted.get(), size - 1, format.c_str(), &buf) == 0) {
        size *= 2;
        formatted.reset(new char[size]);
    }

    std::stringstream ss;
    ss << formatted.get();

    if (appendMicroseconds) {
        // Get number of microseconds since last second.
        auto microsec =
                duration_cast<microseconds>(now.time_since_epoch()) % 1000000;
        ss << '.' << std::setfill('0') << std::setw(6) << microsec.count();
    }
    return ss.str();
}

std::string OpenSim::detectDelimiter(
        const std::string& input, const std::vector<std::string>& delimiters) {

    std::unordered_map<std::string, std::size_t> counts;

    // Count occurrences of common delimiters in the input string
    std::transform(delimiters.begin(), delimiters.end(),
            std::inserter(counts, counts.end()), [&input](const std::string& d) {
                std::size_t count = 0;
                std::size_t pos = 0;
                // Find all occurrences of delimiter in input string
                while ((pos = input.find(d, pos)) != std::string::npos) {
                    ++count;
                    pos += d.length(); // Move past the current delimiter
                }
                return std::pair<std::string, std::size_t>(d, count);
            });

    // Find the delimiter with the highest frequency
    auto maxElem = std::max_element(counts.begin(), counts.end(),
            [](const auto& a, const auto& b) { return a.second < b.second; });

    // If a delimiter is found, return it, otherwise return an empty string
    return (maxElem != counts.end() && maxElem->second > 0) ? maxElem->first : "";
}

SimTK::Vector OpenSim::createVectorLinspace(
        int length, double start, double end) {
    SimTK::Vector v(length);
    const double step_size = (end - start) / static_cast<double>((length - 1));
    for (int i = 0; i < length; ++i) {
        v[i] = std::fma(i, step_size, start);
    }
    return v;
}

SimTK::Vector OpenSim::createVector(
        std::initializer_list<SimTK::Real> elements) {
    return SimTK::Vector((int)elements.size(), elements.begin());
}

SimTK::Vector OpenSim::interpolate(const SimTK::Vector& x,
        const SimTK::Vector& y, const SimTK::Vector& newX,
        const bool ignoreNaNs, const bool extrapolate) {

    OPENSIM_THROW_IF(x.size() != y.size(), Exception,
            "Expected size of x to equal size of y, but size of x "
            "is {} and size of y is {}.",
            x.size(), y.size());

    // Create vectors of non-NaN values if user set 'ignoreNaNs' argument to
    // 'true', otherwise throw an exception. If no NaN's are present in the
    // provided data vectors, the '*_no_nans' variables below will contain
    // the original data vector values.
    std::vector<double> x_no_nans;
    std::vector<double> y_no_nans;
    for (int i = 0; i < x.size(); ++i) {

        bool shouldNotPushBack =
                (SimTK::isNaN(x[i]) || SimTK::isNaN(y[i])) && ignoreNaNs;
        if (!shouldNotPushBack) {
            x_no_nans.push_back(x[i]);
            y_no_nans.push_back(y[i]);
        }
    }

    OPENSIM_THROW_IF(x_no_nans.empty(), Exception,
            "Input vectors are empty (perhaps after removing NaNs).");

    PiecewiseLinearFunction function(
            (int)x_no_nans.size(), &x_no_nans[0], &y_no_nans[0]);
    SimTK::Vector newY(newX.size(), SimTK::NaN);
    for (int i = 0; i < newX.size(); ++i) {
        const auto& newXi = newX[i];
        bool inBounds = (x_no_nans[0] <= newXi) &&
                        (newXi <= x_no_nans[x_no_nans.size() - 1]);
        if (extrapolate || inBounds) {
            newY[i] = function.calcValue(SimTK::Vector(1, newXi));
        }
    }
    return newY;
}

std::string OpenSim::convertRelativeFilePathToAbsoluteFromXMLDocument(
        const std::string& documentFileName,
        const std::string& filePathRelativeToDocument) {
    // Get the directory containing the XML file.
    std::string directory;
    bool dontApplySearchPath;
    std::string fileName, extension;
    SimTK::Pathname::deconstructPathname(documentFileName, dontApplySearchPath,
                                         directory, fileName, extension);
    return SimTK::Pathname::getAbsolutePathnameUsingSpecifiedWorkingDirectory(
            directory, filePathRelativeToDocument);
}

void OpenSim::rotateMarkerTable(
        OpenSim::TimeSeriesTableVec3& table, const SimTK::Rotation& R) {
    size_t nt = table.getNumRows();
    // Rotate table
    std::vector<size_t> row_indicies(nt);
    std::iota(row_indicies.begin(), row_indicies.end(), 0);
    std::for_each(
            row_indicies.begin(), row_indicies.end(), [&table, &R](size_t i) {
                auto row = table.updRowAtIndex(i);
                std::transform(row.begin(), row.end(), row.begin(),
                        [&R](const SimTK::Vec<3>& v) { return R * v; });
            });
}

SimTK::Real OpenSim::solveBisection(
        std::function<SimTK::Real(const SimTK::Real&)> calcResidual,
        SimTK::Real left, SimTK::Real right, const SimTK::Real& tolerance,
        int maxIterations) {
    SimTK::Real midpoint = left;

    OPENSIM_THROW_IF(maxIterations < 0, Exception,
            "Expected maxIterations to be positive, but got {}.",
            maxIterations);

    const bool sameSign = calcResidual(left) * calcResidual(right) >= 0;
    if (sameSign && Logger::shouldLog(Logger::Level::Debug)) {
        const int numRows = 1000;
        const auto x = createVectorLinspace(numRows, left, right);
        SimTK::Matrix residual(numRows, 1);
        for (int i = 0; i < numRows; ++i) {
            residual(i, 0) = calcResidual(x[i]);
        }
        TimeSeriesTable table(std::vector<double>(x.getContiguousScalarData(),
                                      x.getContiguousScalarData() + x.size()),
                residual, {"residual"});
        STOFileAdapter::write(
                table, fmt::format("solveBisection_residual_{}.sto",
                               getFormattedDateTime()));
    }
    OPENSIM_THROW_IF(sameSign, Exception,
            "Function has same sign at bounds of {} and {}.", left, right);

    SimTK::Real residualMidpoint;
    SimTK::Real residualLeft = calcResidual(left);
    int iterCount = 0;
    while (iterCount < maxIterations && (right - left) > tolerance) {
        midpoint = 0.5 * (left + right);
        residualMidpoint = calcResidual(midpoint);
        if (residualMidpoint * residualLeft < 0) {
            // The solution is to the left of the current midpoint.
            right = midpoint;
        } else {
            left = midpoint;
            residualLeft = calcResidual(left);
        }
        ++iterCount;
    }
    if (iterCount == maxIterations) {
        log_warn("Bisection reached max iterations at x = {}.", midpoint);
    }
    return midpoint;
}

SimTK::Matrix OpenSim::computeKNearestNeighbors(const SimTK::Matrix& x,
        const SimTK::Matrix& y, int k) {

    OPENSIM_THROW_IF(x.ncol() != y.ncol(), Exception,
            "Expected the matrices 'x' and 'y' to have the same number of "
            "columns, but found {} and {}, respectively.",
            x.ncol(), y.ncol());

    // Initialize the output matrices.
    SimTK::Matrix distances(y.nrow(), k, 0.0);

    // Loop over each row in 'y'.
    for (int iy = 0; iy < y.nrow(); ++iy) {
        std::vector<double> distancesVec;
        distancesVec.reserve(x.nrow());

        // Compute distances between the current row in 'y' and all rows in 'x'.
        for (int ix = 0; ix < x.nrow(); ++ix) {
            auto row_y = y.row(iy).getAsRowVector();
            auto row_x = x.row(ix).getAsRowVector();
            auto diff = row_y - row_x;
            distancesVec.push_back(diff.norm());
        }

        // Sort the distances in ascending order.
        std::sort(distancesVec.begin(), distancesVec.end(),
                [](const double& a, const double& b) { return a < b; });

        // Take the first K distances.
        for (int ik = 0; ik < k; ++ik) {
            distances.set(iy, ik, distancesVec[ik]);
        }
    }

    return distances;
}

double OpenSim::factorizeMatrixNonNegative(const SimTK::Matrix& A, 
        int numFactors, int maxIterations, double tolerance, 
        SimTK::Matrix& W, SimTK::Matrix& H) {

    // Initialize W and H.
    int k = numFactors;
    W.clear();
    H.clear();
    W.resize(A.nrow(), k);
    H.resize(k, A.ncol());

    log_info("");
    log_info("Non-negative matrix factorization");
    log_info("---------------------------------");
    log_info("Number of factors, k = {}", numFactors);
    log_info("A: {} x {}", A.nrow(), A.ncol());
    log_info("W: {} x {}", W.nrow(), W.ncol());
    log_info("H: {} x {}", H.nrow(), H.ncol());
    log_info("Max iterations = {}", maxIterations);
    log_info("Tolerance = {}", tolerance);
    log_info("");

    // Create random initial guess.
    SimTK::Random::Uniform rand(0,1);
    SimTK::Matrix W0(A.nrow(), k);
    for (int i = 0; i < W0.nrow(); ++i) {
        for (int j = 0; j < W0.ncol(); ++j) {
            W0(i, j) = rand.getValue();
        }
    }

    // Initialize error matrix.
    SimTK::Matrix error(A.nrow(), A.ncol());
    double normError0 = SimTK::Infinity;
    double normError = SimTK::Infinity;
    double deltaError = SimTK::Infinity;    
    for (int j = 0; j < maxIterations; ++j) {

        // Alternating least squares.
        // W * H = A --> H = W^(-1) * A
        SimTK::FactorSVD svd_W(W0);
        svd_W.solve(A, H);
        for (int i = 0; i < H.nrow(); ++i) {
            for (int k = 0; k < H.ncol(); ++k) {
                // Ensure non-negativity.
                H(i, k) = std::max(0.0, H(i, k));
            }
        }
        // ~H * ~W = ~A --> ~W = ~H^(-1) * ~A  
        SimTK::FactorSVD svd_H(H.transpose().getAsMatrix());
        svd_H.solve(A.transpose().getAsMatrix(), W.transpose().updAsMatrix());
        for (int i = 0; i < W.nrow(); ++i) {
            for (int k = 0; k < W.ncol(); ++k) {
                // Ensure non-negativity.
                W(i, k) = std::max(0.0, W(i, k));
            }
        }

        // Scale W and H assuming that the elements of H are all 0.5.
        SimTK::Vector scaleVec(H.ncol(), 0.5);
        for (int i = 0; i < k; ++i) {
            double scale = scaleVec.norm() / H.row(i).norm();
            H.updRow(i) *= scale;
            W.updCol(i) /= scale;
        }

        // Compute error using the Frobenius norm.
        error = A - W * H;
        normError = std::sqrt(error.scalarNormSqr()) / 
                           std::sqrt(A.nelt());
        deltaError = normError0 - normError;
        log_info("Iteration {} | norm(error) = {} | change in norm(error) = {}", 
                j, normError, deltaError);

        // Check for convergence.
        if (deltaError < tolerance) {
            log_info("Converged after {} iterations.", j + 1);
            log_info("");
            break;
        } else if (j == maxIterations - 1) {
            log_info("Max iterations reached. Exiting...");
            log_info("");
            break;
        }
    
        // Update variables.
        normError0 = normError;
        W0 = W;
    }

    return normError;
}
