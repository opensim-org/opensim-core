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

SimTK::Vector OpenSim::createVectorLinspace(
        int length, double start, double end) {
    SimTK::Vector v(length);
    for (int i = 0; i < length; ++i) {
        v[i] = start + i * (end - start) / (length - 1);
    }
    return v;
}

SimTK::Vector OpenSim::createVector(
        std::initializer_list<SimTK::Real> elements) {
    return SimTK::Vector((int)elements.size(), elements.begin());
}

SimTK::Vector OpenSim::interpolate(const SimTK::Vector& x,
        const SimTK::Vector& y, const SimTK::Vector& newX,
        const bool ignoreNaNs) {

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
        if (x_no_nans[0] <= newXi && newXi <= x_no_nans[x_no_nans.size() - 1])
            newY[i] = function.calcValue(SimTK::Vector(1, newXi));
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
