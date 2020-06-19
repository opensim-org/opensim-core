#ifndef OPENSIM_COMMONUTILITIES_H_
#define OPENSIM_COMMONUTILITIES_H_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  CommonUtilities.h                       *
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

#include "osimCommonDLL.h"
#include <functional>
#include <iostream>
#include <memory>

#include <SimTKcommon/internal/BigMatrix.h>

namespace OpenSim {

/// Since OpenSim does not require C++14 (which contains std::make_unique()),
/// here is an implementation of make_unique().
template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

/// Get a string with the current date and time formatted as %Y-%m-%dT%H%M%S
/// (year, month, day, "T", hour, minute, second). You can change the datetime
/// format via the `format` parameter.
/// If you specify "ISO", then we use the ISO 8601 extended datetime format
/// %Y-%m-%dT%H:%M:%S.
/// See https://en.cppreference.com/w/cpp/io/manip/put_time.
OSIMCOMMON_API std::string getFormattedDateTime(
        bool appendMicroseconds = false,
        std::string format = "%Y-%m-%dT%H%M%S");

/// When an instance of this class is destructed, it removes (deletes)
/// the file at the path provided in the constructor. You can also manually
/// cause removal of the file by invoking `remove()`.
class OSIMCOMMON_API FileRemover {
public:
    FileRemover(std::string filepath)
            : m_filepath(std::move(filepath)) {}
    /// Remove the file at the path provided in the constructor.
    void remove() const {
        std::remove(m_filepath.c_str());
    }
    /// This invokes remove().
    ~FileRemover() {
        remove();
    }
private:
    std::string m_filepath;
};

/// Create a SimTK::Vector with the provided length whose elements are
/// linearly spaced between start and end.
OSIMCOMMON_API
SimTK::Vector createVectorLinspace(int length, double start, double end);

#ifndef SWIG
/// Create a SimTK::Vector using modern C++ syntax.
OSIMCOMMON_API
SimTK::Vector createVector(std::initializer_list<SimTK::Real> elements);
#endif

/// Linearly interpolate y(x) at new values of x. The optional 'ignoreNaNs'
/// argument will ignore any NaN values contained in the input vectors and
/// create the interpolant from the non-NaN values only. Note that this option
/// does not necessarily prevent NaN values from being returned in 'newX', which
/// will have NaN for any values of newX outside of the range of x.
/// @throws Exception if x and y are different sizes, or x or y is empty.
OSIMCOMMON_API
SimTK::Vector interpolate(const SimTK::Vector& x, const SimTK::Vector& y,
        const SimTK::Vector& newX, const bool ignoreNaNs = false);

/// An OpenSim XML file may contain file paths that are relative to the
/// directory containing the XML file; use this function to convert that
/// relative path into an absolute path.
OSIMCOMMON_API
std::string getAbsolutePathnameFromXMLDocument(
        const std::string& documentFileName,
        const std::string& pathnameRelativeToDocument);

/// Solve for the root of a scalar function using the bisection method.
/// If the values of calcResidual(left) and calcResidual(right) have the same
/// sign and Logger::shouldLog(Logger::Level::Debug), then
/// this function writes a file `solveBisection_residual_<timestamp>.sto`
/// containing the residual function.
/// @param calcResidual a function that computes the error
/// @param left lower bound on the root
/// @param right upper bound on the root
/// @param tolerance convergence requires that the bisection's "left" and
///     "right" are less than tolerance apart.
/// @param maxIterations abort after this many iterations.
OSIMCOMMON_API
SimTK::Real solveBisection(std::function<double(const double&)> calcResidual,
        double left, double right, const double& tolerance = 1e-6,
        int maxIterations = 1000);

} // namespace OpenSim

#endif // OPENSIM_COMMONUTILITIES_H_
