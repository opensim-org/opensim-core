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

#include "Assertion.h"
#include "TimeSeriesTable.h"
#include "osimCommonDLL.h"
#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <numeric>
#include <stack>
#include <utility>
#include <vector>

#include <SimTKcommon/internal/BigMatrix.h>

namespace OpenSim {

/// Get a string with the current date and time formatted as %Y-%m-%dT%H%M%S
/// (year, month, day, "T", hour, minute, second). You can change the datetime
/// format via the `format` parameter.
/// If you specify "ISO", then we use the ISO 8601 extended datetime format
/// %Y-%m-%dT%H:%M:%S.
/// See https://en.cppreference.com/w/cpp/io/manip/put_time.
/// @ingroup commonutil
OSIMCOMMON_API std::string getFormattedDateTime(
        bool appendMicroseconds = false,
        std::string format = "%Y-%m-%dT%H%M%S");

/// When an instance of this class is destructed, it removes (deletes)
/// the file at the path provided in the constructor. You can also manually
/// cause removal of the file by invoking `remove()`.
/// @ingroup commonutil
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
/// uniformly spaced between start and end (same as Matlab's linspace()).
/// @ingroup commonutil
OSIMCOMMON_API
SimTK::Vector createVectorLinspace(int length, double start, double end);

/**
 * @brief Creates a vector of uniformly spaced values with a known interval.
 *
 * This function generates a vector of length `length`, where each element
 * is calculated based on the starting value and the specified step size.
 * The elements are computed as:
 * 
 * output[i] = start + i * step_size
 * 
 * for i = 0, 1, 2, ..., length-1.
 *
 * @tparam T The type of the elements in the output vector. This can be any
 *            numeric type (e.g., int, float, double).
 * @param length The number of elements in the output vector.
 * @param start The starting value of the sequence.
 * @param step_size The difference between consecutive elements in the output vector.
 * @return A std::vector<T> containing `length` elements, uniformly spaced
 *         starting from `start` with a step size of `step_size`.
 *
 * @example
 * std::vector<double> vec = createVectorLinspaceInterval(5, 0.0, 2.0);
 * // vec will contain: [0.0, 2.0, 4.0, 6.0, 8.0]
 */
 /// @ingroup commonutil
template <typename T>
std::vector<T> createVectorLinspaceInterval(
        const int length, const T start, const T step_size) {
    std::vector<int> ivec(length);
    std::iota(ivec.begin(), ivec.end(), 0); // ivec will become: [0..length]
    std::vector<T> output(ivec.size());
    std::transform(ivec.begin(), ivec.end(), output.begin(),
                    [step_size, start](int value) {
                    return static_cast<T>(std::fma(static_cast<T>(value), step_size, start));
                    });
    return output;
};

#ifndef SWIG
/// Create a SimTK::Vector using modern C++ syntax.
/// @ingroup commonutil
OSIMCOMMON_API
SimTK::Vector createVector(std::initializer_list<SimTK::Real> elements);
#endif

/**
 * @brief Detects the most likely string delimiter in an input string.
 *
 * This function identifies the most frequent delimiter from a predefined list
 * of common delimiters based on occurrences in the input string.
 *
 * @param input The string to analyze.
 * @param delimiters A vector of candidate delimiters.
 *
 * @return: The most likely delimiter (as an std::string),
 * or an empty string if none was found.
 *
 * @note Supports single-character delimiters.
 */
 /// @ingroup commonutil
OSIMCOMMON_API
std::string detectDelimiter(
        const std::string& input, const std::vector<std::string>& delimiters);

/**
 * @brief Checks if the elements of a vector are uniformly spaced.
 *
 * This function determines whether the elements in the provided vector are
 * uniformly spaced within a specified tolerance. It calculates the mean step
 * size between adjacent elements and checks if the absolute difference between
 * each step and the mean step is within the defined tolerance. If the vector
 * contains only two elements, it is considered uniform by default. Specifically
 * this verifies that the spacing between consecutive elements in numeric vector
 * x does not deviate from the mean spacing by more than 4*eps(max(abs(x))),
 * provided that the mean spacing is greater than that tolerance.
 *
 * @tparam T The type of the elements in the vector. Must support arithmetic
 * operations and the std::abs function.
 *
 * @param x A constant reference to a vector of elements of type T. The vector
 * should contain at least one element.
 *
 * @return A pair containing:
 * - A boolean indicating whether the elements are uniformly spaced
 * (true) or not (false).
 * - The calculated step size if the elements are uniform, or the
 * minimum positive step size found if they are not uniform. If the elements are
 * uniform, this value will be the mean step size. If the input is a one element
 * vector, the step size will be NaN since a valid step size cannot be
 * calculated with only 1 element.
 *
 * @note The function uses a tolerance based on the maximum absolute value of
 *       the first and last elements in the vector, scaled by machine epsilon.
 *       If the vector is empty or contains only one element,
 *       the behavior is undefined.
 * @note The function implementation draws inspiration from Matlab's `isuniform`.
 *       See https://mathworks.com/help/matlab/ref/isuniform.html for more details.
 */
/// @ingroup commonutil
template <typename T> 
std::pair<bool, T> isUniform(const std::vector<T>& x) {

    // Initialize step as NaN
    T step = std::numeric_limits<T>::quiet_NaN();
    bool tf = false;

    T maxElement = std::max(std::abs(x.front()), std::abs(x.back()));
    T tol = 4 * std::numeric_limits<T>::epsilon() * maxElement;
    size_t numSpaces = x.size() - 1;
    T span = x.back() - x.front();
    const T mean_step =
            (std::isfinite(span))
                    ? span / numSpaces
                    : (x.back() / numSpaces - x.front() / numSpaces);

    T stepAbs = std::abs(mean_step);
    if (stepAbs < tol) {
        tol = (stepAbs < std::numeric_limits<T>::epsilon() * maxElement)
                      ? std::numeric_limits<T>::epsilon() * maxElement
                      : stepAbs;
    }
    std::vector<T> results(x.size());
    std::adjacent_difference(x.begin(), x.end(), results.begin());
    // First value from adjacent_difference is the first input so it is skipped
    tf = std::all_of(
            results.begin() + 1, results.end(), [&mean_step, &tol](T val) {
                return std::abs(val - mean_step) <= tol;
            });

    if (!tf && x.size() == 2) {
        tf = true; // Handle special case for two elements
    }
    if (tf) {
        step = mean_step;
    } else {
         // Use std::remove_if to filter out non-positive numbers from the adjacent difference
        auto end = std::remove_if(results.begin(), results.end(), [](T n) { return n <= 0; });
        // Now find the minimum element among the positive numbers
        if (end != results.begin()) { // Check if there are any positive numbers
            step = *std::min_element(results.begin(), end);
        } 
    }

    return {tf, step};
}

/// Linearly interpolate y(x) at new values of x. The optional 'ignoreNaNs'
/// argument will ignore any NaN values contained in the input vectors and
/// create the interpolant from the non-NaN values only. Note that this option
/// does not necessarily prevent NaN values from being returned, which will
/// have NaN for any values of newX outside of the range of x. This is done with
/// the 'extrapolate' option. If the 'extrapolate' argument is true, then the
/// interpolant values will be extrapolated based on a piecewise linear function.
/// Setting both 'ignoreNaNs' and 'extrapolate' to true prevents NaN values from
/// occurring in the interpolant.
/// @throws Exception if x and y are different sizes, or x or y is empty.
/// @ingroup commonutil
OSIMCOMMON_API
SimTK::Vector interpolate(const SimTK::Vector& x, const SimTK::Vector& y,
        const SimTK::Vector& newX, const bool ignoreNaNs = false,
        const bool extrapolate = false);

/// An OpenSim XML file may contain file paths that are relative to the
/// directory containing the XML file; use this function to convert that
/// relative path into an absolute path.
/// @ingroup commonutil
OSIMCOMMON_API
std::string convertRelativeFilePathToAbsoluteFromXMLDocument(
        const std::string& documentFileName,
        const std::string& filePathRelativeToDirectoryContainingDocument);

/// Solve for the root of a scalar function using the bisection method.
/// If the values of calcResidual(left) and calcResidual(right) have the same
/// sign and the logger level is Debug (or more verbose), then this function
/// writes a file `solveBisection_residual_<timestamp>.sto` containing the
/// residual function.
/// @param calcResidual a function that computes the error
/// @param left lower bound on the root
/// @param right upper bound on the root
/// @param tolerance convergence requires that the bisection's "left" and
///     "right" are less than tolerance apart.
/// @param maxIterations abort after this many iterations.
/// @ingroup commonutil
OSIMCOMMON_API
SimTK::Real solveBisection(std::function<double(const double&)> calcResidual,
        double left, double right, const double& tolerance = 1e-6,
        int maxIterations = 1000);

/**
 * @brief Rotates all marker in a table by the rotation matrix R
 *
 *
 * @param table of marker positions that will be modified in place.
 * @param R rotation matrix for desired rotation
 *
 * @return: None
 */
/// @ingroup commonutil
OSIMCOMMON_API
void rotateMarkerTable(
        OpenSim::TimeSeriesTableVec3& table, const SimTK::Rotation& R);

/// This class lets you store objects of a single type for reuse by multiple
/// threads, ensuring threadsafe access to each of those objects.
/// @ingroup commonutil
// TODO: Find a way to always give the same thread the same object.
template <typename T> class ThreadsafeJar {
public:
    /// Request an object for your exclusive use on your thread. This function
    /// blocks the thread until an object is available. Make sure to return
    /// (leave()) the object when you're done!
    std::unique_ptr<T> take() {
        // Only one thread can lock the mutex at a time, so only one thread
        // at a time can be in any of the functions of this class.
        std::unique_lock<std::mutex> lock(m_mutex);
        // Block this thread until the condition variable is woken up
        // (by a notify_...()) and the lambda function returns true.
        m_inventoryMonitor.wait(lock, [this] { return m_entries.size() > 0; });
        std::unique_ptr<T> top = std::move(m_entries.top());
        m_entries.pop();
        return top;
    }
    /// Add or return an object so that another thread can use it. You will need
    /// to std::move() the entry, ensuring that you will no longer have access
    /// to the entry in your code (the pointer will now be null).
    void leave(std::unique_ptr<T> entry) {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_entries.push(std::move(entry));
        lock.unlock();
        m_inventoryMonitor.notify_one();
    }
    /// Obtain the number of entries that can be taken.
    int size() const {
        std::lock_guard<std::mutex> lock(m_mutex);
        return (int)m_entries.size();
    }

private:
    std::stack<std::unique_ptr<T>> m_entries;
    mutable std::mutex m_mutex;
    std::condition_variable m_inventoryMonitor;
};

/// Compute the 'k' nearest neighbors of two matrices 'x' and 'y'. 'x' and 'y'
/// should contain the same number of columns, but can have different numbers of
/// rows. The function returns a matrix with 'k' number of columns and the same
/// number of rows as 'y'. Each row in the output matrix contains 'k' distance
/// values, where the first column contains the distance to the nearest neighbor
/// in 'x', the second column contains the distance to the second nearest
/// neighbor in 'x', and so on.
/// @ingroup commonutil
OSIMCOMMON_API SimTK::Matrix computeKNearestNeighbors(const SimTK::Matrix& x,
        const SimTK::Matrix& y, int k = 1);

/// Use non-negative matrix factorization to decompose an matrix A (NxM) for a 
/// selected number of factors 'K' into two matrices W (NxK) and H (KxM) such 
/// that A = W * H. The alternating least squares (ALS) algorithm is used to 
/// solve for W and H by minimizing the Frobenius norm of the error between A 
/// and W * H. The matrices W and H are scaled assuming that the rows of H
/// have magnitudes as if all elements in H were equal to 0.5, which prevents
/// individual factors from being very large or very small. The algorithm 
/// terminates when the change in the error norm is less than the specified 
/// tolerance or the maximum number of iterations is reached.
///
/// @returns The final Frobenius norm of the error between A and W * H.
///
/// Reference
/// ---------
/// Berry, M. W., et al. (2007). Algorithms and Applications for Approximate 
/// Nonnegative Matrix Factorization. Computational Statistics & Data Analysis, 
/// 52(1), 155-173. doi:10.1016/j.csda.2006.11.006.
OSIMCOMMON_API double factorizeMatrixNonNegative(const SimTK::Matrix& A, 
        int numFactors, int maxIterations, double tolerance, 
        SimTK::Matrix& W, SimTK::Matrix& H);

} // namespace OpenSim

#endif // OPENSIM_COMMONUTILITIES_H_
