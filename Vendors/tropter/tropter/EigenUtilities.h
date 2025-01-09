#ifndef TROPTER_EIGENUTILITIES_H
// ----------------------------------------------------------------------------
// tropter: EigenUtilities.h
// ----------------------------------------------------------------------------
// Copyright (c) 2017 tropter authors
//
// Licensed under the Apache License, Version 2.0 (the "License"); you may
// not use this file except in compliance with the License. You may obtain a
// copy of the License at http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ----------------------------------------------------------------------------
#define TROPTER_EIGENUTILITIES_H

#include <tropter/common.h>
#include <tropter/Exception.hpp>

#include <iostream>

namespace tropter {

/// Write an Eigen matrix to a file in a CSV (comma-separated values) format.
/// If you specify column labels, you must specify one for each column of the
/// matrix.
template<typename Derived>
void write(const Eigen::DenseBase<Derived>& matrix, const std::string& filepath,
        const std::vector<std::string>& columnLabels = {})
{
    std::ofstream f(filepath);
    if (matrix.cols() == 0) {
        f.close();
        return;
    }

    // Column labels.
    if (!columnLabels.empty()) {
        if (columnLabels.size() != size_t(matrix.cols())) {
            throw std::runtime_error("Number of column labels ("
                    + std::to_string(columnLabels.size()) + ") does not match "
                    "the number of columns in the matrix ("
                    + std::to_string(matrix.cols()) + ").");
        }

        f << columnLabels[0];
        for (size_t i_col = 1; i_col < columnLabels.size(); ++i_col) {
            f << "," << columnLabels[i_col];
        }
        f << std::endl;
    }

    // Data.
    // Separate coefficients with a comma.
    Eigen::IOFormat format(Eigen::StreamPrecision, 0, ",");
    // This function only exists on DenseBase (not on EigenBase).
    f << matrix.format(format) << std::endl;
}

/// Similar to the other write() function, but this one also includes a time
/// column. Additionally, the matrix should have time along its columns;
/// it will be transposed in the file so that time is along the rows. The
/// number of columns in matrix must match the number of rows in time.
template<typename Derived>
void write(const Eigen::VectorXd& time,
        const Eigen::DenseBase<Derived>& matrix,
        const std::string& filepath,
        const std::vector<std::string>& columnLabels = {}) {
    auto labels = columnLabels;
    if (!labels.empty()) {
        labels.insert(labels.begin(), "time");
    }

    Eigen::MatrixXd with_time(matrix.cols(), matrix.rows() + 1);
    with_time << time, matrix.transpose();

    write(with_time, filepath, labels);
}

} // namespace tropter

#endif // TROPTER_EIGENUTILITIES_H
