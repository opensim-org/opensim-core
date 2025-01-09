// ----------------------------------------------------------------------------
// tropter: utilities.cpp
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

#include "utilities.h"

#include <cstdarg>
#include <cstdio>
#include <memory>
#include <Eigen/Dense>

std::string tropter::format(const char* format, ...) {
    // Get buffer size.
    va_list args;
    va_start(args, format);
    int bufsize = vsnprintf(nullptr, 0, format, args) + 1; // +1 for '\0'
    va_end(args);

    // Create formatted string.
    std::unique_ptr<char[]> buf(new char[bufsize]);
    va_start(args, format);
    vsnprintf(buf.get(), bufsize, format, args);
    va_end(args);
    return std::string(buf.get());
}

std::vector<double> tropter::linspace(double start, double end, int length) {
    Eigen::VectorXd tmp = Eigen::VectorXd::LinSpaced(length, start, end);
    std::vector<double> ret(tmp.data(), tmp.data() + length);
    return ret;
}
