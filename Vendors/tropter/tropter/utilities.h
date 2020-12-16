#ifndef TROPTER_UTILITIES_H
#define TROPTER_UTILITIES_H
// ----------------------------------------------------------------------------
// tropter: utilities.h
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

#include <iostream>
#include <string>
#include <vector>

namespace tropter {
/// Format a string in the style of sprintf.
std::string format(const char* format, ...);

std::vector<double> linspace(double start, double end, int length);

/// This class stores the formatting of a stream and restores that format
/// when the StreamFormat is destructed.
/// This is useful when you want to make temporary changes to the formatting of
/// the stream.
///
/// @code
/// std::stringstream ss;
/// double x = 5.0;
/// ss << x << std::endl; // '5'
/// {
///   StreamFormat format(ss);
///   ss << std::scientific << std::setprecision(1) << x << std::endl; // '5.0e+00'
///   ss << x << std::endl; // '5.0e+00'
/// }
/// ss << x << std::endl; // '5'
/// @endcode
class StreamFormat {
public:
    StreamFormat(std::ostream& stream)
            : m_stream(stream), m_format_flags(stream.flags()) {
        m_format.copyfmt(stream);
    }
    ~StreamFormat() {
        m_stream.flags(m_format_flags);
        m_stream.copyfmt(m_format);
    }

private:
    std::ostream& m_stream;
    std::ios_base::fmtflags m_format_flags;
    std::ios m_format{nullptr};
}; // StreamFormat

} // namespace tropter

#endif // TROPTER_UTILITIES_H_
