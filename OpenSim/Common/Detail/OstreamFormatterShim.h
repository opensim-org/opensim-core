#ifndef OPENSIM_OSTREAM_FORMATTER_SHIM_H_
#define OPENSIM_OSTREAM_FORMATTER_SHIM_H_
/* -------------------------------------------------------------------------- *
 *                      OpenSim: OstreamFormatterShim.h                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2026 Stanford University and the Authors                *
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

/// OstreamFormatterShim is an internal header that provides a shim for C++23's
/// `std::ostream_formatter`. It should only be used internally by OpenSim's
/// library code (i.e. we can change/break this over time - beware).

#ifndef SWIG

#include <format>
#include <sstream>

namespace OpenSim::detail {

    /// A temporary shim for C++23's `std::ostream_formatter`.
    struct ostream_formatter {
        // Parses the format specifier (e.g., inside the {}).
        // Since ostreams don't support std::format specifiers (like {:02d}),
        // this ensures the format string is empty.
        constexpr auto parse(std::format_parse_context& ctx) {
            auto it = ctx.begin();
            if (it != ctx.end() && *it != '}') {
                throw std::format_error("ostream_formatter does not support format specifiers.");
            }
            return it;
        }

        // Formats the value by dumping it into a `std::stringstream`,
        // then piping that string to the format context.
        template <typename T, typename FormatContext>
        auto format(const T& value, FormatContext& ctx) const {
            std::ostringstream oss;
            oss << value;
            return std::format_to(ctx.out(), "{}", oss.str());
        }
    };
}

#endif  // ifndef SWIG
#endif  // ifndef OPENSIM_OSTREAM_FORMATTER_SHIM_H_
