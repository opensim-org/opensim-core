#ifndef OPENSIM_JOIN_SHIM_H
#define OPENSIM_JOIN_SHIM_H

/* -------------------------------------------------------------------------- *
 *                         OpenSim: JoinShim.h                                *
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

/// JoinShim is an internal header that provides a shim for C++23's
/// `std::views::join`. It should only be used internally by OpenSim's
/// library code (i.e. we can change/break this over time - beware).

#include <format>
#include <ranges>
#include <string_view>

namespace OpenSim::detail {

    /// A shim for C++23's `std::views::join`.
    template <std::ranges::input_range R>
    struct join_view {
        const R& range;
        std::string_view separator;
    };

    /// A shim for C++23's `std::views::join`.
    template <std::ranges::input_range R>
    constexpr auto join(const R& r, std::string_view sep)
    {
        return join_view<R>{r, sep};
    }
}

/// Formatter for `detail::join_view` shim (i.e. prints lists joined by delimiters).
template <std::ranges::input_range R>
struct std::formatter<OpenSim::detail::join_view<R>> {
    constexpr auto parse(std::format_parse_context& ctx) {
        auto it = ctx.begin();
        if (it != ctx.end() && *it != '}') {
            throw std::format_error("invalid format for join_view");
        }
        return it;
    }

    template <class FormatContext>
    auto format(const OpenSim::detail::join_view<R>& value, FormatContext& ctx) const {
        auto out = ctx.out();

        bool first = true;
        for (const auto& elem : value.range) {
            if (not first) {
                out = std::ranges::copy(value.separator, out).out;
            }
            first = false;
            out = std::format_to(out, "{}", elem);
        }

        return out;
    }
};

#endif // OPENSIM_JOIN_SHIM_H
