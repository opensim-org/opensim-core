#ifndef OPENSIM_SIMBODY_FORMATTER_SHIMS_H_
#define OPENSIM_SIMBODY_FORMATTER_SHIMS_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim: FormatterShims.h                          *
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

/// SimbodyFormatterShim is an internal header that provides `std::formatter`
/// support for simbody's. It should only be used internally by OpenSim's
/// library code (i.e. we can change/break this over time - beware).

#ifndef SWIG

#include <OpenSim/Common/Detail/OstreamFormatterShim.h>

#include <SimTKcommon/SmallMatrix.h>             // for Vec3
#include <SimTKcommon/internal/BigMatrix.h>      // for Vector
#include <SimTKcommon/internal/MassProperties.h> // for Inertia
#include <SimTKcommon/internal/Rotation.h>       // for Rotation
#include <SimTKcommon/internal/String.h>         // for String
#include <simbody/internal/common.h>             // for ConstraintIndex

#include <format>

/// A formatter for `SimTK::Vec`, where the format specifier is applied
/// to each of its elements.
template<int M, class ELT, int STRIDE>
struct std::formatter<SimTK::Vec<M, ELT, STRIDE>> {
    constexpr auto parse(std::format_parse_context& ctx) {
        return _inner.parse(ctx);
    }

    template<class FormatContext>
    auto format(const SimTK::Vec<M, ELT, STRIDE>& v, FormatContext& ctx) const -> decltype(ctx.out()) {
        // This implementation should match upstream's:
        //     `SimTK::operator<<(std::ostream&, SimTK::Vec<M, ELT, STRIDE>)`

        auto out = ctx.out();
        out = std::format_to(out, "~[");
        if constexpr (M > 0) {
            out = _inner.format(v[0], ctx);
            for (int i = 1; i < M; ++i) {
                out = std::format_to(out, ",");
                out = _inner.format(v[i], ctx);
            }
        }
        out = std::format_to(out, "]");
        return out;
    }
private:
    std::formatter<ELT> _inner;
};
template <> struct std::formatter<SimTK::String> : OpenSim::detail::ostream_formatter {};
template <> struct std::formatter<SimTK::Vector> : OpenSim::detail::ostream_formatter {};
template <> struct std::formatter<SimTK::Rotation> : OpenSim::detail::ostream_formatter {};
template <> struct std::formatter<SimTK::Inertia> : OpenSim::detail::ostream_formatter {};
template <> struct std::formatter<SimTK::ConstraintIndex> : OpenSim::detail::ostream_formatter {};

#endif  // ifndef SWIG
#endif  // ifndef OPENSIM_SIMBODY_FORMATTER_SHIMS_H_
