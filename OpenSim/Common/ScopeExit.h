#ifndef OPENSIM_SCOPE_EXIT_H_
#define OPENSIM_SCOPE_EXIT_H_

/* -------------------------------------------------------------------------- *
 *                          OpenSim: ScopeExit.h                              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2026 Stanford University and the Authors                *
 * Author(s): Adam Kewley                                                     *
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

#include <concepts>
#include <utility>

namespace OpenSim
{
    /**
     * A general-purpose scope guard intended to call its exit function when
     * a scope is exited - either normally, or via an exception.
     *
     * This utility class is effectively an OpenSim rewrite of `std::experimental::scope_exit`,
     * which is documented here: https://en.cppreference.com/w/cpp/experimental/scope_exit.html
     */
    template<std::invocable EF>
    class [[nodiscard]] ScopeExit final {
    public:
        /**
         * Constructs a `ScopeExit` from a function or function object.
         */
        template<typename Fn>
        requires (std::is_nothrow_constructible_v<EF, Fn>)
        requires(std::is_nothrow_invocable_v<EF>)
        explicit ScopeExit(Fn&& fn) : exit_function_{std::forward<Fn>(fn)} {}

        ScopeExit(const ScopeExit&) = delete;
        ScopeExit(ScopeExit&&) noexcept = delete;
        ScopeExit& operator=(const ScopeExit&) = delete;
        ScopeExit& operator=(ScopeExit&&) noexcept = delete;
        ~ScopeExit() noexcept(noexcept(exit_function_()))
        {
            if (is_active_) {
                exit_function_();
            }
        }

        /**
         * Makes the `ScopeExit` inactive, meaning it will will not call its
         * exit function on destruction.
         *
         * Once a `ScopeExit` is inactive, it cannot become active again.
         */
        void release() noexcept { is_active_ = false; }

        private:
            EF exit_function_;
            bool is_active_ = true;
    };

    /**
     * One template deduction guide permits the deduction of an argument of function
     * or function object type.
     *
     * The argument (after function-to-pointer decay, if any) is copied or moved into
     * the constructed scope_exit.
     */
    template<typename EF>
    ScopeExit(EF) -> ScopeExit<EF>;
}

#endif  // #ifndef OPENSIM_SCOPE_EXIT_H_