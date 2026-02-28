/* -------------------------------------------------------------------------- *
 *                        OpenSim: testScopeExit.h                            *
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

#include <OpenSim/Common/ScopeExit.h>

#include <catch2/catch_all.hpp>

using namespace OpenSim;

namespace
{
    /**
     * Helper class for exercising the move-constructor behavior that's
     * documented for `std::experimental::scope_exit<EF>`, to ensure that
     * OpenSim's rewrite of it is mostly forward-compatible.
     */
    template<bool IsNoexceptMoveConstructible, bool IsNoexceptCopyConstructible>
    class Callable final {
    public:
        explicit Callable(int* calls, int* copies, int* moves) :
            calls_{calls}, copies_{copies}, moves_{moves}
        {}
        Callable(Callable&& tmp) noexcept(IsNoexceptMoveConstructible) :
            calls_{tmp.calls_}, copies_{tmp.copies_}, moves_{tmp.moves_}
        {
            ++(*moves_);
        }
        Callable(const Callable& src) noexcept(IsNoexceptCopyConstructible) :
            calls_{src.calls_}, copies_{src.copies_}, moves_{src.moves_}
        {
            ++(*copies_);
        }
        ~Callable() noexcept = default;

        Callable& operator=(const Callable&) = delete;
        Callable& operator=(Callable&&) noexcept = delete;


        void operator()() const noexcept { ++(*calls_); }
    private:
        int* calls_;
        int* copies_;
        int* moves_;
    };
}


TEST_CASE("ScopeExit calls exit function on normal scope exit")
{
    bool called = false;
    {
        ScopeExit exit{[&called]() { called = true; }};
    }
    REQUIRE(called);
}

TEST_CASE("ScopeExit calls exit function when an exception is thrown")
{
    bool called = false;
    {
        try {
            ScopeExit exit{[&called]() { called = true; }};
            throw std::runtime_error{"throw something"};
        }
        catch (const std::exception&) {}  // ignore the exception
    }
    REQUIRE(called);
}

TEST_CASE("ScopeExit when move constructed only calls exit function once")
{
    int calls = 0;
    {
        ScopeExit first{[&calls]() { ++calls; }};
        ScopeExit second{std::move(first)};
    }
    REQUIRE(calls == 1);
}

TEST_CASE("ScopeExit when move constructed does not forget release of source")
{
    int calls = 0;
    {
        ScopeExit first{[&calls]() { ++calls; }};
        first.release();
        ScopeExit second{std::move(first)};
    }
    REQUIRE(calls == 0);
}

TEST_CASE("ScopeExit copies the function object if its move constructor is not noexcept")
{
    int calls = 0;
    int copies = 0;
    int moves = 0;
    {
        ScopeExit first{Callable<false, true>{&calls, &copies, &moves}};
        ScopeExit second{std::move(first)};
    }
    REQUIRE(calls == 1);   // Destruction of `second`
    REQUIRE(copies == 1);  // Copying via `ScopeExit::ScopeExit(ScopeExit&&)`
    REQUIRE(moves == 1);   // Initial construction
}

TEST_CASE("ScopeExit moves the function object if its move constructor is noexcept")
{
    int calls = 0;
    int copies = 0;
    int moves = 0;
    {
        ScopeExit first{Callable<true, true>{&calls, &copies, &moves}};
        ScopeExit second{std::move(first)};
    }
    REQUIRE(calls == 1);   // Destruction of `second`
    REQUIRE(copies == 0);
    REQUIRE(moves == 2);   // Initial construction and move construction of `ScopeExit`
}

TEST_CASE("ScopeExit release stops the exit function from being called on normal scope exit")
{
    bool called = false;
    {
        ScopeExit exit{[&called]() { called = true; }};
        exit.release();
    }
    REQUIRE(not called);
}

TEST_CASE("ScopeExit release stops the exit function from being called when an exception is thrown")
{
    bool called = false;
    {
        try {
            ScopeExit exit{[&called]() { called = true; }};
            exit.release();
            throw std::runtime_error{"throw something"};
        }
        catch (const std::exception&) {}  // ignore the exception
    }
    REQUIRE(not called);
}
