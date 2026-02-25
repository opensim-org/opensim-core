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

TEST_CASE("ScopeExit calls exit function on normal scope exit")
{
    bool called = false;
    {
        ScopeExit exit{[&called]() { called = true; }};
    }
    ASSERT(called == true);
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
    ASSERT(called == true);
}

TEST_CASE("ScopeExit release stops the exit function from being called on normal scope exit")
{
    bool called = false;
    {
        ScopeExit exit{[&called]() { called = true; }};
        exit.release();
    }
    ASSERT(called == false);
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
    ASSERT(called == false);
}
