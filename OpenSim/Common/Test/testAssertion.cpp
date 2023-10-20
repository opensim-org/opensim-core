/* ------------------------------------------------------------------------- *
*                           OpenSim:  testAssertion.cpp                      *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2017 Stanford University and the Authors                *
* Author(s): Frank C. Anderson                                               *
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

#include <OpenSim/Common/Assertion.h>

#include <OpenSim/Common/Exception.h>
#include <OpenSim/Common/Object.h>
#define CATCH_CONFIG_MAIN
#include <OpenSim/Auxiliary/catch/catch.hpp>

namespace TestAssertion {

    [[noreturn]] void SomeAssertingFunction() {
        OPENSIM_ASSERT_ALWAYS(false && "i-am-in-the-error-msg");
    }

    class SomeAssertingObject : public OpenSim::Object {
        OpenSim_DECLARE_CONCRETE_OBJECT(SomeAssertingObject, OpenSim::Object);
    public:
        SomeAssertingObject()
        {
            setName("name-of-the-object");
            OPENSIM_ASSERT_FRMOBJ(false && "and-i-am-also-in-the-error-msg");
        }
    };
}

TEST_CASE("OPENSIM_ASSERT_ALWAYS throws an OpenSim::Exception on failure")
{
    CHECK_THROWS_AS(OPENSIM_ASSERT_ALWAYS(false), OpenSim::Exception);
}

TEST_CASE("OPENSIM_ASSERT_ALWAYS exception contains expected information")
{
    // sorry: the line number in this source file that `OPENSIM_ASSERT_ALWAYS`
    // appears on is hard-coded here because automating that is difficult
    const char* lineNumberOfAssertionInThisFile = "34";

    try {
        TestAssertion::SomeAssertingFunction();
    } catch (const OpenSim::Exception& ex) {
        REQUIRE_THAT(ex.what(), Catch::Matchers::Contains("SomeAssertingFunction"));
        REQUIRE_THAT(ex.what(), Catch::Matchers::Contains("testAssertion.cpp"));
        REQUIRE_THAT(ex.what(), Catch::Matchers::Contains(lineNumberOfAssertionInThisFile));
        REQUIRE_THAT(ex.what(), Catch::Matchers::Contains("i-am-in-the-error-msg"));
    }
}

TEST_CASE("OPENSIM_ASSERT_FRMOBJ_ALWAYS exception contains expected information")
{
    // sorry: the line number in this source file that `OPENSIM_ASSERT_FRMOBJ`
    // appears on is hard-coded here because automating that is difficult
    const char* lineNumberOfAssertionInThisFile = "43";

    try {
        TestAssertion::SomeAssertingObject throwsOnConstruction{};
    } catch (const OpenSim::Exception& ex) {
        REQUIRE_THAT(ex.what(), Catch::Matchers::Contains("SomeAssertingObject"));
        REQUIRE_THAT(ex.what(), Catch::Matchers::Contains("testAssertion.cpp"));
        REQUIRE_THAT(ex.what(), Catch::Matchers::Contains(lineNumberOfAssertionInThisFile));
        REQUIRE_THAT(ex.what(), Catch::Matchers::Contains("name-of-the-object"));
        REQUIRE_THAT(ex.what(), Catch::Matchers::Contains("and-i-am-also-in-the-error-msg"));
    }
}
