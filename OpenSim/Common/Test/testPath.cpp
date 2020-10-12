/* ------------------------------------------------------------------------- *
*                        OpenSim:  testPath.cpp                              *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2017 Stanford University and the Authors                *
* Author(s): Carmichael Ong                                                  *
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

#include <OpenSim/Common/ComponentPath.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

/* The purpose of this test is strictly to check that classes derived from
 * the Path class work outside of the objects/components they are meant to 
 * service (i.e. check that the path logic works).
 */

using namespace OpenSim;
using namespace std;

void testComponentPath() {
    /* Test that creating ComponentPath with paths that should not be cleaned up */

    using CP = ComponentPath;
    using std::string;

    // Absolute paths
    ASSERT(CP{"/a/b/c/d"}.toString() == "/a/b/c/d");
    ASSERT(CP{"/a/b/e/f/g/h"}.toString() == "/a/b/e/f/g/h");
    ASSERT(CP{"/a/b"}.toString() == "/a/b");

    // Relative paths
    //     note: relative path can start with ".."
    ASSERT(CP{"c/d"}.toString() == "c/d");
    ASSERT(CP{"e/f/g/h"}.toString() == "e/f/g/h");
    ASSERT(CP{"../../../../c/d"}.toString() == "../../../../c/d");

    // Empty paths
    ASSERT(CP{""}.toString() == "");
    ASSERT(CP{"/"}.toString() == "/");

    /* Test the equality operator */
    ASSERT(CP{"/a/b"} == CP{"/a/b"});
    ASSERT(CP{"/a/b"} != CP{"/a/b/c/d"});
    ASSERT(CP{"/c/d"} != CP{"c/d"});

    // test vector ctor
    ASSERT_THROW(Exception, CP(std::vector<std::string>{"in+valid"}, true));
    ASSERT_THROW(Exception, CP(std::vector<std::string>{"in+valid"}, false));
    ASSERT_THROW(Exception, CP(std::vector<std::string>{"a", "in+valid"}, true));
    ASSERT_THROW(Exception, CP(std::vector<std::string>{"b", "in+valid"}, false));

    /* Test formAbsolutePath(). Test a variety of paths. The argument 
     * in formAbsolutePath() must be an absolute path itself. */
    // Test without any ".."
    ASSERT(CP{"c/d"}.formAbsolutePath(CP{"/a/b"}).toString() == "/a/b/c/d");
    ASSERT(CP{"e/f/g/h"}.formAbsolutePath(CP{"/a/b"}).toString() == "/a/b/e/f/g/h");
    // Any absolute path should return itself
    ASSERT(CP{"/a/b/c/d"}.formAbsolutePath(CP{"/a/b/e/f/g/h"}) == CP{"/a/b/c/d"});
    // Test if this works from root.
    ASSERT(CP{"c/d"}.formAbsolutePath(CP{"/"}).toString() == "/c/d");
    // Test with lots of ".."
    ASSERT(CP{"../../../../c/d"}.formAbsolutePath(CP{"/a/b/e/f/g/h"}) == CP{"/a/b/c/d"});
    // argument can't be a relative path
    ASSERT_THROW(Exception, CP{"c/d"}.formAbsolutePath(CP{"e/f/g/h"}));

    /* Test formRelativePath(). Both paths must be absolute */
    // Test path that go up and back down the tree
    {
        struct TestCase {
            std::string from;
            std::string to;
            std::string expected;
        };

        TestCase testCases[] = {
            { "/a/b/e/f/g/h", "/a/b/c/d", "../../../../c/d" },
            { "/a/b/c/d", "/a/b/e/f/g/h", "../../e/f/g/h" },
            // Test path that just goes down a tree
            { "/a/b", "/a/b/c/d", "c/d" },
            // Test path that only goes up the tree
            { "/a/b/e/f/g/h", "/a/b", "../../../.." },
            // An example that failed post-merge
            //     see https://github.com/opensim-org/opensim-core/pull/2805
            { "/contact", "/contact_point", "../contact_point" },
            // other potentially-failing examples, for good measure
            { "/a/b", "/c/d", "../../c/d" },
            { "/a/b/c/", "/e/f/g", "../../../e/f/g" },
            // logically, the relative path should be "." for this. However,
            // the existing implementation returns this value, so it's kept
            // for backwards-compat
            { "/", "/", "" },
            { "/a", "/a", "" },
            { "/a/b", "/a/b", "" },
        };

        for (const TestCase& tc : testCases) {
            const ComponentPath ans = CP{tc.to}.formRelativePath(CP{tc.from});
            if (ans.toString() != tc.expected) {
                std::stringstream ss;
                ss << "ComponentPath::formRelativePath produced an invalid output" << std::endl;
                ss << "      from = " << tc.from << std::endl;
                ss << "        to = " << tc.to << std::endl;
                ss << "  expected = " << tc.expected << std::endl;
                ss << "       got = " << ans.toString();
                throw std::runtime_error{ss.str()};
            }
        }
    }

    // Throw exceptions if either or both paths are not absolute
    ASSERT_THROW(Exception, CP{"c/d"}.formRelativePath(CP{"/a/b/e/f/g/h"}));
    ASSERT_THROW(Exception, CP{"/a/b/e/f/g/h"}.formRelativePath(CP{"e/f/g/h"}));
    ASSERT_THROW(Exception, CP{"e/f/g/h"}.formRelativePath(CP{"c/d"}));

    /* Test paths with "." and ".." */
    // Remove all ".", clean up ".." and ignore "/" at the end
    ASSERT(CP{"/a/././b/c/..//d/.././"}.toString() == CP{"/a/b"}.toString());
    // Test ".." at the end of a path
    ASSERT(CP{"/a/b/c/d/../.."} == CP{"/a/b"});
    // Test ".." at the beginning of an absolute path (should throw exception)
    ASSERT_THROW(Exception, CP{"/../b/c/d"});
    // Test if there are so many ".." that it will end up at the front of
    // an absolute path, and thus should fail like the one above
    ASSERT_THROW(Exception, CP{"/a/../../c/d"});

    /* Test that invalid characters will throw an exception. This should throw
     * exceptions when "\\", "+", or "*" appear in the string that initializes
     * the ComponentPath. Although "/" is also an invalid character, it should
     * NOT throw an exception here since it is read in as a separator. */
    ASSERT_THROW(Exception, CP{"/a/b\\/c/"});
    ASSERT_THROW(Exception, CP{"/a+b+c/"});
    ASSERT_THROW(Exception, CP{"/abc*/def/g/"});

    /* Test the pushBack() function */
    {
        ComponentPath path1{"/a/b"};
        // can't add multiple levels at once
        ASSERT_THROW(Exception, path1.pushBack("c/d"));

        // add the levels separately
        path1.pushBack("c");
        path1.pushBack("d");
        ASSERT(path1 == CP{"/a/b/c/d"});

        /* Test invalid characters in pushBack(). Unlike the invalid
         * character test above, "/" should be considered invalid since
         * pushBack() cannot add multiple levels at once. It is also
         * invalid to attempt to add an empty pathElement.
         */
        ASSERT_THROW(Exception, path1.pushBack("a\\b"));
        ASSERT_THROW(Exception, path1.pushBack("a+b*"));
        ASSERT_THROW(Exception, path1.pushBack("test/this"));
        ASSERT_THROW(Exception, path1.pushBack(""));

        CP cp2{""};
        cp2.pushBack("a");
        ASSERT(cp2.toString() == "a");

        CP cp3{"/"};
        cp3.pushBack("a");
        ASSERT(cp3.toString() == "/a");
    }

    /* Test functions for getting certain parts of ComponentPath. */
    // Create a path "/zero/one/two/three/four"
    std::vector<std::string> levels = {"zero", "one", "two", "three", "four"};
    ComponentPath numberedAbsPath(levels, true);
    // Parent path is "/zero/one/two/three"
    std::string numberedAbsPathParentStr = "/zero/one/two/three";
    ComponentPath numberedAbsPathParent(numberedAbsPathParentStr);
    // Test if getParentPath() returns correct ComponentPath object
    ASSERT(numberedAbsPath.getParentPath() == numberedAbsPathParent);
    // Test if getParentPathStr() returns correct string
    ASSERT(numberedAbsPath.getParentPathString() == numberedAbsPathParentStr);

    // test number of path levels behaves sanely
    {
        static const std::pair<std::string, size_t> expectedNumPathLevels[] = {
                {"", 0},
                {"/", 0},
                {"a", 1},
                {"/a", 1},
                {"/a/", 1},
                {"a/b", 2},
                {"/a/b", 2},
                {"/a/b/", 2},
                {"/a/b/c", 3},
                {"../", 1},
                {"a/..", 0},
                {"/a/..", 0},
        };

        for (auto p : expectedNumPathLevels) {
            ASSERT(CP{p.first}.getNumPathLevels() == p.second);
        }
    }

    // Loop through all levels of the subtree and see if names match
    for (size_t ind = 0; ind < levels.size(); ++ind) {
        ASSERT(numberedAbsPath.getSubcomponentNameAtLevel(ind) == levels[ind]);
    }
    // Test getComponentName()
    ASSERT(numberedAbsPath.getComponentName() == levels[levels.size()-1]);
    ASSERT(CP{""}.getComponentName() == ""); // empty ComponentPath should return empty string

    // Do the same as above but with a relative path instead
    ComponentPath numberedRelPath(levels, false);
    std::string numberedRelPathParentStr = "zero/one/two/three";
    ComponentPath numberedRelPathParent(numberedRelPathParentStr);
    ASSERT(numberedRelPath.getParentPath() == numberedRelPathParent);
    ASSERT(numberedRelPath.getParentPathString() == numberedRelPathParentStr);
    for (size_t ind = 0; ind < levels.size(); ++ind) {
        ASSERT(numberedRelPath.getSubcomponentNameAtLevel(ind) == levels[ind]);
    }

    // ensure isAbsolute is sane for vector inputs
    ASSERT(CP{std::vector<std::string>{}, true}.isAbsolute());
    ASSERT(CP{std::vector<std::string>{""}, true}.isAbsolute());
    ASSERT(CP{std::vector<std::string>{"a", "b"}, true}.isAbsolute());

    // general tests to ensure it normalizes a variety of paths correctly
    {
        static const std::pair<std::string, std::string> shouldPass[] = {
            { "",                "" },
            { "/",               "/" },
            { "a/b/c",           "a/b/c" },
            { "a/..",            "" },
            { "a/../",           "" },
            { "a/../c",          "c" },
            { "a/../c/",         "c" },
            { "/a/../c",         "/c" },
            { "/a/b/../../c",    "/c" },
            { "a/b/../../c",     "c" },
            { "/./././c",        "/c" },
            { "./././c",         "c" },
            { "./",              "" },
            { ".",               "" },
            { "./.",             "" },
            { "./a/.",           "a" },
            { "./a/./",          "a" },
            { "a//b/.///",       "a/b" },
            { "///",             "/" },
            { ".///",            "" },
            { "a///b",           "a/b" },
            { "a/b/c/",          "a/b/c" },
            { "a/b/c//",         "a/b/c" },
            { "../a/b",          "../a/b" },
            { "../a/b/",         "../a/b" },
            { "./../a/../",      ".." },
            { "/a/b/c/d",        "/a/b/c/d" },
            { "/a/b/e/f/g/h",    "/a/b/e/f/g/h" },
            { "/a/b",            "/a/b" },
            { "c/d",             "c/d" },
            { "e/f/g/h",         "e/f/g/h" },
            { "/a/././b/c/..//d/.././", "/a/b" },
            { "../../../../c/d", "../../../../c/d" },
            { "/a/b/c/d/../..", "/a/b" },
        };

        for (const auto& tc : shouldPass) {
            std::cerr << "input = " << tc.first << std::endl;
            std::string output = ComponentPath{tc.first}.toString();
            if (output != tc.second) {
                std::stringstream msg;
                msg << "ComponentPath::resolveRelativeElements: invalid output:" << std::endl;
                msg << "              input = " << tc.first << std::endl;
                msg << "             output = " << output << std::endl;
                msg << "    expected output = " << tc.second << std::endl;
                throw std::runtime_error{msg.str()};
            }
        }

        static const std::string shouldThrow[] = {
            "a/../..",
            "./a/../..",
            "/..",
            "/./..",
            "/a/../..",
            "/./../",
            "/a/./.././..",
            "/../b/c/d",
            "/a/../../c/d",

            // contain invalid chars
            "foo\\bar",
            "a/foo\\bar/c",
            "foo*bar",
            "a/foo*bar*/c",
            "foo+bar",
            "a/foo+bar",
            "foo\tbar",
            "a/b/c/foo\tbar/d",
            "foo\nbar",
            "/a/foo\nbar",
        };

        for (const std::string& tc : shouldThrow) {
            std::string maybeError;
            try {
                std::string p = ComponentPath{tc}.toString();
                std::stringstream msg;
                msg << tc << ": did not throw an exception: instead, it output: " << p;
                maybeError = msg.str();
            } catch (...) {
                // good
            }
            if (!maybeError.empty()) {
                throw std::runtime_error{maybeError};
            }
        }
    }
}

int main()
{
    SimTK_START_TEST("testPath");
        SimTK_SUBTEST(testComponentPath);
    SimTK_END_TEST();

    return 0;
}
