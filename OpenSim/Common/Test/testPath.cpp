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
    // Absolute paths
    string absStr1 = "/a/b/c/d";
    string absStr2 = "/a/b/e/f/g/h";
    string absStr3 = "/a/b";
    ComponentPath absPath1(absStr1);
    ComponentPath absPath2(absStr2);
    ComponentPath absPath3(absStr3);
    ASSERT(absPath1.toString() == absStr1);
    ASSERT(absPath2.toString() == absStr2);
    ASSERT(absPath3.toString() == absStr3);

    // Relative paths
    string relStr1 = "c/d";
    string relStr2 = "e/f/g/h";
    string relStr3 = "../../../../c/d"; // relative path can start with ".."
    ComponentPath relPath1(relStr1);
    ComponentPath relPath2(relStr2);
    ComponentPath relPath3(relStr3);
    ASSERT(relPath1.toString() == relStr1);
    ASSERT(relPath2.toString() == relStr2);
    ASSERT(relPath3.toString() == relStr3);

    // Paths that will have an empty vector of strings
    string emptyStr = "";
    string rootStr = "/";
    ComponentPath emptyPath(emptyStr);
    ComponentPath rootPath(rootStr);
    ASSERT(emptyPath.toString() == emptyStr);
    ASSERT(rootPath.toString() == rootStr);


    /* Test the equality operator */
    ComponentPath absPath4("/a/b");
    ASSERT(absPath4 == absPath3); // exactly the same
    ASSERT(absPath4 != absPath1); // wrong absolute path
    ComponentPath absPath5("/c/d");
    ASSERT(absPath5 != relPath1); // wrong absolute/relative comparison

    /* Test formAbsolutePath(). Test a variety of paths. The argument 
     * in formAbsolutePath() must be an absolute path itself. */
    // Test without any ".."
    ASSERT(relPath1.formAbsolutePath(absPath3).toString() == absStr1);
    ASSERT(relPath2.formAbsolutePath(absPath3).toString() == absStr2);
    // Any absolute path should return itself
    ASSERT(absPath1.formAbsolutePath(absPath2) == absPath1);
    // Test if this works from root.
    ASSERT(relPath1.formAbsolutePath(rootPath).toString() == "/c/d");
    // Test with lots of ".."
    ASSERT(relPath3.formAbsolutePath(absPath2) == absPath1);
    // argument can't be a relative path
    ASSERT_THROW(Exception, relPath1.formAbsolutePath(relPath2));
    
    /* Test formRelativePath(). Both paths must be absolute */
    // Test path that go up and back down the tree
    ASSERT(absPath1.formRelativePath(absPath2).toString() == "../../../../c/d");
    ASSERT(absPath2.formRelativePath(absPath1).toString() == "../../e/f/g/h");
    // Test path that just goes down a tree
    ASSERT(absPath1.formRelativePath(absPath3).toString() == "c/d");
    // Test path that only goes up the tree
    ASSERT(absPath3.formRelativePath(absPath2).toString() == "../../../..");
    // Throw exceptions if either or both paths are not absolute
    ASSERT_THROW(Exception, relPath1.formRelativePath(absPath2));
    ASSERT_THROW(Exception, absPath2.formRelativePath(relPath2));
    ASSERT_THROW(Exception, relPath2.formRelativePath(relPath1));


    /* Test paths with "." and ".." */
    // Remove all ".", clean up ".." and ignore "/" at the end
    string oddStr1 = "/a/././b/c/..//d/.././";
    ComponentPath oddPath1(oddStr1);
    ASSERT(oddPath1.toString() == absPath3.toString());
    // Test ".." at the end of a path
    string oddStr2 = "/a/b/c/d/../..";
    ComponentPath oddPath2(oddStr2);
    ASSERT(oddPath2.toString() == absPath3.toString());
    // Test ".." at the beginning of an absolute path (should throw exception)
    string oddStr3 = "/../b/c/d";
    ASSERT_THROW(Exception, ComponentPath oddPath3(oddStr3));
    // Test if there are so many ".." that it will end up at the front of
    // an absolute path, and thus should fail like the one above
    string oddStr4 = "/a/../../c/d";
    ASSERT_THROW(Exception, ComponentPath oddPath4(oddStr4));

    /* Test that invalid characters will throw an exception. This should throw
     * exceptions when "\\", "+", or "*" appear in the string that initializes
     * the ComponentPath. Although "/" is also an invalid character, it should
     * NOT throw an exception here since it is read in as a separator. */
    string badChar1 = "/a/b\\/c/";
    string badChar2 = "/a+b+c/";
    string badChar3 = "/abc*/def/g/";
    ASSERT_THROW(Exception, ComponentPath badPath1(badChar1));
    ASSERT_THROW(Exception, ComponentPath badPath2(badChar2));
    ASSERT_THROW(Exception, ComponentPath badPath3(badChar3));

    /* Test the pushBack() function */
    string str1 = "/a/b";
    ComponentPath path1(str1);
    // can't add multiple levels at once
    ASSERT_THROW(Exception, path1.pushBack("c/d")); 
    // add the levels separately
    path1.pushBack("c");
    path1.pushBack("d");
    // now it should match absPath1 ("/a/b/c/d")
    ASSERT(path1 == absPath1);

    /* Test invalid characters in pushBack(). Unlike the invalid
     * character test above, "/" should be considered invalid since
     * pushBack() cannot add multiple levels at once. It is also 
     * invalid to attempt to add an empty pathElement. */
    ASSERT_THROW(Exception, path1.pushBack("a\\b"));
    ASSERT_THROW(Exception, path1.pushBack("a+b*"));
    ASSERT_THROW(Exception, path1.pushBack("test/this"));
    ASSERT_THROW(Exception, path1.pushBack(""));

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
    // Loop through all levels of the subtree and see if names match
    for (size_t ind = 0; ind < levels.size(); ++ind) {
        ASSERT(numberedAbsPath.getSubcomponentNameAtLevel(ind) == levels[ind]);
    }
    // Test getComponentName()
    ASSERT(numberedAbsPath.getComponentName() == levels[levels.size()-1]);
    ASSERT(emptyPath.getComponentName() == ""); // empty ComponentPath should return empty string

    // Do the same as above but with a relative path instead
    ComponentPath numberedRelPath(levels, false);
    std::string numberedRelPathParentStr = "zero/one/two/three";
    ComponentPath numberedRelPathParent(numberedRelPathParentStr);
    ASSERT(numberedRelPath.getParentPath() == numberedRelPathParent);
    ASSERT(numberedRelPath.getParentPathString() == numberedRelPathParentStr);
    for (size_t ind = 0; ind < levels.size(); ++ind) {
        ASSERT(numberedRelPath.getSubcomponentNameAtLevel(ind) == levels[ind]);
    }

    // ComponentPath::split
    {
        auto testSplit = [](
                std::string input,
                std::string expectedHead,
                std::string expectedTail) {
            auto p = ComponentPath::split(input);

            if (p.first != expectedHead || p.second != expectedTail) {
                std::stringstream msg;
                msg << "invalid output for input '" << input << "':" << std::endl;
                msg << "    expected head = " << expectedHead << std::endl;
                msg << "      actual head = " << p.first << std::endl;
                msg << "    expected tail = " << expectedTail << std::endl;
                msg << "      actual tail = " << p.second << std::endl;
                throw std::runtime_error{msg.str()};
            }
        };

        testSplit("some/path/to/var", "some/path/to", "var");
        testSplit("/var", "/", "var");
        testSplit("var", "", "var");
        testSplit("", "", "");

        // '.' and '..' elements are not automatically handled by this function.
        // If needed, a separate function (e.g. 'abspath') should be used to
        // handle de-relativizing the elements
        testSplit("../var", "var");
    }
}

int main()
{
    SimTK_START_TEST("testPath");
        SimTK_SUBTEST(testComponentPath);
    SimTK_END_TEST();

    return 0;
}
