/* ------------------------------------------------------------------------- *
*                        OpenSim:  testPath.cpp                              *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2016 Stanford University and the Authors                *
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

/* The purpose of this test is strictly to check that the Path functions work
 * outside of the context of the objects/components they are meant to service. 
 */

using namespace OpenSim;
using namespace std;

void testComponentPath() {
    // Test that input matches printing out
    string absStr1 = "/a/b/c/d";
    string absStr2 = "/a/b/e/f/g/h";
    string absStr3 = "/a/b";
    ComponentPath absPath1(absStr1);
    ComponentPath absPath2(absStr2);
    ComponentPath absPath3(absStr3);
    ASSERT(absPath1.getString() == absStr1);
    ASSERT(absPath2.getString() == absStr2);
    ASSERT(absPath3.getString() == absStr3);

    string relStr1 = "c/d";
    string relStr2 = "e/f/g/h";
    string relStr3 = "../../../../c/d";
    ComponentPath relPath1(relStr1);
    ComponentPath relPath2(relStr2);
    ComponentPath relPath3(relStr3);
    ASSERT(relPath1.getString() == relStr1);
    ASSERT(relPath2.getString() == relStr2);
    ASSERT(relPath3.getString() == relStr3);

    string emptyStr = "";
    string rootStr = "/";
    ComponentPath emptyPath(emptyStr);
    ComponentPath rootPath(rootStr);
    ASSERT(emptyPath.getString() == emptyStr);
    ASSERT(rootPath.getString() == rootStr);

    // Test getAbsolutePath()
    ASSERT(relPath1.getAbsolutePath(&absPath3).getString() == absStr1);
    ASSERT(relPath2.getAbsolutePath(&absPath3).getString() == absStr2);
    ASSERT(absPath1.getAbsolutePath(&absPath2).getString() == absStr1);
    ASSERT(relPath1.getAbsolutePath(&rootPath).getString() == "/c/d");
    ASSERT(relPath3.getAbsolutePath(&absPath2).getString() == absPath1.getString());
    ASSERT_THROW(Exception, relPath1.getAbsolutePath(&relPath2));
    
    // Test findRelativePath()
    ASSERT(absPath1.getRelativePath(&absPath2).getString() == "../../../../c/d");
    ASSERT(absPath2.getRelativePath(&absPath1).getString() == "../../e/f/g/h");
    ASSERT(absPath1.getRelativePath(&absPath3).getString() == "c/d");
    ASSERT(absPath3.getRelativePath(&absPath2).getString() == "../../../..");
    ASSERT_THROW(Exception, relPath1.getRelativePath(&absPath2));
    ASSERT_THROW(Exception, absPath2.getRelativePath(&relPath2));
    ASSERT_THROW(Exception, relPath2.getRelativePath(&relPath1));

    // Test some odd paths
    string oddStr1 = "/a/././b/c/..//d/.././";
    ComponentPath oddPath1(oddStr1);
    ASSERT(oddPath1.getString() == absPath3.getString());
    string oddStr2 = "/a/b/c/d/../..";
    ComponentPath oddPath2(oddStr2);
    ASSERT(oddPath2.getString() == absPath3.getString());
    string oddStr3 = "/../b/c/d";
    ASSERT_THROW(Exception, ComponentPath oddPath3(oddStr3));
    string oddStr4 = "/a/../../c/d";
    ASSERT_THROW(Exception, ComponentPath oddPath4(oddStr4));

    string badChar1 = "/a/b\\/c/";
    string badChar2 = "/a+b+c/";
    string badChar3 = "/abc*/def/g/";
    ASSERT_THROW(Exception, ComponentPath badPath1(badChar1));
    ASSERT_THROW(Exception, ComponentPath badPath2(badChar2));
    ASSERT_THROW(Exception, ComponentPath badPath3(badChar3));

    // Test inserts and erase
    string str1 = "/a/b/d";
    ComponentPath path1(str1);
    path1.insertPathElement(2, "c");
    ASSERT(path1.getString() == absPath1.getString());
    path1.erasePathElement(2);
    path1.erasePathElement(2);
    ASSERT(path1.getString() == absPath3.getString());
    ASSERT_THROW(Exception, path1.appendPathElement("e/f/g/h"));
    ASSERT_THROW(Exception, path1.insertPathElement(2, "e/f/g/h"));
    path1.appendPathElement("e");
    path1.appendPathElement("f");
    path1.appendPathElement("g");
    path1.appendPathElement("h");
    ASSERT(path1.getString() == absPath2.getString());
    path1.insertPathElement(0, "..");
    ASSERT_THROW(Exception, path1.cleanPath());
    path1.insertPathElement(0, "z");
    path1.cleanPath();
    ASSERT(path1.getString() == absPath2.getString());

    ASSERT_THROW(Exception, path1.appendPathElement("a\\b"));
    ASSERT_THROW(Exception, path1.insertPathElement(2, "a+b*"));
}

int main()
{
    SimTK_START_TEST("testPath");
        SimTK_SUBTEST(testComponentPath);
    SimTK_END_TEST();

    return 0;
}