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
    ComponentPath relPath1(relStr1);
    ComponentPath relPath2(relStr2);
    ASSERT(relPath1.getString() == relStr1);
    ASSERT(relPath2.getString() == relStr2);

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
    ASSERT_THROW(Exception, relPath1.getAbsolutePath(&relPath2));
    
    // Test findRelativePath()
    ASSERT(absPath1.findRelativePath(&absPath2).getString() == "../../e/f/g/h");
    ASSERT_THROW(Exception, relPath1.findRelativePath(&absPath2));
    ASSERT_THROW(Exception, absPath2.findRelativePath(&relPath2));
    ASSERT_THROW(Exception, relPath2.findRelativePath(&relPath1));

    // Test some odd paths
    string oddStr1 = "/a/././b/c/..//d/.././";
    ComponentPath oddPath1(oddStr1);
    ASSERT(oddPath1.getString() == absPath3.getString());

    string oddStr2 = "/a/b/c/d/../..";
    ComponentPath oddPath2(oddStr2);
    ASSERT(oddPath2.getString() == absPath3.getString());

    string oddStr3 = "/../b/c/d";
    //ASSERT_THROW(Exception, ComponentPath(oddStr3));

    


}

int main()
{
    SimTK_START_TEST("testPath");
        SimTK_SUBTEST(testComponentPath);
    SimTK_END_TEST();

    return 0;
}