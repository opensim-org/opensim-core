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
    ComponentPath absPath1("/a/b/c/d");
    ComponentPath absPath2("/a/b/e/f/g/h");
    ComponentPath absPath3("/a/b");

    ComponentPath relPath1("c/d");
    ComponentPath relPath2("e/f/g/h");

    ComponentPath emptyPath("");
    ComponentPath rootPath("/");

}

int main()
{
    SimTK_START_TEST("testPath");
        SimTK_SUBTEST(testComponentPath);
    SimTK_END_TEST();

    return 0;
}