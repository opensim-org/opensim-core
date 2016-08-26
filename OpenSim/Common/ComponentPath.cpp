/* -------------------------------------------------------------------------- *
 *                       OpenSim: ComponentPath.cpp                           *
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

#include "ComponentPath.h"

using namespace OpenSim;
using namespace std;

const char separator = '/';
const std::string invalidChars = "\\/*+";

ComponentPath::ComponentPath(const string path) :
    Path(path, separator, invalidChars)
{}

ComponentPath::ComponentPath(std::vector<std::string> pathVec, bool isAbsolute) :
    Path(pathVec, separator, invalidChars, isAbsolute)
{}

ComponentPath ComponentPath::getAbsolutePath(ComponentPath* otherPath)
{
    vector<string> absPathVec = getAbsolutePathVec(otherPath);
    return ComponentPath(absPathVec, true);

}

ComponentPath ComponentPath::getRelativePath(ComponentPath* otherPath)
{
    vector<string> relPathVec = getRelativePathVec(otherPath);
    return ComponentPath(relPathVec, false);
}

ComponentPath ComponentPath::getParentPath()
{
    vector<string> parentPathVec = getParentPathVec();
    return ComponentPath(parentPathVec, isAbsolute());
}

ComponentPath ComponentPath::getSubcomponent(size_t index)
{
    return ComponentPath(getPathElement(index));
}

ComponentPath ComponentPath::getLastSubcomponent() 
{
    return getSubcomponent(getPathLength() - 1);
}