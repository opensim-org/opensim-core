/* -------------------------------------------------------------------------- *
 *                       OpenSim: ComponentPath.cpp                           *
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

#include "ComponentPath.h"

using namespace OpenSim;
using namespace std;

// Set static member variables.
const char ComponentPath::separator = '/';
const std::string ComponentPath::invalidChars = "\\/*+ \t\n";

std::pair<std::string, std::string> ComponentPath::split(std::string path) {
    return {"todo", "todo"};
}

ComponentPath::ComponentPath() :
    Path(getSeparator(), getInvalidChars())
{}

ComponentPath::ComponentPath(const string& path) :
    Path(path, getSeparator(), getInvalidChars())
{}

ComponentPath::ComponentPath(const std::vector<std::string>& pathVec, bool isAbsolute) :
    Path(pathVec, getSeparator(), getInvalidChars(), isAbsolute)
{}

ComponentPath ComponentPath::formAbsolutePath(const ComponentPath& otherPath) const
{
    vector<string> absPathVec = formAbsolutePathVec(otherPath);
    return ComponentPath(absPathVec, true);

}

ComponentPath ComponentPath::formRelativePath(const ComponentPath& otherPath) const
{
    vector<string> relPathVec = formRelativePathVec(otherPath);
    return ComponentPath(relPathVec, false);
}

ComponentPath ComponentPath::getParentPath() const
{
    vector<string> parentPathVec = getParentPathVec();
    return ComponentPath(parentPathVec, isAbsolute());
}

std::string ComponentPath::getParentPathString() const
{
    return getParentPath().toString();
}

std::string ComponentPath::getSubcomponentNameAtLevel(size_t index) const
{
    return getPathElement(index);
}

std::string ComponentPath::getComponentName() const
{
    if (getNumPathLevels() == 0) {
        std::string emptyStr{};
        return emptyStr;
    }

    return getSubcomponentNameAtLevel(getNumPathLevels() - 1);
}
