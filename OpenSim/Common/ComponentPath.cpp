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
static const std::string newInvalidChars = "\\*+ \t\n";

std::string ComponentPath::normalize(std::string path) {
    // note: this implementation is fairly low-level and involves mutating `path` quite a bit.
    //       the test suite is heavily relied on for developing this kind of tricky code.
    //
    //       the reason it's done this way is because profiling shown that `ComponentPath` was
    //       accounting for ~6-10 % of OpenSim's CPU usage for component-heavy sims. The reason
    //       this was so high was because `ComponentPath` used a simpler algorithm that split
    //       the path into a std::vector.
    //
    //       usually, that alg. wouldn't be a problem, but path normalization can happen millions
    //       of times during a simulation, all those vector allocations can thrash the allocator and
    //       increase L1 misses

    // ensure `path` contains no invalid chars
    if (path.find_first_of(newInvalidChars) != std::string::npos) {
        throw std::runtime_error{path + ": path contains invalid characters"};
    }

    // helper: shift chars starting at `offset+n` `n` characters left
    auto shift = [&](size_t offset, size_t n) {
        std::copy(path.begin() + offset + n, path.end(), path.begin() + offset);
        path.resize(path.size() - n);
    };

    // remove duplicate adjacent slashes
    for (size_t i = 0; i < path.size(); ++i) {
        if (path[i] == '/' && path[i+1] == '/') {
            shift(i, 1);
            --i;
        }
    }

    // skip absolute slash
    bool isAbsolute = path.front() == '/';
    size_t contentStart = isAbsolute ? 1 : 0;

    // skip/dereference relative elements at the start of a path
    while (path[contentStart] == '.') {
        switch (path[contentStart + 1]) {
            case '/':
                shift(contentStart, 2);
                break;
            case '\0':
                shift(contentStart, 1);
                break;
            case '.': {
                char c2 = path[contentStart + 2];
                if (c2 == '/' || c2 == '\0') {
                    // starts with '..' element: only allowed if the path is relative
                    if (isAbsolute) {
                        OPENSIM_THROW(Exception,
                                      path + ": invalid path: is absolute, but starts with relative elements");
                    }

                    // if not absolute, then make sure `contentStart` skips past these
                    // elements because the alg can't reduce them down
                    if (c2 == '/') {
                        contentStart += 3;
                    } else {
                        contentStart += 2;
                    }
                } else {
                    // normal element that starts with '..'
                    ++contentStart;
                }
                break;
            }
            default:
                // normal element that starts with '..'
                ++contentStart;
                break;
        }
    }

    size_t offset = contentStart;

    while (offset < path.size()) {
        // this parser has a <= 2-char lookahead
        char c0 = path[offset];
        char c1 = c0 != '\0' ? path[offset+1] : '\0';
        char c2 = c1 != '\0' ? path[offset+2] : '\0';

        // handle '.' (if found)
        if (c0 == '.' && (c1 == '\0' || c1 == '/')) {
            shift(offset, c1 == '/' ? 2 : 1);
            if (offset != contentStart) {
                --offset;
            }
            continue;
        }

        // handle '..' (if found)
        if (c0 == '.' && c1 == '.' && (c2 == '\0' || c2 == '/')) {
            if (offset == contentStart) {
                throw std::runtime_error{path + ": cannot handle '..' element in string: would hop above the root of the path"};
            }
            auto contentStartIt = path.rend() - contentStart;
            size_t prevEnd = offset - 1;
            size_t prevEndReverseOffset = path.size() - prevEnd;
            auto it = std::find(path.rbegin() + prevEndReverseOffset, contentStartIt, '/');
            size_t prevStart = it == contentStartIt ? contentStart : std::distance(it, path.rend());
            size_t n = (prevEnd - prevStart) + (c2 == '/' ? 4 : 3);
            offset = prevStart;
            shift(offset, n);
            continue;
        }

        offset = path.find('/', offset);
        if (offset == std::string::npos) {
            break;  // end of input
        } else {
            ++offset;  // skip the slash
        }
    }

    // edge-case: trailing slashes should be removed, unless the string only
    // contains a slash
    if (path.size() > 1 && path.back() == '/') {
        path.pop_back();
    }

    if (!isAbsolute && path.size() == 1 && path.back() == '/') {
        path.pop_back();
    }

    return path;
}

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
