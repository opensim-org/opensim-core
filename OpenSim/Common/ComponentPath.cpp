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
static const char invalidCharGlyphs[] = { '\\', '*', '+', ' ', '\t', '\n' };
const std::string ComponentPath::invalidChars = "\\/*+ \t\n";

static bool isValid(const std::string& path) {
    for (char c : invalidCharGlyphs) {
        if (path.find(c) != std::string::npos) {
            return false;
        }
    }
    return true;
}

std::string ComponentPath::normalize(std::string path) {
    // TODO: this is gruesome, and needs tidying up, but it has some handy
    // properties:
    //
    // - Pure function with an input and an output (easy to test)
    // - Has tests
    // - Is only reliant on standard library and raw language features, so the
    //   entire implementation is very standalone
    // - No allocations
    // - Extremely cache-friendly

    if (!isValid(path)) {
        throw std::runtime_error{path + ": path contains invalid characters"};
    }

    // shifts a C-string `s` leftwards `n` places, such that `s` then
    // points to the characters that were at `s+n`
    // TODO: replace with something like std::rotate
    auto shift = [&](char* s, size_t n) {
        size_t len = path.size() - (path.c_str() - s);
        size_t to_move = len-n;
        for (size_t i = 0; i < to_move; ++i) {
            s[i] = s[i+n];
        }
        path.resize(path.size() - n);
    };

    // searches for '/' rightwards in the range [start, end) backwards,
    // returns `nullptr` if '/' is not found in [start, end)
    auto rfind = [](char* start, char* end) {
        while (--end >= start) {
            if (*end == '/') {
                return end;
            }
        }
        return static_cast<char*>(nullptr);
    };

    char* start = &path[0];

    // setup loop invariants:
    //
    // - skip starting slash (absolute paths)
    //
    // - skip starting '..' elements, unless it is an absolute path, then throw
    //   an error
    //
    // - remove any starting '.' relative elements './././a/b' --> 'a/b'
    //
    // - after this setup phase, the remainder of the algorithm can assume
    //   that any slashes in [start, cur) delimit a real previous element
    //   (effectively, the alg can ignore the prefix)

    bool isAbsolute = false;
    if (start[0] == '/') {
        ++start;
        isAbsolute = true;
    }

    // skip/shift leading relative elements
    while (start[0] == '.') {
        if (start[1] == '/') {
            shift(start, 2);
        } else if (start[1] == '\0') {
            shift(start, 1);
        } else if (start[1] == '.' && (start[2] == '/' || start[2] == '\0')) {
            if (isAbsolute) {
                throw std::runtime_error{path + ": invalid path: is absolute, but starts with relative elements"};
            }
            // skip leading '..' elements
            if (start[2] == '/') {
                start += 3;
            } else {
                start += 2;
            }
        }
    }

    // [start, cur) delimits a fully-resolved path string that contains no
    // leading slash and no relative elements.
    char* cur = start;

    // iterate through the string, with `cur` as the cursor, and resolve any
    // relative elements, such that the [start, cur) loop invariant is
    // maintained.
    while (cur[0] != '\0') {
        if (cur[0] == '.') {
            if (cur[1] == '\0' || cur[1] == '/') {
                // it's a '.'
                shift(cur, 2);
                if (cur != start) {
                    --cur;
                }
                continue;
            }

            if (cur[1] == '.' && (cur[2] == '\0' || cur[2] == '/')) {
                // it's a '..'

                if (cur == start) {
                    throw std::runtime_error{path + ": cannot handle '..' element in string: would hop above the root of the path"};
                }

                char* prevElEnd = cur-1;
                char* prevElStart = rfind(start, prevElEnd);
                if (prevElStart == nullptr) {
                    prevElStart = start;
                }
                size_t n = (prevElEnd - prevElStart) + (cur[2] == '/' ? 4 : 3); // '/a/..'
                cur = prevElStart;
                shift(cur, n);
                continue;
            }
        }

        // relative elements handled: skip to the next element in the input, or
        // the end of the input
        while (cur[0] != '\0') {
            if (*cur++ == '/') {
                // consecutive slashes are combined into one slash
                while (cur[0] == '/') {
                    shift(cur, 1);
                }
                break;
            }
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
