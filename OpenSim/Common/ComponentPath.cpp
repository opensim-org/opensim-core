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

#include "Exception.h"
#include <algorithm>
#include <sstream>


static const std::string newInvalidChars{"\\*+ \t\n"};
static const char separator = '/';
static const std::string legacyInvalidChars = newInvalidChars + '/';


using OpenSim::ComponentPath;

namespace {
    // helper: joins the supplied vector of path components to form a single path string
    std::string stringifyPath(const std::vector<std::string>& pathVec, bool isAbsolute) {
        std::string ret;
        if (isAbsolute) {
            ret += '/';
        }
        if (pathVec.empty()) {
            return ret;
        }
        for (size_t i = 0; i < (pathVec.size()-1); ++i) {
            ret += pathVec[i] + '/';
        }
        ret += pathVec.back();
        return ret;
    }

    // helper: returns a string iterator that points to the first component (i.e. non-separator)
    // in a normalized path string. Points to `.end()` if the path contains no components
    auto firstComponent(const std::string& normalizedPath) -> decltype(normalizedPath.begin()) {
        if (normalizedPath.empty()) {
            return normalizedPath.end();
        }

        auto it = normalizedPath.begin();
        if (normalizedPath[0] == '/') {
            ++it;
        }

        return it;
    }
}

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
        OPENSIM_THROW(Exception, path + ": path contains invalid characters");
    }

    // helper: shift chars starting at `offset+n` `n` characters left
    auto shift = [&](size_t offset, size_t n) {
        std::copy(path.begin() + offset + n, path.end(), path.begin() + offset);
        path.resize(path.size() - n);
    };

    // remove duplicate adjacent separators
    for (size_t i = 0; i < path.size(); ++i) {
        if (path[i] == separator && path[i+1] == separator) {
            shift(i, 1);
            --i;
        }
    }

    // skip absolute slash
    bool isAbsolute = path.front() == separator;
    size_t contentStart = isAbsolute ? 1 : 0;

    // skip/dereference relative elements at the start of a path
    while (path[contentStart] == '.') {
        switch (path[contentStart + 1]) {
            case separator:
                shift(contentStart, 2);
                break;
            case '\0':
                shift(contentStart, 1);
                break;
            case '.': {
                char c2 = path[contentStart + 2];
                if (c2 == separator || c2 == '\0') {
                    // starts with '..' element: only allowed if the path is relative
                    if (isAbsolute) {
                        OPENSIM_THROW(Exception,
                                      path + ": invalid path: is absolute, but starts with relative elements");
                    }

                    // if not absolute, then make sure `contentStart` skips past these
                    // elements because the alg can't reduce them down
                    if (c2 == separator) {
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
        if (c0 == '.' && (c1 == '\0' || c1 == separator)) {
            shift(offset, c1 == separator ? 2 : 1);
            if (offset != contentStart) {
                --offset;
            }
            continue;
        }

        // handle '..' (if found)
        if (c0 == '.' && c1 == '.' && (c2 == '\0' || c2 == separator)) {
            if (offset == contentStart) {
                OPENSIM_THROW(Exception, path + ": cannot handle '..' element in string: would hop above the root of the path");
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
    if (path.size() > 1 && path.back() == separator) {
        path.pop_back();
    }

    if (!isAbsolute && path.size() == 1 && path.back() == separator) {
        path.pop_back();
    }

    return path;
}

ComponentPath::ComponentPath() = default;

ComponentPath::ComponentPath(std::string path) : _path{normalize(std::move(path))} {
}

ComponentPath::ComponentPath(const std::vector<std::string>& pathVec, bool isAbsolute) :
    _path{normalize(stringifyPath(pathVec, isAbsolute))} {
}

bool ComponentPath::operator==(const ComponentPath& other) const {
    return _path == other._path;
}

bool ComponentPath::operator!=(const ComponentPath& other) const {
    return _path != other._path;
}

char ComponentPath::getSeparator() const {
    return separator;
}

const std::string& ComponentPath::getInvalidChars() const {
    return legacyInvalidChars;
}

ComponentPath ComponentPath::formAbsolutePath(const ComponentPath& otherPath) const {
    if (this->isAbsolute()) {
        return *this;
    }

    if (!otherPath.isAbsolute()) {
        OPENSIM_THROW(Exception, otherPath._path + ":  must be an absolute path");
    }

    return ComponentPath{otherPath._path + separator + _path};
}

OpenSim::ComponentPath OpenSim::ComponentPath::formRelativePath(const ComponentPath& otherPath) const {
    // helper: find a separator to the left of s.begin() + offset
    static auto offsetOfSeparator = [](const std::string& s, size_t offset) {
        auto offsetFromEnd = std::find_if(s.rend() - (offset + 1), s.rend(), [](const char c) {
            return c == separator || c == '\0';
        });
        assert(offsetFromEnd != s.rend());  // because we know there is a root prefix
        auto distance = std::distance(offsetFromEnd, s.rend());
        // in this instance, distance is 1-indexed, so subtract 1 to make it a 0-indexed offset
        return distance-1;
    };

    // helper: returns the index of the first character where s1[index] != s2[index]
    //         returns 0 for empty inputs, s1.size() for identical inputs
    static auto mismatchOffset = [](const std::string& s1, const std::string& s2) {
        const size_t shortest = std::min(s1.size(), s2.size());
        size_t offset = 0;
        while (offset < shortest) {
            if (s1[offset] != s2[offset]) {
                break;
            }
            ++offset;
        }
        return offset;
    };


    if (!isAbsolute()) {
        OPENSIM_THROW(Exception, _path + ": is not absolute");
    }

    if (!otherPath.isAbsolute()) {
        OPENSIM_THROW(Exception, _path + ": is not absolute");
    }

    // for readability: we are going FROM p1 and TO p2 by stepping up in P1 to the common
    //                  root and then stepping down into p2
    const std::string p1 = otherPath._path;
    const std::string& p2 = this->_path;

    // find the point at which (if any) that p1 and p2 lexographically mismatch
    const size_t mismatchStart = mismatchOffset(p1, p2);

    // `mismatchStart` is now the index of the *string* divergence point between p1 and p2.
    // Because of the absolute requirement (above), we know that mismatchStart > 0 and that
    // a reverse search in both strings from `mismatchStart` backwards for the separator
    // will definitely find a separator (at worst, the root)
    size_t p1Start = offsetOfSeparator(p1, mismatchStart);
    size_t p2Start = offsetOfSeparator(p2, mismatchStart);

    // `pXstart` is now the index of the *tree* divergence point between p1 and p2. The remaining
    // logic is:
    // - the number of "step ups" is the number of separators in p1 after the divergence point
    // - the "step downs" is simply everything after the divergence point in P2
    size_t stepUps = std::count(p1.begin() + p1Start, p1.end(), separator);

    std::string rv;
    for (size_t i = 0; i < stepUps; ++i) {
        rv += "../";
    }
    if (p2Start < p2.size()) {
        rv += p2.substr(p2Start + 1);
    }

    return ComponentPath{rv};
}

OpenSim::ComponentPath OpenSim::ComponentPath::getParentPath() const {
    return ComponentPath{getParentPathString()};
}

std::string ComponentPath::getParentPathString() const {
    auto end = _path.rend() - (isAbsolute() ? 1 : 0);
    auto it = std::find(_path.rbegin(), end, separator);

    if (it == end) {
        // no parent: emulate existing behavior
        return "";
    }

    it++;  // skip past the found slash

    size_t len = std::distance(it, _path.rend());
    return _path.substr(0, len);
}

std::string ComponentPath::getSubcomponentNameAtLevel(size_t index) const {
    auto componentStart = firstComponent(_path);

    if (std::distance(componentStart, _path.end()) == 0) {
        OPENSIM_THROW(Exception, "Cannot index into this path: it is empty");
    }

    size_t i = 0;
    auto componentEnd = std::find(componentStart, _path.end(), separator);

    while (i < index) {
        if (componentEnd == _path.end()) {
            std::stringstream msg;
            msg << _path << ": invalid index '" << index << "'";
            OPENSIM_THROW(Exception, msg.str());
        }

        componentStart = componentEnd + 1;  // skip past last found separator
        componentEnd = std::find(componentStart, _path.end(), separator);
        ++i;
    }

    return std::string{componentStart, componentEnd};
}

std::string ComponentPath::getComponentName() const {
    if (std::distance(firstComponent(_path), _path.end()) == 0) {
        return {};
    }

    size_t start = _path.rfind(separator);

    if (start == std::string::npos) {
        start = 0;
    } else {
        ++start;  // skip the found separator
    }

    return _path.substr(start);
}

std::string ComponentPath::toString() const {
    return _path;
}

bool ComponentPath::isAbsolute() const {
    return !_path.empty() && _path[0] == separator;
}

size_t ComponentPath::getNumPathLevels() const {
    auto begin = _path.begin();
    if (isAbsolute()) {
        ++begin;
    }

    return std::distance(begin, _path.end())  > 0 ?
           std::count(begin, _path.end(), separator) + 1 :
           0;
}

void ComponentPath::pushBack(const std::string& pathElement) {
    if (pathElement.empty()) {
        OPENSIM_THROW(Exception, "cannot append an empty path element to a path");
    }

    // ensure `path` contains no invalid chars
    if (!isLegalPathElement(pathElement)) {
        OPENSIM_THROW(Exception, pathElement + ": path element contains invalid characters");
    }

    // an additional separator should be added between the existing path
    // and the new element *unless*:
    // - the existing path was empty
    // - the existing path was just the root
    if (!_path.empty() && _path.back() != separator) {
        _path += separator;
    }
    _path += pathElement;
}

bool ComponentPath::isLegalPathElement(const std::string &pathElement) const {
    return pathElement.find_first_of(legacyInvalidChars) == std::string::npos;
}
