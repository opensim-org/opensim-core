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

using OpenSim::ComponentPath;

namespace {
    const std::string newInvalidChars{"\\*+ \t\n"};
    constexpr char separator = '/';
    const std::string legacyInvalidChars = newInvalidChars + separator;
    constexpr std::string::value_type nul = {};

    /**
     * Returns all components in `pathVec` joined by the path separator
     */
    std::string stringifyPath(const std::vector<std::string>& pathVec,
                              bool isAbsolute) {
        std::string ret;

        if (isAbsolute) {
            ret += separator;
        }

        if (pathVec.empty()) {
            return ret;
        }

        for (size_t i = 0; i < (pathVec.size()-1); ++i) {
            ret += pathVec[i];
            ret += separator;
        }
        ret += pathVec.back();

        return ret;
    }

    /**
     * helper: returns a string iterator that points to the first component
     * (i.e. non-separator) in a normalized path string. Returns `.end()`
     * if the path contains no components.
     */
    std::string::const_iterator firstComponentIn(const std::string& normalizedPath) {
        if (normalizedPath.empty()) {
            return normalizedPath.end();
        }

        auto it = normalizedPath.begin();
        if (normalizedPath[0] == separator) {
            ++it;
        }

        return it;
    }

    /**
     * Returns a normalized form of `path`. A normalized path string is
     * guaranteed to:
     *
     * - Not contain any *internal* or *trailing* relative elements (e.g.
     *   'a/../b').
     *
     *     - It may *start* with relative elements (e.g. '../a/b'), but only
     *       if the path is non-absolute (e.g. "/../a/b" is invalid)
     *
     * - Not contain any invalid characters (e.g. '\\', '*')
     *
     * - Not contain any repeated separators (e.g. 'a///b' --> 'a/b')
     *
     * Any attempt to step above the root of the expression with '..' will
     * result in an exception being thrown (e.g. "a/../.." will throw).
     *
     * This method is useful for path traversal and path manipulation
     * methods, because the above ensures that (e.g.) paths can be
     * concatenated and split into individual elements using basic
     * string manipulation techniques.
     */
    std::string normalize(std::string path) {
        // note: this implementation is fairly low-level and involves mutating
        //       `path` quite a bit. The test suite is heavily relied on for
        //       developing this kind of tricky code.
        //
        //       the reason it's done this way is because profiling shown that
        //       `ComponentPath` was accounting for ~6-10 % of OpenSim's CPU
        //       usage for component-heavy sims. The reason this was so high
        //       was because `ComponentPath` used a simpler algorithm that
        //       split the path into a std::vector.
        //
        //       usually, that alg. wouldn't be a problem, but path
        //       normalization can happen millions of times during a simulation,
        //       all those vector allocations can thrash the allocator and
        //       increase L1 misses.

        // assert that `path` contains no invalid chars
        if (path.find_first_of(newInvalidChars) != std::string::npos) {
            OPENSIM_THROW(OpenSim::Exception, path + ": The supplied path contains invalid characters.");
        }

        // pathEnd is guaranteed to be a NUL terminator since C++11
        char* pathBegin = &path[0];
        char* pathEnd = &path[path.size()];

        // helper: shift n chars starting at newStart+n such that, after,
        // newStart..end is equal to what newStart+n..end was before.
        auto shift = [&](char* newStart, size_t n) {
            std::copy(newStart + n, pathEnd, newStart);
            pathEnd -= n;
        };

        // helper: grab 3 lookahead chars, using NUL as a senteniel to
        // indicate "past the end of the content".
        //
        // - The maximum lookahead is 3 characters because the parsing
        //   code below needs to be able to detect the upcoming input 
        //   pattern "..[/\0]"
        struct Lookahead { char a, b, c; };
        auto getLookahead = [](char* start, char* end) {
            return Lookahead{
                start < end - 0 ? start[0] : nul,
                start < end - 1 ? start[1] : nul,
                start < end - 2 ? start[2] : nul,
            };
        };


        // remove duplicate adjacent separators
        for (char* c = pathBegin; c != pathEnd; ++c) {
            Lookahead l = getLookahead(c, pathEnd);
            if (l.a == separator && l.b == separator) {
                shift(c--, 1);
            }
        }

        bool isAbsolute = *pathBegin == separator;
        char* cursor = isAbsolute ? pathBegin + 1 : pathBegin;

        // skip/dereference relative elements *at the start of a path*
        Lookahead l = getLookahead(cursor, pathEnd);
        while (l.a == '.') {
            switch (l.b) {
            case separator:
                shift(cursor, 2);
                break;
            case nul:
                shift(cursor, 1);
                break;
            case '.': {
                if (l.c == separator || l.c == nul) {
                    // starts with '..' element: only allowed if the path
                    // is relative
                    if (isAbsolute) {
                        OPENSIM_THROW(OpenSim::Exception, path + ": is an invalid path: it is absolute, but starts with relative elements.");
                    }

                    // if not absolute, then make sure `contentStart` skips
                    // past these elements because the alg can't reduce
                    // them down
                    if (l.c == separator) {
                        cursor += 3;
                    } else {
                        cursor += 2;
                    }
                } else {
                    // normal element that starts with '..'
                    ++cursor;
                }
                break;
            }
            default:
                // normal element that starts with '.'
                ++cursor;
                break;
            }

            l = getLookahead(cursor, pathEnd);
        }

        char* contentStart = cursor;

        // invariants:
        //
        // - the root path element (if any) has been skipped
        // - `contentStart` points to the start of the non-relative content of
        //   the supplied path string
        // - `path` contains no duplicate adjacent separators
        // - `[0..offset]` is normalized path string, but may contain a
        //   trailing slash
        // - `[contentStart..offset] is the normalized *content* of the path
        //   string

        while (cursor < pathEnd) {
            Lookahead l = getLookahead(cursor, pathEnd);

            if (l.a == '.' && (l.b == nul || l.b == separator)) {
                // handle '.' (if found)
                size_t charsInCurEl = l.b == separator ? 2 : 1;
                shift(cursor, charsInCurEl);

            } else if (l.a == '.' && l.b == '.' && (l.c == nul || l.c == separator)) {
                // handle '..' (if found)

                if (cursor == contentStart) {
                    OPENSIM_THROW(OpenSim::Exception, path + ": cannot handle '..' element in a path string: dereferencing this would hop above the root of the path.");
                }

                // search backwards for previous separator
                char* prevSeparator = cursor - 2;
                while (prevSeparator > contentStart && *prevSeparator != separator) {
                    --prevSeparator;
                }

                char* prevStart = prevSeparator <= contentStart ? contentStart : prevSeparator + 1;
                size_t charsInCurEl = (l.c == separator) ? 3 : 2;
                size_t charsInPrevEl = cursor - prevStart;

                cursor = prevStart;
                shift(cursor, charsInPrevEl + charsInCurEl);

            } else {
                // non-relative element: skip past the next separator or end
                cursor = std::find(cursor, pathEnd, separator) + 1;
            }
        }

        // edge case:
        // - There was a trailing slash in the input and, post reduction, the output
        //   string is only a slash. However, the input path wasnt initially an
        //   absolute path, so the output should be "", not "/"
        {
            char* beg = isAbsolute ? pathBegin + 1 : pathBegin;
            if (pathEnd - beg > 0 && pathEnd[-1] == separator) {
                --pathEnd;
            }
        }

        // resize output to only contain the normalized range
        path.resize(pathEnd - pathBegin);

        return path;
    }
}

ComponentPath::ComponentPath() = default;

ComponentPath::ComponentPath(std::string path) :
    _path{normalize(std::move(path))} {
}

ComponentPath::ComponentPath(const std::vector<std::string>& pathVec,
                             bool isAbsolute) :
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
        OPENSIM_THROW(Exception, otherPath._path + ":  must be an absolute path.");
    }

    return ComponentPath{otherPath._path + separator + _path};
}

ComponentPath ComponentPath::formRelativePath(const ComponentPath& otherPath) const {
    // helper: returns a path string containing `n` step ups (e.g. "../../")
    static auto generateStepUps = [](size_t n) {
        std::string rv;
        for (size_t i = 0; i < n; ++i) {
            rv += "..";
            rv += separator;
        }
        return rv;
    };

    if (!isAbsolute()) {
        OPENSIM_THROW(Exception, _path + ": is not an absolute path.");
    }

    if (!otherPath.isAbsolute()) {
        OPENSIM_THROW(Exception, _path + ": is not an absolute path.");
    }

    // readability: the resulting path goes FROM p1 and TO p2
    const std::string& p1 = otherPath._path;
    const std::string& p2 = this->_path;

    // compute the lexographic mismatch between p1 and p2
    const size_t mismatch = [&]() {
        auto shortest = static_cast<std::string::difference_type>(std::min(p1.size(), p2.size()));
        auto p = std::mismatch(p1.begin(), p1.begin() + shortest, p2.begin());
        return static_cast<size_t>(std::distance(p1.begin(), p.first));
    }();

    // handle edge cases: see test suite for example inputs
    ComponentPath rv;

    if (p1[mismatch] == nul && p2[mismatch] == nul) {
        // p1 == p2
        rv = ComponentPath{""};
    } else if (p1.size() == 1 && p1[0] == separator) {
        // p1 is just "/", p2 is defined to be a direct subpath beginning after
        // the "/"
        rv = ComponentPath{p2.substr(1)};
    } else if (p1[mismatch] == nul && p2[mismatch] == separator) {
        // p2 is a direct subpath of p1, so only step down
        rv = ComponentPath{p2.substr(mismatch + 1)};
    } else if (p1[mismatch] == separator && p2[mismatch] == nul) {
        // p1 is a direct subpath of p2, so only step up
        size_t stepUps = std::count(p1.begin() + mismatch, p1.end(), separator);
        rv = ComponentPath{generateStepUps(stepUps)};
    } else {
        // There is a divergence between the two paths. Step up to the
        // divergence point (dir) then step down to the target
        size_t divergencePoint = p1.rfind(separator, mismatch - 1);

        // step up in p1 and then step down in p2
        size_t stepUps = std::count(p1.begin() + divergencePoint, p1.end(), separator);
        std::string path = generateStepUps(stepUps);
        path += p2.substr(divergencePoint + 1);

        rv = ComponentPath{std::move(path)};
    }

    return rv;
}

OpenSim::ComponentPath OpenSim::ComponentPath::getParentPath() const {
    return ComponentPath{getParentPathString()};
}

std::string ComponentPath::getParentPathString() const {
    auto end = _path.rend() - (isAbsolute() ? 1 : 0);
    auto it = std::find(_path.rbegin(), end, separator);

    if (it == end) {
        return "";  // no parent
    }

    it++;  // skip past the found slash

    size_t len = std::distance(it, _path.rend());
    return _path.substr(0, len);
}

std::string ComponentPath::getSubcomponentNameAtLevel(size_t index) const {
    auto firstComponent = firstComponentIn(_path);

    if (firstComponent == _path.end()) {
        OPENSIM_THROW(Exception, "Cannot index into this path: it is empty.");
    }

    size_t i = 0;
    auto componentEnd = std::find(firstComponent, _path.end(), separator);

    while (i < index) {
        if (componentEnd == _path.end()) {
            std::stringstream msg;
            msg << _path << ": invalid index '" << index << "' into this path.";
            OPENSIM_THROW(Exception, msg.str());
        }

        firstComponent = componentEnd + 1;  // skip past last found separator
        componentEnd = std::find(firstComponent, _path.end(), separator);
        ++i;
    }

    return std::string{firstComponent, componentEnd};
}

std::string ComponentPath::getComponentName() const {
    if (firstComponentIn(_path) == _path.end()) {
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

const std::string& ComponentPath::toString() const {
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

    if (begin != _path.end()) {
        size_t numSeparators = std::count(begin, _path.end(), separator);
        return numSeparators + 1;
    } else {
        return 0;
    }
}

void ComponentPath::pushBack(const std::string& pathElement) {
    if (pathElement.empty()) {
        OPENSIM_THROW(Exception, "Cannot pushBack an empty path element to a ComponentPath object.");
    }

    // ensure `path` contains no invalid chars
    if (!isLegalPathElement(pathElement)) {
        OPENSIM_THROW(Exception, pathElement + ": provided path element contains invalid characters.");
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

bool ComponentPath::isLegalPathElement(const std::string& pathElement) const {
    return pathElement.find_first_of(legacyInvalidChars) == std::string::npos;
}
