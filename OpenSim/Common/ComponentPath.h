#ifndef OPENSIM_COMPONENT_PATH_H_
#define OPENSIM_COMPONENT_PATH_H_
/* -------------------------------------------------------------------------- *
 *                      OpenSim: ComponentPath.h                              *
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

#include "osimCommonDLL.h"

#include <string>
#include <vector>

namespace OpenSim {

/**
 * A representation of a path within a Component tree.
 *
 * This class is effectively a wrapper around a normalized path string. A path
 * string is a sequence of path elements interspersed with '/' as a separator.
 * path elements cannot contain:
 *
 * - back-slash ('\\')
 * - forward-slash ('/')
 * - asterisk ('*')
 * - plus-sign ('+')
 *
 * An empty path, "", is allowed. Adjacent separators in a path (e.g. "//") are
 * combined into one separator.
 *
 * @author Carmichael Ong
 */
class OSIMCOMMON_API ComponentPath {
private:
    std::string _path;

public:
    /**
     * Default constructor that constructs an empty path ("").
     */
    ComponentPath();

    /**
     * Construct a ComponentPath from a path string (e.g. "/a/b/component").
     */
    ComponentPath(std::string path);

    /**
     * Construct a ComponentPath from a vector of its elements.
     *
     * Throws if any element in `pathVec` contains an invalid character.
     */
    ComponentPath(const std::vector<std::string>& pathVec, bool isAbsolute);

    bool operator==(const ComponentPath&) const;
    bool operator!=(const ComponentPath&) const;

    char getSeparator() const;

    /**
     * Returns a string containing a sequence of all invalid characters.
     */
    const std::string& getInvalidChars() const;

    /**
     * Returns a path that is the result of resolving `this` from `otherPath`.
     *
     * - `otherPath` must be an absolute path; otherwise, an exception will be
     *   thrown
     * - if `this` is absolute, then this function just returns a copy of `this`
     *
     * Examples:
     *
     *     ComponentPath{"b/c"}.formAbsolutePath("/a") == "/a/b/c"
     *     ComponentPath{"/b/c"}.formAbsolutePath("/a") == "/b/c"
     *     ComponentPath{"b/c"}.formAbsolutePath("a")  // throws
     */
    ComponentPath formAbsolutePath(const ComponentPath& otherPath) const;

    /**
     * Find the relative Path between this Path and another Path (otherPath)
     * (i.e. the Path to go FROM otherPath TO this Path). Both Paths must be
     * absolute.
     */
    ComponentPath formRelativePath(const ComponentPath& otherPath) const;

    /**
     * Returns the sub-path that contains all subdirectory levels except for
     * the last one.
     */
    ComponentPath getParentPath() const;

    /**
     * Returns the parent path as a string.
     */
    std::string getParentPathString() const;

    /**
     * Returns the name of a subdirectory in the path at the specified level
     * (0-indexed).
     */
    std::string getSubcomponentNameAtLevel(size_t index) const;

    /**
     * Returns the name of the Component in the path (effectively, the last
     * element in the path).
     */
    std::string getComponentName() const;

    /**
     * Returns a string representation of the ComponentPath
     * (e.g. "/a/b/component").
     */
    const std::string& toString() const;

    /**
     * Returns true if the path is absolute (effectively, if it begins
     * with '/').
     */
    bool isAbsolute() const;

    /**
     * Returns the number of levels in the path (e.g. "/a/b/c" == 3).
     */
    size_t getNumPathLevels() const;

    /**
     * Push a string onto the end of the path.
     *
     * Throws if the argument contains invalid characters.
     */
    void pushBack(const std::string& pathElement);

    /**
     * Returns true if the argument does not contain any invalid characters.
     */
    bool isLegalPathElement(const std::string& pathElement) const;

    /**
     * Resolves '.' and ".." elements in the path if possible. Leading ".."
     * elements are allowed only in relative paths (throws if found at the
     * start of an absolute path). Also checks for invalid characters.
     *
     * Effectively, this is the same as internally `normalize`ing the path.
     */
    void trimDotAndDotDotElements() {
        // noop: here for legacy purposes: the path is always
        // internally normalized
    }
};
} // end of namespace OpenSim
#endif // OPENSIM_COMPONENT_PATH_H_
