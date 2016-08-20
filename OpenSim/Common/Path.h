#ifndef OPENSIM_PATH_H_
#define OPENSIM_PATH_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSim: Path.h                                  *
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

#include "OpenSim/Common/Object.h"

namespace OpenSim {

//==============================================================================
//                                 OPENSIM PATH
//==============================================================================
/**
* An abstract class for handling a Path. A Path refers to a list of strings that
* represent a different level in a hierarchical structure. Each level is divided
* by designated separators (e.g., "/" or "\").
*
* This class stores the list of levels as a vector of strings. One char is used
* to denote a path separator when either reading in or writing the Path out to
* a string. A bool is stored to determine whether this Path should be resolved
* relative to the root (denoted with a leading separator char) or not.
*
* @author Carmichael Ong
*/
class OSIMCOMMON_API Path : public Object {
    OpenSim_DECLARE_ABSTRACT_OBJECT(Path, Object);

public:
    /// Default constructor
    Path() = delete;

    /// Construct Path from a string, given separator character and a string
    /// of invalidChars
    Path(const std::string path, 
         const char separator, 
         const std::string invalidChars);

    /// Construct Path from a vector of strings (pathVec), given separator
    /// character and a string of invalidChars
    Path(std::vector<std::string> pathVec,
         const char separator,
         const std::string invalidChars,
         bool isAbsolute);

    /// Use default copy constructor and assignment operator.
    Path(const Path&) = default;
    Path& operator=(const Path&) = default;

    /// Destructor.
    ~Path() = default;

protected:
    /// Cleans up a path. This includes removing "." and resolving ".." if
    /// possible (i.e. it will not remove leading ".." but otherwise will
    /// remove the previous pathElement from _path.
    void cleanPath();

    /// Write out the path to a string with each element separated by the
    /// specified separator.
    std::string getString();

    /// Get an absolute path by resolving it relative to a given otherPath.
    /// If the current Path is already absolute, return the same Path.
    std::vector<std::string> getAbsolutePathVec(Path* otherPath);

    /// Find the relative Path between this Path and another Path (otherPath).
    /// Both Paths must be absolute.
    std::vector<std::string> findRelativePathVec(Path* otherPath);

private:
    size_t getPathLength() { return _path.size(); };
    std::vector<std::string> getPath() { return _path; };
    std::string getPathElement(int index) { return getPath()[index]; };


    // Verify that a pathElement does not contain any chars from the list
    // of _invalidChars
    bool isLegalPathElement(const std::string pathElement);

    // Append a pathElement to _path, first checking if the pathElement
    // is legal
    void appendPathElement(const std::string pathElement);

    // Path variables
    std::vector<std::string> _path;
    char _separator;
    std::string _invalidChars;
    bool _isAbsolute;
}; // end class Path

} // end of namespace OpenSim

#endif // OPENSIM_PATH_H_
