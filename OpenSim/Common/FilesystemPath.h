#ifndef OPENSIM_FILESYSTEM_PATH_H_
#define OPENSIM_FILESYSTEM_PATH_H_
/* -------------------------------------------------------------------------- *
 *                             OpenSim: FilesystemPath.h                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
 * Author(s): Chris Dembia                                                    *
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
#include <iostream>

namespace OpenSim {
class OSIMCOMMON_API FilesystemPath {
public:
    FilesystemPath() {}
    explicit FilesystemPath(const std::string& pathname)
        : m_pathname(pathname) {}
    const std::string& get() const {
        // TODO rename.
        return m_pathname;
    }

    bool operator==(const FilesystemPath& other) const {
        // TODO
        return m_pathname == other.m_pathname;
    }

private:
    std::string m_pathname;
};

OSIMCOMMON_API inline std::ostream& operator<<(std::ostream& o,
        const FilesystemPath& path) {
    if (path.get().find(' ') == std::string::npos) {
        o << path.get();
    } else {
        // TODO prepend spaces with escape character. or
        // put the path in quotes.
        o << path.get();
    }
    return o;
}

} // namespace OpenSim

#endif // OPENSIM_FILESYSTEM_PATH_H_
